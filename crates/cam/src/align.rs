//! Manual registration-mark alignment.
//!
//! Print-then-cut work needs the cut to land on artwork that was printed earlier. The
//! sheet never goes back into the cutter in exactly the same place or at exactly the same
//! angle, so the design must be moved, turned and occasionally scaled to match the marks
//! that are actually on the material.
//!
//! The manual alignment-mark workflow: the operator jogs the blade to
//! each printed mark in turn and records the machine coordinate. The software then solves
//! the transform from design space to machine space. This module is that solver. It is pure
//! arithmetic — no machine access, no protocol knowledge — so it can be tested exactly.
//!
//! The transform is a *similarity*: uniform scale, rotation, and translation. A vinyl cutter
//! cannot shear or stretch a design independently in X and Y, so a full affine fit would
//! produce a matrix the machine cannot execute. Uniform scale absorbs the small size error
//! that comes from printing on a slightly stretched roll.
//!
//! Two marks fix rotation and translation exactly (the standard four-corner workflow still
//! works, it just over-determines the answer). Three or more marks add scale and average out
//! the operator's jog error; a single mark can only translate.

use crate::geom::{Point, Segment, Toolpath};

/// A similarity transform: `p' = R * p + t`, where `R` is a rotation combined with a uniform
/// scale. Stored as the four coefficients so applying it is two multiplies and two adds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Similarity {
    /// `scale * cos(theta)`.
    pub a: f64,
    /// `scale * sin(theta)`.
    pub b: f64,
    /// Translation in X, millimetres.
    pub tx: f64,
    /// Translation in Y, millimetres.
    pub ty: f64,
}

impl Similarity {
    /// The identity: no movement, no turn, no scale change.
    pub const IDENTITY: Self = Self { a: 1.0, b: 0.0, tx: 0.0, ty: 0.0 };

    /// Apply to one point.
    pub fn apply(&self, p: Point) -> Point {
        Point::new(self.a * p.x - self.b * p.y + self.tx, self.b * p.x + self.a * p.y + self.ty)
    }

    /// Uniform scale factor implied by the coefficients.
    pub fn scale(&self) -> f64 {
        (self.a * self.a + self.b * self.b).sqrt()
    }

    /// Rotation in radians implied by the coefficients.
    pub fn rotation_rad(&self) -> f64 {
        self.b.atan2(self.a)
    }
}

/// Solve the similarity that maps `design` points onto `machine` points.
///
/// Both slices must be the same length and non-empty. The answer is the least-squares fit,
/// so a 4-mark jog is averaged rather than trusting one corner. With exactly one
/// correspondence only translation can be recovered; with two, rotation and translation;
/// with three or more, scale as well.
///
/// The scale is clamped to a sane band around 1.0. An operator who mis-jogs a mark by a few
/// millimetres would otherwise produce a wildly scaled design that looks like a machine
/// fault. A sheet that is genuinely 5% long is still accommodated.
pub fn solve(design: &[Point], machine: &[Point]) -> Result<Similarity, String> {
    if design.len() != machine.len() {
        return Err(format!("alignment needs matching point counts, got {} design and {} machine", design.len(), machine.len()));
    }
    match design.len() {
        0 => Err("alignment needs at least one point".into()),
        1 => {
            // A single mark cannot reveal rotation or scale — only how far the design must
            // shift to sit on it.
            let d = design[0];
            let m = machine[0];
            Ok(Similarity { tx: m.x - d.x, ty: m.y - d.y, ..Similarity::IDENTITY })
        }
        2 => {
            // Two marks fix rotation and translation, but not scale: two points are always
            // exactly two points apart under any scale, so a scale term here would be
            // fitting noise. Pin scale to 1 and rotate onto the observed baseline.
            let d = (design[1].x - design[0].x, design[1].y - design[0].y);
            let m = (machine[1].x - machine[0].x, machine[1].y - machine[0].y);
            let dlen = (d.0 * d.0 + d.1 * d.1).sqrt();
            let mlen = (m.0 * m.0 + m.1 * m.1).sqrt();
            if dlen < 1e-9 || mlen < 1e-9 {
                return Err("alignment points are coincident; cannot determine rotation".into());
            }
            let dtheta = m.1.atan2(m.0) - d.1.atan2(d.0);
            let (sin, cos) = dtheta.sin_cos();
            Ok(Similarity {
                a: cos,
                b: sin,
                tx: machine[0].x - (cos * design[0].x - sin * design[0].y),
                ty: machine[0].y - (sin * design[0].x + cos * design[0].y),
            })
        }
        _ => solve_least_squares(design, machine),
    }
}

/// Umeyama's closed-form similarity fit. Used for three or more correspondences, where a
/// least-squares average is more trustworthy than picking two marks and ignoring the rest.
fn solve_least_squares(design: &[Point], machine: &[Point]) -> Result<Similarity, String> {
    let n = design.len() as f64;
    let (mut px, mut py, mut mx, mut my) = (0.0, 0.0, 0.0, 0.0);
    for p in design {
        px += p.x;
        py += p.y;
    }
    for m in machine {
        mx += m.x;
        my += m.y;
    }
    px /= n;
    py /= n;
    mx /= n;
    my /= n;

    let (mut sxx, mut sxy, mut syx, mut syy, mut var) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (p, m) in design.iter().zip(machine.iter()) {
        let (dx, dy) = (p.x - px, p.y - py);
        let (ex, ey) = (m.x - mx, m.y - my);
        sxx += dx * ex;
        sxy += dx * ey;
        syx += dy * ex;
        syy += dy * ey;
        var += dx * dx + dy * dy;
    }
    if var < 1e-12 {
        return Err("alignment design points are coincident; cannot determine the fit".into());
    }

    let a = (sxx + syy) / var;
    let b = (sxy - syx) / var;
    let scale = (a * a + b * b).sqrt();
    if !scale.is_finite() || !(0.5..=2.0).contains(&scale) {
        return Err(format!(
            "alignment implied a scale of {scale:.3}, which is outside the 0.5–2.0 band — \
             check that each mark was jogged to its matching printed position"
        ));
    }
    Ok(Similarity { a, b, tx: mx - (a * px - b * py), ty: my - (b * px + a * py) })
}

/// Apply a similarity to every segment of every toolpath, in place.
pub fn apply_to_toolpaths(paths: &mut [Toolpath], xf: &Similarity) {
    let scale = xf.a.hypot(xf.b);
    let theta = xf.b.atan2(xf.a);
    for tp in paths {
        for seg in &mut tp.segments {
            match seg {
                Segment::Line { a, b } => {
                    *a = xf.apply(*a);
                    *b = xf.apply(*b);
                }
                // A similarity maps a circle to a circle: the centre moves, the radius scales
                // and the start angle turns. The sweep is untouched, because angles are
                // preserved; negative sweeps stay clockwise.
                Segment::Arc { center, radius, start, sweep: _ } => {
                    *center = xf.apply(*center);
                    *radius *= scale;
                    *start += theta;
                }
                Segment::Cubic { p0, p1, p2, p3 } => {
                    *p0 = xf.apply(*p0);
                    *p1 = xf.apply(*p1);
                    *p2 = xf.apply(*p2);
                    *p3 = xf.apply(*p3);
                }
                Segment::Conic { a, apex, b, .. } => {
                    *a = xf.apply(*a);
                    *apex = xf.apply(*apex);
                    *b = xf.apply(*b);
                }
                Segment::Bezier { n, p } => {
                    let count = (*n as usize).min(p.len().saturating_sub(1)) + 1;
                    for c in p.iter_mut().take(count) {
                        *c = xf.apply(*c);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn one_point_only_translates() {
        let d = [Point::new(10.0, 20.0)];
        let m = [Point::new(15.0, 25.0)];
        let xf = solve(&d, &m).unwrap();
        assert!(close(xf.scale(), 1.0), "scale must stay 1");
        assert!(close(xf.rotation_rad(), 0.0), "rotation must stay 0");
        let out = xf.apply(Point::new(0.0, 0.0));
        assert!(close(out.x, 5.0) && close(out.y, 5.0));
    }

    #[test]
    fn two_points_rotate_without_scaling() {
        // Design baseline along +X; machine baseline along +Y. That is a 90 degree turn.
        let d = [Point::new(0.0, 0.0), Point::new(10.0, 0.0)];
        let m = [Point::new(0.0, 0.0), Point::new(0.0, 10.0)];
        let xf = solve(&d, &m).unwrap();
        assert!(close(xf.scale(), 1.0), "two points cannot reveal scale");
        assert!(close(xf.rotation_rad(), std::f64::consts::FRAC_PI_2), "expected 90 degrees, got {}", xf.rotation_rad().to_degrees());
        let out = xf.apply(Point::new(10.0, 0.0));
        assert!(close(out.x, 0.0) && close(out.y, 10.0));
    }

    #[test]
    fn two_points_translate_and_rotate() {
        // A design baseline at (5,5)->(15,5) landing on (20,30)->(20,40).
        let d = [Point::new(5.0, 5.0), Point::new(15.0, 5.0)];
        let m = [Point::new(20.0, 30.0), Point::new(20.0, 40.0)];
        let xf = solve(&d, &m).unwrap();
        // The first correspondence must land exactly on its mark.
        let out = xf.apply(d[0]);
        assert!(close(out.x, 20.0) && close(out.y, 30.0), "got {out:?}");
        let out = xf.apply(d[1]);
        assert!(close(out.x, 20.0) && close(out.y, 40.0), "got {out:?}");
    }

    #[test]
    fn three_points_recover_uniform_scale() {
        // A 2x enlarged, translated square. Scale must come back as 2.
        let d = [Point::new(0.0, 0.0), Point::new(10.0, 0.0), Point::new(0.0, 10.0)];
        let m = [Point::new(100.0, 100.0), Point::new(120.0, 100.0), Point::new(100.0, 120.0)];
        let xf = solve(&d, &m).unwrap();
        assert!(close(xf.scale(), 2.0), "expected scale 2, got {}", xf.scale());
        assert!(close(xf.rotation_rad(), 0.0), "no rotation expected");
    }

    #[test]
    fn four_points_average_a_small_jog_error() {
        // Three perfect corners and one 0.1 mm long. The fit should stay close to identity
        // and every mark should land within a fraction of a millimetre.
        let d = [Point::new(0.0, 0.0), Point::new(100.0, 0.0), Point::new(100.0, 100.0), Point::new(0.0, 100.0)];
        let m = [Point::new(0.0, 0.0), Point::new(100.0, 0.0), Point::new(100.0, 100.0), Point::new(0.0, 100.1)];
        let xf = solve(&d, &m).unwrap();
        assert!((xf.scale() - 1.0).abs() < 1e-3);
        for (p, q) in d.iter().zip(m.iter()) {
            let out = xf.apply(*p);
            let err = ((out.x - q.x).powi(2) + (out.y - q.y).powi(2)).sqrt();
            assert!(err < 0.1, "mark error {err:.4} mm at {p:?}");
        }
    }

    #[test]
    fn mismatched_counts_are_refused() {
        let d = [Point::new(0.0, 0.0), Point::new(1.0, 1.0)];
        let m = [Point::new(0.0, 0.0)];
        assert!(solve(&d, &m).is_err());
        assert!(solve(&[], &[]).is_err());
    }

    #[test]
    fn coincident_design_points_are_refused() {
        let d = [Point::new(5.0, 5.0), Point::new(5.0, 5.0)];
        let m = [Point::new(0.0, 0.0), Point::new(10.0, 0.0)];
        assert!(solve(&d, &m).is_err());
    }

    #[test]
    fn absurd_scale_is_refused() {
        // A mark jogged 10x away suggests a mis-jog, not a 10x stretched sheet.
        let d = [Point::new(0.0, 0.0), Point::new(10.0, 0.0), Point::new(0.0, 10.0)];
        let m = [Point::new(0.0, 0.0), Point::new(100.0, 0.0), Point::new(0.0, 100.0)];
        assert!(solve(&d, &m).is_err());
    }

    #[test]
    fn toolpaths_transform_in_place() {
        let mut tp = Toolpath::rectangle(0.0, 0.0, 10.0, 10.0);
        let xf = Similarity { a: 0.0, b: 1.0, tx: 0.0, ty: 0.0 };
        apply_to_toolpaths(std::slice::from_mut(&mut tp), &xf);
        // A 90 degree turn maps (10,0) onto (0,10).
        let seg = &tp.segments[0];
        assert!(close(seg.start().x, 0.0) && close(seg.start().y, 0.0));
        assert!(close(seg.end().x, 0.0) && close(seg.end().y, 10.0));
    }
}
