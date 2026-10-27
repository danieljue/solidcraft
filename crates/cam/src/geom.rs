//! 2D geometry for CAM, built on the workspace curve types.
//!
//! `solidcraft_geom::Seg2` already models straight lines, circular arcs, cubic and
//! higher-degree Béziers and rational conics, and carries the traversal maths a
//! post-processor needs: length, splitting, tangents and tolerance-driven flattening.
//! Re-exporting it rather than defining a second, line-only segment type is what lets one
//! toolpath drive a vinyl knife emitting HPGL arcs, a plasma torch emitting `G2`/`G3`, and a
//! controller that only understands straight moves — without any of them owning the
//! geometry code or drifting apart from the rest of the workspace.

use serde::{Deserialize, Serialize};

pub use solidcraft_geom::Seg2 as Segment;
pub use solidcraft_geom::Vec2 as Point;

/// Chord tolerance used when a curve has to become straight edges for an algorithm that only
/// reasons about polygons — offsetting, boolean operations, tiling. Fine enough that the
/// approximation is invisible on a cut, coarse enough to keep the vertex count sane.
pub const DEFAULT_FLATTEN_TOL_MM: f64 = 0.05;

/// A contour: its segments in order, and whether the last one closes back onto the first.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Toolpath {
    pub segments: Vec<Segment>,
    pub closed: bool,
}

impl Toolpath {
    /// Shift every segment by `(dx, dy)` millimetres.
    ///
    /// Handled here for every segment kind, so a new curve type costs one arm in one place
    /// instead of a change at every call site.
    pub fn translate(&mut self, dx: f64, dy: f64) {
        for seg in &mut self.segments {
            translate_segment(seg, dx, dy);
        }
    }

    /// A rectangle from `(x, y)` with width `w` and height `h`, closed.
    pub fn rectangle(x: f64, y: f64, w: f64, h: f64) -> Self {
        let a = Point::new(x, y);
        let b = Point::new(x + w, y);
        let c = Point::new(x + w, y + h);
        let d = Point::new(x, y + h);
        Self {
            segments: vec![Segment::Line { a, b }, Segment::Line { a: b, b: c }, Segment::Line { a: c, b: d }, Segment::Line { a: d, b: a }],
            closed: true,
        }
    }

    /// A circle as two counter-clockwise half arcs, so a post-processor that understands
    /// arcs can emit two commands instead of tessellating the whole curve.
    pub fn circle(center: Point, radius: f64) -> Self {
        Self { segments: solidcraft_geom::Loop2::circle(center, radius).segs, closed: true }
    }

    /// A toolpath that follows `points` in order, one straight segment per pair.
    pub fn from_polyline(points: &[Point], closed: bool) -> Self {
        let mut segments: Vec<Segment> = points.windows(2).map(|w| Segment::Line { a: w[0], b: w[1] }).collect();
        if closed {
            if let (Some(first), Some(last)) = (points.first(), points.last()) {
                if first != last {
                    segments.push(Segment::Line { a: *last, b: *first });
                }
            }
        }
        Self { segments, closed }
    }

    /// The contour as a polyline, flattening curves to within `tol` millimetres.
    ///
    /// A closed contour ends on its first point, matching the convention the design model
    /// and the nesting code already use.
    pub fn polyline(&self, tol: f64) -> Vec<Point> {
        let mut out: Vec<Point> = Vec::new();
        for seg in &self.segments {
            for p in seg.polyline(tol) {
                if out.last().map(|l| *l == p).unwrap_or(false) {
                    continue;
                }
                out.push(p);
            }
        }
        if self.closed {
            if let (Some(first), Some(last)) = (out.first().copied(), out.last().copied()) {
                if first != last {
                    out.push(first);
                }
            }
        }
        out
    }

    /// Total path length in millimetres, exact for lines and arcs.
    pub fn length_mm(&self) -> f64 {
        self.segments.iter().map(|s| s.length()).sum()
    }

    /// This contour with every curve replaced by chords.
    ///
    /// The polygon algorithms in this crate — offsetting, boolean operations, tiling — reason
    /// about vertices and straight edges. Handing them an arc would silently take its chord,
    /// which is why they call this first. Lines are passed through untouched, so a design that
    /// is already polygonal costs one vector and no accuracy.
    pub fn flattened(&self, tol: f64) -> Toolpath {
        let mut segments = Vec::with_capacity(self.segments.len());
        for seg in &self.segments {
            if matches!(seg, Segment::Line { .. }) {
                segments.push(*seg);
                continue;
            }
            let pts = seg.polyline(tol);
            for w in pts.windows(2) {
                if w[0] != w[1] {
                    segments.push(Segment::Line { a: w[0], b: w[1] });
                }
            }
        }
        Toolpath { segments, closed: self.closed }
    }

    /// True when every segment is straight, i.e. nothing is lost by emitting moves only.
    pub fn is_all_lines(&self) -> bool {
        self.segments.iter().all(|s| matches!(s, Segment::Line { .. }))
    }
}

/// Shift one segment by `(dx, dy)`.
fn translate_segment(seg: &mut Segment, dx: f64, dy: f64) {
    match seg {
        Segment::Line { a, b } => {
            shift(a, dx, dy);
            shift(b, dx, dy);
        }
        Segment::Arc { center, .. } => shift(center, dx, dy),
        Segment::Cubic { p0, p1, p2, p3 } => {
            shift(p0, dx, dy);
            shift(p1, dx, dy);
            shift(p2, dx, dy);
            shift(p3, dx, dy);
        }
        Segment::Conic { a, apex, b, .. } => {
            shift(a, dx, dy);
            shift(apex, dx, dy);
            shift(b, dx, dy);
        }
        Segment::Bezier { n, p } => {
            let count = (*n as usize).min(p.len().saturating_sub(1)) + 1;
            for c in p.iter_mut().take(count) {
                shift(c, dx, dy);
            }
        }
    }
}

fn shift(p: &mut Point, dx: f64, dy: f64) {
    p.x += dx;
    p.y += dy;
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn rectangle_has_four_lines() {
        let tp = Toolpath::rectangle(0.0, 0.0, 10.0, 10.0);
        assert_eq!(tp.segments.len(), 4);
        assert!(tp.is_all_lines());
        assert!((tp.length_mm() - 40.0).abs() < 1e-9);
    }

    #[test]
    fn circle_is_two_arcs_and_keeps_its_length_exact() {
        let tp = Toolpath::circle(Point::new(5.0, 5.0), 5.0);
        assert_eq!(tp.segments.len(), 2);
        assert!(!tp.is_all_lines());
        assert!((tp.length_mm() - 2.0 * PI * 5.0).abs() < 1e-9);
    }

    #[test]
    fn translate_moves_arcs_and_their_centre() {
        let mut tp = Toolpath::circle(Point::ZERO, 5.0);
        tp.translate(10.0, -3.0);
        if let Segment::Arc { center, .. } = tp.segments[0] {
            assert_eq!(center, Point::new(10.0, -3.0));
        } else {
            panic!("expected an arc");
        }
    }

    #[test]
    fn polyline_flattens_a_circle_to_its_tolerance() {
        let tp = Toolpath::circle(Point::ZERO, 50.0);
        let poly = tp.polyline(0.05);
        assert!(poly.len() > 16, "expected a tessellated circle, got {}", poly.len());
        assert_eq!(poly.first(), poly.last());
        for p in &poly {
            assert!((p.len() - 50.0).abs() < 1e-6);
        }
    }

    #[test]
    fn from_polyline_closes_without_duplicating_the_first_point() {
        let pts = vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0), Point::new(10.0, 10.0)];
        let tp = Toolpath::from_polyline(&pts, true);
        assert_eq!(tp.segments.len(), 3);
        assert!(tp.closed);
    }
}
