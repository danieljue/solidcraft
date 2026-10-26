//! 2D drawing exchange for sketches: DXF (ASCII, read and write) and SVG (read). Readers produce
//! plain 2D geometry in millimetres; the engine turns it into sketch curves.

use solidcraft_geom::{Loop2, Seg2, Vec2};

/// One imported 2D entity (millimetres, y up).
#[derive(Clone, Debug, PartialEq)]
pub enum Geom2 {
    Point(Vec2),
    Line(Vec2, Vec2),
    Circle(Vec2, f64),
    /// Counter-clockwise arc around `c` from `a` to `b`.
    Arc {
        c: Vec2,
        a: Vec2,
        b: Vec2,
    },
    /// Full ellipse: centre, end of the major axis, minor radius.
    Ellipse {
        c: Vec2,
        major: Vec2,
        minor: f64,
    },
    /// Spline through (`control == false`) or over (`control`) the points.
    Spline {
        pts: Vec<Vec2>,
        control: bool,
        degree: u8,
    },
    /// Quadratic Bézier / conic.
    Conic {
        a: Vec2,
        apex: Vec2,
        b: Vec2,
        rho: f64,
    },
    Text {
        text: String,
        at: Vec2,
        height: f64,
        angle: f64,
    },
}

/// Most entities one file may produce.
pub const MAX_ENTITIES: usize = 200_000;
/// Largest drawing file we read.
pub const MAX_DRAWING_BYTES: usize = 64 << 20;

/// Counter-clockwise arc from `p0` to `p1` with DXF bulge `b` (tan of a quarter of the
/// included angle, negative = clockwise).
pub(crate) fn bulge_arc(p0: Vec2, p1: Vec2, b: f64) -> Option<Geom2> {
    if b.abs() < 1e-12 || !b.is_finite() {
        return Some(Geom2::Line(p0, p1));
    }
    let chord = p1 - p0;
    let l = chord.len();
    if l < 1e-12 {
        return None;
    }
    let theta = 4.0 * b.atan();
    let r = l / (2.0 * (theta / 2.0).sin()).abs();
    // Centre on the perpendicular bisector, to the left for counter-clockwise bulges.
    let mid = (p0 + p1) * 0.5;
    let h = r * (theta / 2.0).cos().abs();
    let left = chord.perp() / l;
    let sign = if theta.abs() > std::f64::consts::PI { -1.0 } else { 1.0 };
    let c = if b > 0.0 { mid + left * (h * sign) } else { mid - left * (h * sign) };
    Some(if b > 0.0 { Geom2::Arc { c, a: p0, b: p1 } } else { Geom2::Arc { c, a: p1, b: p0 } })
}


/// Control-point factor that approximates a quarter circle with one cubic Bézier to
/// within about 0.02% of the radius.
const KAPPA: f64 = 0.552_284_749_830_793_4;

impl Geom2 {
    /// This entity as exact curve segments, for a consumer that can emit arcs and Béziers
    /// instead of flattening them first — a CAM post-processor, a viewer, an offsetter.
    ///
    /// Lines, circles, arcs and conics convert exactly. An ellipse becomes four cubic
    /// Béziers, the standard approximation. Splines and text return nothing: evaluating
    /// them needs the spline and text-layout machinery that lives above this interchange
    /// layer, so a caller that gets an empty result should fall back to flattening the
    /// entity itself rather than dropping it.
    pub fn to_segments(&self) -> Vec<Seg2> {
        use std::f64::consts::TAU;
        match self {
            Geom2::Line(a, b) => vec![Seg2::Line { a: *a, b: *b }],
            Geom2::Circle(c, r) => {
                if *r <= 0.0 || !r.is_finite() {
                    return Vec::new();
                }
                Loop2::circle(*c, *r).segs
            }
            Geom2::Arc { c, a, b } => {
                let radius = a.dist(*c);
                if radius < 1e-12 {
                    return Vec::new();
                }
                // A closed arc is a circle; two half arcs express it exactly.
                if a.dist(*b) < 1e-12 {
                    return Loop2::circle(*c, radius).segs;
                }
                let start = (a.y - c.y).atan2(a.x - c.x);
                let end = (b.y - c.y).atan2(b.x - c.x);
                let mut sweep = end - start;
                while sweep <= 0.0 {
                    sweep += TAU;
                }
                vec![Seg2::Arc { center: *c, radius, start, sweep }]
            }
            Geom2::Ellipse { c, major, minor } => {
                let ex = *major - *c;
                let ax = ex.len();
                let ry = *minor;
                if ax < 1e-12 || ry.abs() < 1e-12 {
                    return Vec::new();
                }
                let theta = ex.y.atan2(ex.x);
                let (ct, st) = (theta.cos(), theta.sin());
                let rot = |p: Vec2| Vec2::new(p.x * ct - p.y * st, p.x * st + p.y * ct);
                let at = |x: f64, y: f64| *c + rot(Vec2::new(x, y));
                let (kx, ky) = (KAPPA * ax, KAPPA * ry);
                // Four quarter arcs, counter-clockwise from the +major end.
                let quads = [
                    (ax, 0.0, ax, ky, kx, ry, 0.0, ry),
                    (0.0, ry, -kx, ry, -ax, ky, -ax, 0.0),
                    (-ax, 0.0, -ax, -ky, -kx, -ry, 0.0, -ry),
                    (0.0, -ry, kx, -ry, ax, -ky, ax, 0.0),
                ];
                quads
                    .iter()
                    .map(|&(x0, y0, x1, y1, x2, y2, x3, y3)| Seg2::Cubic {
                        p0: at(x0, y0),
                        p1: at(x1, y1),
                        p2: at(x2, y2),
                        p3: at(x3, y3),
                    })
                    .collect()
            }
            Geom2::Conic { a, apex, b, rho } => vec![Seg2::Conic {
                a: *a,
                apex: *apex,
                b: *b,
                w: *rho,
            }],
            Geom2::Point(_) | Geom2::Spline { .. } | Geom2::Text { .. } => Vec::new(),
        }
    }
}

#[cfg(test)]
mod to_segments_tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn a_circle_is_two_exact_arcs() {
        let s = Geom2::Circle(Vec2::new(3.0, 4.0), 10.0).to_segments();
        assert_eq!(s.len(), 2);
        let len: f64 = s.iter().map(|x| x.length()).sum();
        assert!((len - 2.0 * PI * 10.0).abs() < 1e-9, "circumference was {len}");
    }

    #[test]
    fn a_quarter_arc_keeps_its_sweep_and_endpoints() {
        let g = Geom2::Arc {
            c: Vec2::ZERO,
            a: Vec2::new(10.0, 0.0),
            b: Vec2::new(0.0, 10.0),
        };
        let s = g.to_segments();
        assert_eq!(s.len(), 1);
        match s[0] {
            Seg2::Arc { sweep, radius, start, .. } => {
                assert!((sweep - PI / 2.0).abs() < 1e-12);
                assert!((radius - 10.0).abs() < 1e-12);
                assert!(start.abs() < 1e-12);
            }
            _ => panic!("expected an arc"),
        }
        assert_eq!(s[0].start(), Vec2::new(10.0, 0.0));
        assert!(s[0].end().dist(Vec2::new(0.0, 10.0)) < 1e-9);
    }

    #[test]
    fn an_arc_whose_endpoints_coincide_is_a_full_circle() {
        let g = Geom2::Arc { c: Vec2::ZERO, a: Vec2::new(5.0, 0.0), b: Vec2::new(5.0, 0.0) };
        let s = g.to_segments();
        assert_eq!(s.len(), 2, "a full turn must not collapse to a zero sweep");
        let len: f64 = s.iter().map(|x| x.length()).sum();
        assert!((len - 2.0 * PI * 5.0).abs() < 1e-9);
    }

    #[test]
    fn lines_and_conics_convert_exactly() {
        let l = Geom2::Line(Vec2::ZERO, Vec2::new(5.0, 0.0)).to_segments();
        assert!(matches!(l[0], Seg2::Line { .. }));
        assert!((l[0].length() - 5.0).abs() < 1e-12);

        let c = Geom2::Conic {
            a: Vec2::ZERO,
            apex: Vec2::new(1.0, 1.0),
            b: Vec2::new(2.0, 0.0),
            rho: 0.7,
        };
        assert!(matches!(c.to_segments()[0], Seg2::Conic { .. }));
    }

    #[test]
    fn an_ellipse_becomes_four_cubics_that_close_and_stay_on_the_curve() {
        let g = Geom2::Ellipse {
            c: Vec2::new(10.0, 10.0),
            major: Vec2::new(30.0, 10.0),
            minor: 6.0,
        };
        let s = g.to_segments();
        assert_eq!(s.len(), 4);
        for i in 0..4 {
            assert_eq!(s[i].end(), s[(i + 1) % 4].start(), "gap at piece {i}");
        }
        for seg in &s {
            let p = seg.end();
            let v = ((p.x - 10.0) / 20.0).powi(2) + ((p.y - 10.0) / 6.0).powi(2);
            assert!((v - 1.0).abs() < 1e-9, "point {p:?} is not on the ellipse");
        }
    }

    #[test]
    fn text_returns_nothing_so_the_caller_can_flatten_it_itself() {
        let t = Geom2::Text {
            text: "A".into(),
            at: Vec2::ZERO,
            height: 5.0,
            angle: 0.0,
        };
        assert!(t.to_segments().is_empty());
    }
}
