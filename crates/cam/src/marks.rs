//! Registration, alignment and weeding geometry.
//!
//! These are the CAM-side features that separate a plotter from a vinyl cutter. A cutter
//! must know where the design sits so a printed sheet can be lined up to the cut; it must
//! cut a weed box so the operator can peel the waste in one piece; and it must put lift
//! marks at holes so small counters do not tear when weeded.
//!
//! Two registration-mark styles cover print-then-cut work: corner L-marks and "ultra"
//! circle-cross marks. A 5 mm alignment mark is a 2x2 mm square plus 5 mm crosshair arms;
//! the "Ultra" variant uses concentric circles (9.5/9.0/5/4.5 mm) plus 10 mm arms. This
//! module builds that geometry as ordinary [`Toolpath`]s, so any protocol can cut it without
//! understanding a plotter macro language.
//!
//! Everything here is pure geometry: no machine state, no protocol knowledge. The caller
//! decides where the marks go and how they are emitted.

use crate::geom::{Point, Segment, Toolpath};
use std::f64::consts::PI;

/// Shape of one registration mark.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MarkStyle {
    /// An L with `size_mm` arms, opening toward the design. The common corner mark.
    CornerL { size_mm: f64 },
    /// A plus sign with `size_mm` arms. Reads from any direction.
    Cross { size_mm: f64 },
    /// A circle of `diameter_mm` with a crosshair through it. The "Ultra" registration mark.
    CircleCross {
        diameter_mm: f64,
        segments: usize,
    },
}

impl Default for MarkStyle {
    fn default() -> Self {
        Self::CornerL { size_mm: 5.0 }
    }
}

/// Where the marks are placed relative to the design's bounding box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MarkPlacement {
    /// One mark at each corner of the bounding box.
    Corners,
    /// One mark centred on each edge midpoint.
    EdgeMidpoints,
    /// A single mark at the centre of the design.
    Centre,
}

impl Default for MarkPlacement {
    fn default() -> Self {
        Self::Corners
    }
}

/// Configuration for registration marks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MarkConfig {
    pub style: MarkStyle,
    pub placement: MarkPlacement,
    /// Distance from the design's bounding box to the mark's anchor point, in mm. The mark
    /// must sit outside the design so it never overlaps the artwork.
    pub margin_mm: f64,
}

impl Default for MarkConfig {
    fn default() -> Self {
        Self {
            style: MarkStyle::default(),
            placement: MarkPlacement::default(),
            margin_mm: 5.0,
        }
    }
}

/// Generate registration marks around `bbox` (min, max) in millimetres.
///
/// The marks sit outside the box by `cfg.margin_mm` and open toward the design's centre, so
/// a corner L wraps its corner instead of pointing away from it.
pub fn registration_marks(bbox: (Point, Point), cfg: &MarkConfig) -> Vec<Toolpath> {
    let (min, max) = bbox;
    let cx = (min.x + max.x) * 0.5;
    let cy = (min.y + max.y) * 0.5;
    let centre = Point::new(cx, cy);
    let m = cfg.margin_mm;
    let anchors: Vec<Point> = match cfg.placement {
        MarkPlacement::Corners => vec![
            Point::new(min.x - m, min.y - m),
            Point::new(max.x + m, min.y - m),
            Point::new(max.x + m, max.y + m),
            Point::new(min.x - m, max.y + m),
        ],
        MarkPlacement::EdgeMidpoints => vec![
            Point::new(cx, min.y - m),
            Point::new(max.x + m, cy),
            Point::new(cx, max.y + m),
            Point::new(min.x - m, cy),
        ],
        MarkPlacement::Centre => vec![centre],
    };
    anchors
        .into_iter()
        .flat_map(|p| mark_at(p, centre, cfg.style))
        .collect()
}

/// One mark's geometry at `p`, opening toward `toward`.
fn mark_at(p: Point, toward: Point, style: MarkStyle) -> Vec<Toolpath> {
    match style {
        MarkStyle::CornerL { size_mm } => {
            let s = size_mm.max(0.1);
            // Arms point from the anchor toward the design so the L wraps the corner.
            let dx = if toward.x >= p.x { 1.0 } else { -1.0 };
            let dy = if toward.y >= p.y { 1.0 } else { -1.0 };
            vec![
                line(p, Point::new(p.x + dx * s, p.y)),
                line(p, Point::new(p.x, p.y + dy * s)),
            ]
        }
        MarkStyle::Cross { size_mm } => {
            let h = size_mm.max(0.1) * 0.5;
            vec![
                line(Point::new(p.x - h, p.y), Point::new(p.x + h, p.y)),
                line(Point::new(p.x, p.y - h), Point::new(p.x, p.y + h)),
            ]
        }
        MarkStyle::CircleCross {
            diameter_mm,
            segments,
        } => {
            let r = diameter_mm.max(0.1) * 0.5;
            let n = segments.max(8);
            let mut segs = Vec::with_capacity(n);
            for i in 0..n {
                let a0 = 2.0 * PI * i as f64 / n as f64;
                let a1 = 2.0 * PI * (i + 1) as f64 / n as f64;
                segs.push(Segment::Line {
                    a: Point::new(p.x + r * a0.cos(), p.y + r * a0.sin()),
                    b: Point::new(p.x + r * a1.cos(), p.y + r * a1.sin()),
                });
            }
            let mut out = vec![Toolpath {
                segments: segs,
                closed: true,
            }];
            // Crosshair through the circle, matching the "Ultra" registration mark.
            out.push(line(
                Point::new(p.x - r, p.y),
                Point::new(p.x + r, p.y),
            ));
            out.push(line(
                Point::new(p.x, p.y - r),
                Point::new(p.x, p.y + r),
            ));
            out
        }
    }
}

/// A rectangle around the design, `margin_mm` clear on every side. Cutting this first lets
/// the operator peel the waste vinyl in one piece instead of picking at it.
pub fn weed_box(bbox: (Point, Point), margin_mm: f64) -> Toolpath {
    let (min, max) = bbox;
    let m = margin_mm.max(0.0);
    Toolpath::rectangle(
        min.x - m,
        min.y - m,
        (max.x - min.x) + 2.0 * m,
        (max.y - min.y) + 2.0 * m,
    )
}

/// A short "grab tab" at each closed contour's start point. Weeding a hole by hand starts at
/// the tab instead of poking at the vinyl edge and tearing the design. The tab runs at 135°
/// so it clears the contour instead of cutting back through it.
pub fn lift_marks(toolpaths: &[Toolpath], size_mm: f64) -> Vec<Toolpath> {
    let s = size_mm.max(0.1);
    toolpaths
        .iter()
        .filter(|tp| tp.closed)
        .filter_map(|tp| tp.segments.first())
        .map(|seg| {
            let p = seg.start();
            let a = 3.0 * PI / 4.0;
            line(p, Point::new(p.x + s * a.cos(), p.y + s * a.sin()))
        })
        .collect()
}

fn line(a: Point, b: Point) -> Toolpath {
    Toolpath {
        segments: vec![Segment::Line { a: a, b: b }],
        closed: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bbox() -> (Point, Point) {
        (Point::new(0.0, 0.0), Point::new(100.0, 50.0))
    }

    #[test]
    fn corner_l_marks_sit_outside_the_bbox_and_open_inward() {
        let cfg = MarkConfig {
            style: MarkStyle::CornerL { size_mm: 5.0 },
            placement: MarkPlacement::Corners,
            margin_mm: 5.0,
        };
        let marks = registration_marks(bbox(), &cfg);
        // Four corners, two arms each.
        assert_eq!(marks.len(), 8);

        // Bottom-left anchor is (-5, -5); arms must run toward +x and +y.
        let bl: Vec<&Toolpath> = marks.iter().take(2).collect();
        for tp in bl {
            let s = tp.segments[0].start();
            let e = tp.segments[0].end();
            assert_eq!(s, Point::new(-5.0, -5.0));
            assert!(e.x > s.x || e.y > s.y, "arm should open toward the design");
        }

        // Every arm endpoint stays within the mark's reach of the box.
        for tp in &marks {
            for seg in &tp.segments {
                for p in [seg.start(), seg.end()] {
                    assert!(p.x >= -11.0 && p.x <= 106.0);
                    assert!(p.y >= -11.0 && p.y <= 56.0);
                }
            }
        }
    }

    #[test]
    fn cross_marks_are_centred_on_their_anchor() {
        let cfg = MarkConfig {
            style: MarkStyle::Cross { size_mm: 10.0 },
            placement: MarkPlacement::Centre,
            margin_mm: 0.0,
        };
        let marks = registration_marks(bbox(), &cfg);
        assert_eq!(marks.len(), 2);
        let centre = Point::new(50.0, 25.0);
        for tp in &marks {
            let seg = &tp.segments[0];
            assert!((seg.start().x + seg.end().x) * 0.5 == centre.x);
            assert!((seg.start().y + seg.end().y) * 0.5 == centre.y);
        }
    }

    #[test]
    fn circle_cross_is_closed_and_has_a_crosshair() {
        let cfg = MarkConfig {
            style: MarkStyle::CircleCross {
                diameter_mm: 9.0,
                segments: 32,
            },
            placement: MarkPlacement::Centre,
            margin_mm: 0.0,
        };
        let marks = registration_marks(bbox(), &cfg);
        // One closed circle plus two crosshair arms.
        assert_eq!(marks.len(), 3);
        assert!(marks[0].closed);
        assert_eq!(marks[0].segments.len(), 32);
        // The circle's diameter should be within a segment's sagitta of 9 mm.
        let xs: Vec<f64> = marks[0]
            .segments
            .iter()
            .flat_map(|s| [s.start().x, s.end().x])
            .collect();
        let min = xs.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!((max - min - 9.0).abs() < 0.1, "diameter was {}", max - min);
    }

    #[test]
    fn weed_box_encloses_the_design_with_margin() {
        let wb = weed_box(bbox(), 3.0);
        assert!(wb.closed);
        assert_eq!(wb.segments.len(), 4);
        let xs: Vec<f64> = wb.segments.iter().flat_map(|s| [s.start().x, s.end().x]).collect();
        let ys: Vec<f64> = wb.segments.iter().flat_map(|s| [s.start().y, s.end().y]).collect();
        assert_eq!(xs.iter().cloned().fold(f64::INFINITY, f64::min), -3.0);
        assert_eq!(xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max), 103.0);
        assert_eq!(ys.iter().cloned().fold(f64::INFINITY, f64::min), -3.0);
        assert_eq!(ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max), 53.0);
    }

    #[test]
    fn lift_marks_only_attach_to_closed_contours() {
        let closed = Toolpath::rectangle(0.0, 0.0, 10.0, 10.0);
        let open = Toolpath {
            segments: vec![Segment::Line {
                a: Point::new(0.0, 0.0),
                b: Point::new(10.0, 0.0),
            }],
            closed: false,
        };
        let marks = lift_marks(&[closed, open], 2.0);
        assert_eq!(marks.len(), 1, "an open path has no hole to weed");
        let s = marks[0].segments[0].start();
        assert_eq!(s, Point::new(0.0, 0.0));
    }
}
