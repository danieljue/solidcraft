//! What a machine does to the material, as opposed to how it talks to a controller.
//!
//! A [`Protocol`](crate::profile::Protocol) says how to send commands and a
//! [`MachineProfile`](crate::profile::MachineProfile) says how that controller is wired. A
//! process says what the cut physically *is*, which decides how far the toolpath moves off the
//! drawn line before any protocol sees it.
//!
//! The point of the split is reuse: a drag knife, a plasma torch, a laser, an abrasive waterjet
//! and a router all cut a closed contour, and the compensation they need is the same
//! calculation — move the path by half the cut width, outward for a part and inward for a hole.
//! Only the width differs. Keeping that calculation here, in one place, is what lets a new
//! machine type be a new number rather than a new offsetter.
//!
//! A knife is the exception that proves the rule: its cut width is a point, so it needs no
//! kerf compensation at all. What it needs instead — the blade trailing the pivot — depends on
//! the blade, not the material, and stays in the profile as `blade_offset_mm`.

use serde::{Deserialize, Serialize};

use crate::boolean::signed_area;
use crate::geom::{Point, Toolpath};
use crate::offset::offset_closed_with_tolerance;

/// The manufacturing process a machine performs.
///
/// Serialised externally tagged, so a profile carries `"process": "knife"` or
/// `"process": {"plasma": {"kerf_mm": 1.5}}`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum Process {
    /// A drag knife. The cut is a point; `blade_offset_mm` and `overcut_mm` in the profile do
    /// the compensating, because they describe the blade rather than the material.
    #[default]
    Knife,
    /// A pen, marker or scoring tool. Nothing to compensate.
    Pen,
    /// A plasma torch. `kerf_mm` is the full width of the cut.
    Plasma { kerf_mm: f64 },
    /// A laser. `kerf_mm` is the full width of the cut.
    Laser { kerf_mm: f64 },
    /// An abrasive waterjet. `kerf_mm` is the width at the exit face; the taper through the
    /// depth is not modelled, so a thick part still needs its own test cut.
    Waterjet { kerf_mm: f64 },
    /// A milling cutter or router. `tool_diameter_mm` is the cutter's diameter, which is the
    /// cut width.
    Router { tool_diameter_mm: f64 },
}

impl Process {
    /// Full width of the cut, in millimetres. Zero when the tool removes no material.
    pub fn kerf_mm(self) -> f64 {
        match self {
            Process::Knife | Process::Pen => 0.0,
            Process::Plasma { kerf_mm } | Process::Laser { kerf_mm } | Process::Waterjet { kerf_mm } => kerf_mm.max(0.0),
            Process::Router { tool_diameter_mm } => tool_diameter_mm.max(0.0),
        }
    }

    /// Half the cut width: how far the tool centre sits from the drawn line. The tool is
    /// centred on its path, so the edge of the cut is half a kerf either side of it.
    pub fn half_kerf_mm(self) -> f64 {
        self.kerf_mm() * 0.5
    }

    /// True for processes that pierce the material to start a cut. A thermal cut benefits from
    /// a lead-in so the pierce crater lands off the part; nothing here assumes one, because the
    /// length depends on the material and the geometry.
    pub fn is_thermal(self) -> bool {
        matches!(self, Process::Plasma { .. } | Process::Laser { .. })
    }

    /// Move every closed contour off the drawn line by half the cut width.
    ///
    /// A counter-clockwise contour is treated as a part outline and grows outward; a clockwise
    /// one is treated as a hole and shrinks inward, so the finished part keeps the size that was
    /// drawn and each hole comes out the size that was drawn. That is the same convention
    /// [`Region2`](solidcraft_geom::Region2) documents, and the same rule
    /// [`region_to_toolpaths`](crate::cut_job::region_to_toolpaths) applies to a section.
    ///
    /// Open contours are engraving lines: there is no inside or outside, so they pass through
    /// untouched. A contour the offsetter cannot handle is returned as it was rather than
    /// dropped — losing a cut is worse than cutting it uncompensated.
    pub fn compensate(self, paths: &[Toolpath], curve_tol_mm: f64) -> Vec<Toolpath> {
        let half = self.half_kerf_mm();
        if half <= 0.0 {
            return paths.to_vec();
        }
        let mut out = Vec::with_capacity(paths.len());
        for path in paths {
            if !path.closed {
                out.push(path.clone());
                continue;
            }
            let distance = if signed_area(path) >= 0.0 { half } else { -half };
            let mut shifted = offset_closed_with_tolerance(path, distance, curve_tol_mm);
            if shifted.is_empty() {
                // The offsetter refuses degenerate or self-intersecting input; keep the
                // contour so the job still cuts the shape the operator drew.
                out.push(path.clone());
            } else {
                out.append(&mut shifted);
            }
        }
        out
    }
}

/// Bounding box of a toolpath, for tests and callers that need the extent of a result.
#[doc(hidden)]
pub fn bounds(paths: &[Toolpath]) -> Option<(Point, Point)> {
    let mut min: Option<Point> = None;
    let mut max: Option<Point> = None;
    for path in paths {
        for seg in &path.segments {
            for p in [seg.start(), seg.end()] {
                min = Some(Point::new(min.map_or(p.x, |m: Point| m.x.min(p.x)), min.map_or(p.y, |m: Point| m.y.min(p.y))));
                max = Some(Point::new(max.map_or(p.x, |m: Point| m.x.max(p.x)), max.map_or(p.y, |m: Point| m.y.max(p.y))));
            }
        }
    }
    match (min, max) {
        (Some(lo), Some(hi)) => Some((lo, hi)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square_ccw(size: f64) -> Toolpath {
        Toolpath::rectangle(0.0, 0.0, size, size)
    }

    fn square_cw(size: f64) -> Toolpath {
        let mut tp = square_ccw(size);
        tp.segments.reverse();
        for s in &mut tp.segments {
            *s = s.reversed();
        }
        tp
    }

    fn open_line() -> Toolpath {
        Toolpath::from_polyline(&[Point::new(0.0, 0.0), Point::new(10.0, 0.0)], false)
    }

    fn extents(paths: &[Toolpath]) -> (f64, f64, f64, f64) {
        let (lo, hi) = bounds(paths).expect("geometry");
        (lo.x, lo.y, hi.x, hi.y)
    }

    #[test]
    fn a_knife_and_a_pen_compensate_nothing() {
        let paths = vec![square_ccw(10.0)];
        for process in [Process::Knife, Process::Pen] {
            assert_eq!(process.kerf_mm(), 0.0);
            let out = process.compensate(&paths, 0.05);
            assert_eq!(extents(&out), (0.0, 0.0, 10.0, 10.0));
        }
    }

    #[test]
    fn plasma_grows_an_outer_contour_by_half_the_kerf() {
        // A 2 mm kerf puts the tool centre 1 mm outside the drawn line on every side.
        let out = Process::Plasma { kerf_mm: 2.0 }.compensate(&[square_ccw(10.0)], 0.05);
        let (x0, y0, x1, y1) = extents(&out);
        assert!((x0 + 1.0).abs() < 1e-6, "left edge at {x0}, expected -1");
        assert!((y0 + 1.0).abs() < 1e-6, "bottom edge at {y0}, expected -1");
        assert!((x1 - 11.0).abs() < 1e-6, "right edge at {x1}, expected 11");
        assert!((y1 - 11.0).abs() < 1e-6, "top edge at {y1}, expected 11");
    }

    #[test]
    fn plasma_shrinks_a_hole_by_half_the_kerf() {
        // A clockwise contour is a hole: the tool runs inside the void, so the path shrinks.
        let out = Process::Plasma { kerf_mm: 2.0 }.compensate(&[square_cw(10.0)], 0.05);
        let (x0, y0, x1, y1) = extents(&out);
        assert!((x0 - 1.0).abs() < 1e-6, "left edge at {x0}, expected 1");
        assert!((y0 - 1.0).abs() < 1e-6, "bottom edge at {y0}, expected 1");
        assert!((x1 - 9.0).abs() < 1e-6, "right edge at {x1}, expected 9");
        assert!((y1 - 9.0).abs() < 1e-6, "top edge at {y1}, expected 9");
    }

    #[test]
    fn a_router_uses_its_tool_diameter_as_the_kerf() {
        // A 1/8 in cutter is 3.175 mm across, so the centre runs 1.5875 mm off the line.
        let process = Process::Router { tool_diameter_mm: 3.175 };
        assert!((process.kerf_mm() - 3.175).abs() < 1e-12);
        let (x0, _, x1, _) = extents(&process.compensate(&[square_ccw(10.0)], 0.05));
        assert!((x0 + 1.5875).abs() < 1e-6, "left edge at {x0}");
        assert!((x1 - 11.5875).abs() < 1e-6, "right edge at {x1}");
    }

    #[test]
    fn open_contours_are_engraving_and_pass_through() {
        let out = Process::Plasma { kerf_mm: 2.0 }.compensate(&[open_line()], 0.05);
        assert_eq!(out.len(), 1);
        assert!(!out[0].closed);
        assert_eq!(extents(&out), (0.0, 0.0, 10.0, 0.0));
    }

    #[test]
    fn a_zero_kerf_process_is_the_identity() {
        let paths = vec![square_ccw(10.0), square_cw(4.0)];
        let out = Process::Plasma { kerf_mm: 0.0 }.compensate(&paths, 0.05);
        assert_eq!(out.len(), paths.len());
        assert_eq!(extents(&out[..1]), (0.0, 0.0, 10.0, 10.0));
    }

    #[test]
    fn thermal_processes_are_flagged_and_cold_ones_are_not() {
        assert!(Process::Plasma { kerf_mm: 1.5 }.is_thermal());
        assert!(Process::Laser { kerf_mm: 0.2 }.is_thermal());
        assert!(!Process::Waterjet { kerf_mm: 1.0 }.is_thermal());
        assert!(!Process::Router { tool_diameter_mm: 3.0 }.is_thermal());
        assert!(!Process::Knife.is_thermal());
    }

    #[test]
    fn the_process_tolerance_decides_how_finely_a_curve_is_chorded() {
        // A torch does not need a knife's chord density. The tolerance travels from the profile
        // through the process into the offsetter, so a plasma profile's coarse setting really
        // does produce a shorter command stream rather than being ignored.
        let circle = Toolpath::circle(Point::ZERO, 20.0);
        let process = Process::Plasma { kerf_mm: 2.0 };
        let fine = process.compensate(&[circle.clone()], 0.01);
        let coarse = process.compensate(&[circle], 2.0);
        let fine_segments: usize = fine.iter().map(|p| p.segments.len()).sum();
        let coarse_segments: usize = coarse.iter().map(|p| p.segments.len()).sum();
        assert!(fine_segments > 4 * coarse_segments, "fine {fine_segments} vs coarse {coarse_segments} segments");
        // Both still describe the same ring: a 20 mm radius grown by half the 2 mm kerf, so a
        // 21 mm radius and a 42 mm diameter, whatever the chord count.
        for paths in [&fine, &coarse] {
            let (lo, hi) = bounds(paths).expect("geometry");
            assert!((hi.x - lo.x - 42.0).abs() < 0.5, "diameter {:.2}, expected 42", hi.x - lo.x);
        }
    }

    #[test]
    fn a_negative_kerf_is_treated_as_no_kerf_rather_than_inverting_the_cut() {
        let out = Process::Plasma { kerf_mm: -2.0 }.compensate(&[square_ccw(10.0)], 0.05);
        assert_eq!(extents(&out), (0.0, 0.0, 10.0, 10.0));
    }
}
