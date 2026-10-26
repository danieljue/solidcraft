//! G-code post-processor. `G0` is a rapid (tool clear), `G1` is a feed move (cutting):
//! the "implicit" tool-lift mode that Inkcut's `gcode.py` defaults to. The profile's
//! header carries the unit/mode setup (`G21;G90;`) and the footer carries the park or
//! return sequence. Coordinates are millimetres, so `units_per_mm` is 1.0.
//!
//! Arcs are emitted as `G2`/`G3` with `I`/`J` centre offsets, which is what a plasma, laser
//! or router controller expects; the same toolpath therefore cuts either as HPGL for a
//! vinyl knife or as G-code for a torch without touching the geometry.

use crate::geom::{Point, Toolpath};
use crate::post::{CommandSet, Pen};
use crate::profile::MachineProfile;

pub fn emit(profile: &MachineProfile, toolpaths: &[Toolpath]) -> String {
    emit_with_pen(profile, toolpaths, Pen::Down)
}

pub fn emit_with_pen(profile: &MachineProfile, toolpaths: &[Toolpath], pen: Pen) -> String {
    super::emit_generic(profile, toolpaths, pen, &Gcode { u: profile.units_per_mm })
}

struct Gcode {
    u: f64,
}

impl CommandSet for Gcode {
    fn move_to(&self, p: Point) -> String {
        format!("G0 X{:.2} Y{:.2}\n", p.x * self.u, p.y * self.u)
    }

    fn line_to(&self, p: Point) -> String {
        format!("G1 X{:.2} Y{:.2}\n", p.x * self.u, p.y * self.u)
    }

    /// `I`/`J` are the centre relative to the arc's start, not absolute, per RS-274.
    fn arc_to(
        &self,
        from: Point,
        to: Point,
        center: Point,
        _radius: f64,
        sweep: f64,
    ) -> Option<String> {
        let (i, j) = (
            (center.x - from.x) * self.u,
            (center.y - from.y) * self.u,
        );
        let g = if sweep < 0.0 { "G2" } else { "G3" };
        Some(format!(
            "{g} X{:.2} Y{:.2} I{:.2} J{:.2}\n",
            to.x * self.u,
            to.y * self.u,
            i,
            j
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Segment;
    use crate::profile::Protocol;
    use std::f64::consts::PI;

    fn profile() -> MachineProfile {
        MachineProfile {
            name: "gcode test".into(),
            protocol: Protocol::Gcode,
            units_per_mm: 1.0,
            header: "G21;G90;\n".into(),
            footer: "M2\n".into(),
            overcut_mm: 0.0,
            ..MachineProfile::default()
        }
    }

    #[test]
    fn a_line_is_one_g1() {
        let tp = Toolpath {
            segments: vec![Segment::Line { a: Point::new(0.0, 0.0), b: Point::new(10.0, 5.0) }],
            closed: false,
        };
        let out = emit(&profile(), &[tp]);
        assert!(out.contains("G1 X10.00 Y5.00"), "{out}");
    }

    #[test]
    fn a_half_circle_is_one_g3_with_centre_offsets() {
        // Half circle, radius 10, about (0, 0), starting at (10, 0) and running CCW to (-10, 0).
        let tp = Toolpath {
            segments: vec![Segment::Arc {
                center: Point::new(0.0, 0.0),
                radius: 10.0,
                start: 0.0,
                sweep: PI,
            }],
            closed: false,
        };
        let out = emit(&profile(), &[tp]);
        assert!(out.contains("G3 X-10.00 Y0.00 I-10.00 J0.00"), "{out}");
        // One arc command, not a fan of chords.
        assert_eq!(out.matches("G3").count(), 1, "{out}");
        assert!(!out.contains("G1"), "an arc must not be flattened: {out}");
    }

    #[test]
    fn a_clockwise_arc_is_g2() {
        let tp = Toolpath {
            segments: vec![Segment::Arc {
                center: Point::new(0.0, 0.0),
                radius: 10.0,
                start: 0.0,
                sweep: -PI / 2.0,
            }],
            closed: false,
        };
        let out = emit(&profile(), &[tp]);
        assert!(out.contains("G2 "), "{out}");
    }

    #[test]
    fn a_dry_run_flattens_the_arc_instead_of_using_g3() {
        let tp = Toolpath::circle(Point::ZERO, 10.0);
        let out = emit_with_pen(&profile(), &[tp], Pen::Up);
        assert!(!out.contains("G3"), "{out}");
        assert!(out.matches("G0").count() > 4, "{out}");
    }
}
