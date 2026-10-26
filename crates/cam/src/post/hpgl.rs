//! HPGL post-processor (Hewlett-Packard Graphics Language, and the near-clone
//! GPGL used by Graphtec and Silhouette).
//!
//! The HPGL protocol definition gives the canonical forms for this family:
//!   MC=PU;PA%.0f,%.0f;   (move — pen up, absolute position)
//!   LC=PD%.0f,%.0f;      (line — pen down, cut to position)
//!   HE=IN;               (header)
//!   FD=!PG;              (feed/eject)
//!   XU=40 YU=40          (40 plotter units per mm)
//!
//! The move carries an explicit `PA` absolute-mode command between the pen word
//! and the coordinates. `PUx,y;` is also legal HPGL, but some Chinese controllers
//! only parse the two-command form, and matching the vendor's own output is the
//! safest thing to send to a machine we cannot interrogate. The draw does NOT
//! carry `PA`: absolute mode persists once set, and the draw form omits it.
//!
//! Arcs use `AA` — arc absolute — which takes the centre in plotter units and the sweep
//! in degrees, positive counter-clockwise. The start point is wherever the pen already is.

use crate::geom::{Point, Toolpath};
use crate::post::{CommandSet, Pen};
use crate::profile::MachineProfile;

pub fn emit(profile: &MachineProfile, toolpaths: &[Toolpath]) -> String {
    emit_with_pen(profile, toolpaths, Pen::Down)
}

pub fn emit_with_pen(profile: &MachineProfile, toolpaths: &[Toolpath], pen: Pen) -> String {
    super::emit_generic(profile, toolpaths, pen, &Hpgl { u: profile.units_per_mm })
}

struct Hpgl {
    u: f64,
}

impl CommandSet for Hpgl {
    fn move_to(&self, p: Point) -> String {
        format!("PU;PA{:.0},{:.0};", p.x * self.u, p.y * self.u)
    }

    fn line_to(&self, p: Point) -> String {
        format!("PD{:.0},{:.0};", p.x * self.u, p.y * self.u)
    }

    /// `AA x,y,angle;`: the point is the arc's centre, the angle its sweep in degrees.
    fn arc_to(
        &self,
        _from: Point,
        _to: Point,
        center: Point,
        _radius: f64,
        sweep: f64,
    ) -> Option<String> {
        Some(format!(
            "AA{:.0},{:.0},{:.2};",
            center.x * self.u,
            center.y * self.u,
            sweep.to_degrees()
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Segment;
    use crate::profile::{MachineProfile, Protocol};
    use std::f64::consts::PI;

    fn hpgl_profile() -> MachineProfile {
        MachineProfile {
            name: "HPGL test".into(),
            protocol: Protocol::Hpgl,
            units_per_mm: 40.0,
            header: "IN;".into(),
            footer: "!PG;".into(),
            ..MachineProfile::default()
        }
    }

    #[test]
    fn move_carries_absolute_mode_and_draw_does_not() {
        // HPGL canonical form: MC=PU;PA%.0f,%.0f; and LC=PD%.0f,%.0f;
        // A move must set absolute mode explicitly; the draw relies on it persisting.
        let tp = Toolpath {
            segments: vec![Segment::Line {
                a: Point::new(0.0, 0.0),
                b: Point::new(10.0, 20.0),
            }],
            closed: false,
        };
        let out = emit(&hpgl_profile(), &[tp]);
        assert!(out.contains("PU;PA0,0;"), "move should set absolute mode: {out}");
        assert!(out.contains("PD400,800;"), "draw should be a bare PD with coordinates: {out}");
        assert!(!out.contains("PD;PA"), "draw must not repeat absolute mode: {out}");
    }

    #[test]
    fn header_and_footer_wrap_the_job() {
        let tp = Toolpath {
            segments: vec![Segment::Line {
                a: Point::new(0.0, 0.0),
                b: Point::new(1.0, 1.0),
            }],
            closed: false,
        };
        let out = emit(&hpgl_profile(), &[tp]);
        assert!(out.starts_with("IN;"), "header first: {out}");
        assert!(out.ends_with("!PG;"), "footer last: {out}");
    }

    #[test]
    fn an_arc_is_one_aa_command_in_degrees() {
        let tp = Toolpath {
            segments: vec![Segment::Arc {
                center: Point::new(0.0, 0.0),
                radius: 10.0,
                start: 0.0,
                sweep: PI / 2.0,
            }],
            closed: false,
        };
        let out = emit(&hpgl_profile(), &[tp]);
        assert!(out.contains("AA0,0,90.00;"), "expected a quarter-turn AA: {out}");
        assert_eq!(out.matches("AA").count(), 1, "{out}");
        assert!(!out.contains("PD"), "the arc must not be flattened: {out}");
    }
}
