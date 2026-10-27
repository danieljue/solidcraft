//! DM/PL post-processor for the SC-series vinyl cutters.
//!
//! The SC2 family's protocol definition gives the canonical forms used here:
//!   MC=U%.0f,%.0f;       (move — pen up, absolute position)
//!   LC=D%.0f,%.0f;       (line — pen down, cut to position)
//!   HE=IN;PA;            (header)
//!   FO=@                 (feed/eject)
//!   SC=VS%.0f;           (speed)
//!   XU=40 YU=40          (40 units per mm)
//!
//! This dialect has no arc word. A curve is therefore flattened to chords in the shared
//! loop, which is why `arc_to` is left at its default: the controller would reject an
//! `AA` that HPGL accepts, so geometry is the one place the two families differ.

use crate::geom::{Point, Toolpath};
use crate::post::{CommandSet, Pen};
use crate::profile::MachineProfile;

pub fn emit(profile: &MachineProfile, toolpaths: &[Toolpath]) -> String {
    emit_with_pen(profile, toolpaths, Pen::Down)
}

pub fn emit_with_pen(profile: &MachineProfile, toolpaths: &[Toolpath], pen: Pen) -> String {
    super::emit_generic(profile, toolpaths, pen, &Dmpl { u: profile.units_per_mm })
}

struct Dmpl {
    u: f64,
}

impl CommandSet for Dmpl {
    fn move_to(&self, p: Point) -> String {
        format!("U{:.0},{:.0};", p.x * self.u, p.y * self.u)
    }

    fn line_to(&self, p: Point) -> String {
        format!("D{:.0},{:.0};", p.x * self.u, p.y * self.u)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::{FlowControl, MachineProfile, Protocol};

    fn sc2() -> MachineProfile {
        MachineProfile {
            name: "US Cutter SC2".into(),
            protocol: Protocol::Dmpl,
            units_per_mm: 40.0,
            header: "IN;PA;".into(),
            footer: "@".into(),
            serial: Some(crate::profile::SerialSettings { port: "COM4".into(), baud: 57600, flow: FlowControl::RtsCts }),
            overcut_mm: 0.0,
            ..MachineProfile::default()
        }
    }

    #[test]
    fn square_10mm_is_400_units_per_side() {
        let out = emit(&sc2(), &[Toolpath::rectangle(0.0, 0.0, 10.0, 10.0)]);
        assert_eq!(out, "IN;PA;U0,0;D400,0;D400,400;D0,400;D0,0;U0,0;@");
    }

    /// A closed contour must run past its own start point or the blade leaves a hinge.
    /// 0.2 mm at 40 units/mm on a 10 mm edge is 8 units of overlap.
    #[test]
    fn overcut_extends_past_the_start_point() {
        let mut p = sc2();
        p.overcut_mm = 0.2;
        let out = emit(&p, &[Toolpath::rectangle(0.0, 0.0, 10.0, 10.0)]);
        // The blade stays down through the overcut and is lifted once at the end, so the
        // machine is never left with the knife in the vinyl.
        assert_eq!(out, "IN;PA;U0,0;D400,0;D400,400;D0,400;D0,0;D8,0;U8,0;@");
    }

    /// The overcut must not drag the blade on a dry run: same bytes, but as a move.
    ///
    /// There is no second `U0,0;` before the overcut: the final segment already left the
    /// carriage at the contour's start, so returning there first would be a zero-length
    /// move. The shared emission loop omits it.
    #[test]
    fn overcut_follows_pen_state_on_dry_run() {
        let mut p = sc2();
        p.overcut_mm = 0.2;
        let out = emit_with_pen(&p, &[Toolpath::rectangle(0.0, 0.0, 10.0, 10.0)], Pen::Up);
        assert_eq!(out, "IN;PA;U0,0;U400,0;U400,400;U0,400;U0,0;U8,0;@");
    }
}
