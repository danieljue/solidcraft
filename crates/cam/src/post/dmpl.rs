//! DMPL post-processor (US Cutter SC2 family).
//!
//! The SC2 family's protocol definition gives:
//! `EN=DMPL`, `HE=IN;PA;`, `MC=U%.0f,%.0f;`, `LC=D%.0f,%.0f;`,
//! `PU=U;`, `FO=@`, `XU=40`, `YU=40`.
//!
//! DMPL is not HPGL: moves are `U`, draws are `D`, and the terminator is `@`.

use crate::geom::{Point, Segment, Toolpath};
use crate::post::Pen;
use crate::profile::MachineProfile;

pub fn emit(profile: &MachineProfile, toolpaths: &[Toolpath]) -> String {
    emit_with_pen(profile, toolpaths, Pen::Down)
}

pub fn emit_with_pen(profile: &MachineProfile, toolpaths: &[Toolpath], pen: Pen) -> String {
    let u = profile.units_per_mm;
    let mut out = String::new();
    out.push_str(&profile.header);
    // Force, speed and pen-up speed come from the profile's own templates. Empty when
    // the profile declares none, so an unconfigured cutter keeps its panel settings.
    out.push_str(&profile.setup_commands());

    // Where the knife is and whether it is down. Tracked so the job can always end with
    // the blade lifted: a stream that stops on a `D` leaves the knife pressed into the
    // vinyl, and the operator has to free it by hand.
    let mut cursor = Point::new(0.0, 0.0);
    let mut blade_down = false;

    for tp in toolpaths {
        let mut first = true;
        for seg in &tp.segments {
            match seg {
                Segment::Line { from, to } => {
                    if first {
                        out.push_str(&move_cmd(*from, u));
                        first = false;
                    }
                    match pen {
                        Pen::Down => {
                            out.push_str(&draw_cmd(*to, u));
                            blade_down = true;
                        }
                        Pen::Up => {
                            out.push_str(&move_cmd(*to, u));
                            blade_down = false;
                        }
                    }
                    cursor = *to;
                }
            }
        }
        // Close the contour: return to the first point, then — if the profile asks for
        // it — keep going past the start so the entry and exit cuts actually separate.
        // A drag knife leaves a thin vinyl bridge wherever the two cuts fail to meet; a
        // short overlap is what turns a hinge into a free part.
        if tp.closed {
            if let Some(first_seg) = tp.segments.first() {
                let start = first_seg.start();
                let end = first_seg.end();
                // Where the overcut ends, if the profile asks for one. A drag knife leaves a
                // thin vinyl bridge wherever the two cuts fail to meet; a short overlap past
                // the start is what turns a hinge into a free part.
                let over = if profile.overcut_mm > 0.0 {
                    let (dx, dy) = (end.x - start.x, end.y - start.y);
                    let len = (dx * dx + dy * dy).sqrt();
                    if len > 1e-9 {
                        let k = profile.overcut_mm / len;
                        Some(Point::new(start.x + dx * k, start.y + dy * k))
                    } else {
                        None
                    }
                } else {
                    None
                };
                match pen {
                    // The final segment already drew back to the start, so the outline is
                    // closed with the blade still down. Lifting here and dropping again for
                    // the overcut would puncture the vinyl at the one point the overcut is
                    // meant to hide. Stay down, run past the start, and let the safety lift
                    // at the end of the job take the blade up once.
                    Pen::Down => {
                        match over {
                            // Keep cutting past the start; the safety lift at the end of the
                            // job takes the blade up once, after the overlap is complete.
                            Some(o) => {
                                out.push_str(&draw_cmd(o, u));
                                cursor = o;
                                blade_down = true;
                            }
                            // No overcut: lift here so the knife does not sit on the vinyl
                            // while the machine travels to the next contour.
                            None => {
                                out.push_str(&move_cmd(start, u));
                                cursor = start;
                                blade_down = false;
                            }
                        }
                    }
                    // A dry run cannot close anything, so it returns to the start in the air
                    // and traces the overcut there too.
                    Pen::Up => {
                        out.push_str(&move_cmd(start, u));
                        cursor = start;
                        blade_down = false;
                        if let Some(o) = over {
                            out.push_str(&move_cmd(o, u));
                            cursor = o;
                        }
                    }
                }
            }
        }
    }

    // Safety: never hand the machine a stream that ends with the knife in the vinyl.
    // Open paths and overcuts can both finish on a draw, so lift before terminating.
    if blade_down {
        out.push_str(&move_cmd(cursor, u));
    }

    out.push_str(&profile.footer);
    out
}

fn move_cmd(p: Point, units_per_mm: f64) -> String {
    format!("U{:.0},{:.0};", p.x * units_per_mm, p.y * units_per_mm)
}

fn draw_cmd(p: Point, units_per_mm: f64) -> String {
    format!("D{:.0},{:.0};", p.x * units_per_mm, p.y * units_per_mm)
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
            serial: Some(crate::profile::SerialSettings {
                port: "COM4".into(),
                baud: 57600,
                flow: FlowControl::RtsCts,
            }),
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
    #[test]
    fn overcut_follows_pen_state_on_dry_run() {
        let mut p = sc2();
        p.overcut_mm = 0.2;
        let out = emit_with_pen(
            &p,
            &[Toolpath::rectangle(0.0, 0.0, 10.0, 10.0)],
            Pen::Up,
        );
        assert_eq!(out, "IN;PA;U0,0;U400,0;U400,400;U0,400;U0,0;U0,0;U8,0;@");
    }
}
