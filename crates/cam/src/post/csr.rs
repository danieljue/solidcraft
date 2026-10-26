//! CSR post-processor: a relative-only vinyl-cutter protocol.
//!
//! The CSR protocol definition gives:
//!   MC=R%.0f,%.0f/r/l   (move — pen up, relative position)
//!   LC=I%.0f,%.0f/r/l   (line — pen down, relative position)
//!   HE=                 (no header)
//!   FD=                 (no footer)
//!   XU=40 YU=40         (40 plotter units per mm)
//!
//! Every command is a delta from where the carriage currently is, so unlike every other
//! protocol this emitter must carry position state through the whole job. The `/r/l`
//! suffix in the vendor table is a format annotation (relative/line), not part of the
//! bytes sent: the commands are `R<dx>,<dy>;` and `I<dx>,<dy>;`.
//!
//! Because there is no explicit pen-up word, the pen state is implied by the command:
//! `R` moves without cutting, `I` cuts while moving. A job that ends on an `I` leaves the
//! blade pressed into the vinyl, so the final act is a zero-length `R0,0;` — a pen-up move
//! that goes nowhere.

use crate::geom::{Point, Toolpath};
use crate::post::Pen;
use crate::profile::MachineProfile;

pub fn emit(profile: &MachineProfile, toolpaths: &[Toolpath]) -> String {
    emit_with_pen(profile, toolpaths, Pen::Down)
}

pub fn emit_with_pen(profile: &MachineProfile, toolpaths: &[Toolpath], pen: Pen) -> String {
    let u = profile.units_per_mm;
    let mut out = String::new();
    // CSR's HE and FD are both empty, so the header and footer contribute nothing unless
    // a particular model's profile declares them. setup_commands still runs so a profile
    // can inject a speed or force word if its controller needs one.
    out.push_str(&profile.header);
    out.push_str(&profile.setup_commands());

    // The carriage's position in millimetres. Every emitted command is a delta from here.
    let mut cursor = Point::new(0.0, 0.0);
    let mut blade_down = false;

    for tp in toolpaths {
        let Some(first) = tp.segments.first() else {
            continue;
        };
        // Travel to the contour's start with the pen up. A job must never begin with the
        // blade pressed into the material.
        let start = first.start();
        let (dx, dy) = ((start.x - cursor.x) * u, (start.y - cursor.y) * u);
        out.push_str(&format!("R{dx:.0},{dy:.0};"));
        blade_down = false;
        cursor = start;

        for seg in &tp.segments {
            let end = seg.end();
            let (dx, dy) = ((end.x - cursor.x) * u, (end.y - cursor.y) * u);
            match pen {
                Pen::Down => {
                    out.push_str(&format!("I{dx:.0},{dy:.0};"));
                    blade_down = true;
                }
                Pen::Up => {
                    out.push_str(&format!("R{dx:.0},{dy:.0};"));
                    blade_down = false;
                }
            }
            cursor = end;
        }

        // Overcut: keep cutting past the start point so the entry and exit cuts separate
        // instead of leaving a vinyl hinge. The final segment already drew back to the
        // start, so the blade stays down through the overlap.
        if tp.closed && profile.overcut_mm > 0.0 {
            let end = first.end();
            let (sx, sy) = (end.x - start.x, end.y - start.y);
            let len = (sx * sx + sy * sy).sqrt();
            if len > 1e-9 {
                let k = profile.overcut_mm / len;
                let over = Point::new(start.x + sx * k, start.y + sy * k);
                let (dx, dy) = ((over.x - cursor.x) * u, (over.y - cursor.y) * u);
                match pen {
                    Pen::Down => {
                        out.push_str(&format!("I{dx:.0},{dy:.0};"));
                        blade_down = true;
                    }
                    Pen::Up => {
                        out.push_str(&format!("R{dx:.0},{dy:.0};"));
                        blade_down = false;
                    }
                }
                cursor = over;
            }
        }
    }

    // Safety: a stream that stops on a draw leaves the knife in the vinyl. CSR has no
    // pen-up word, so issue a zero-length move — the pen lifts without going anywhere.
    if blade_down {
        let _ = cursor;
        out.push_str("R0,0;");
    }

    out.push_str(&profile.footer);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Segment;
    use crate::profile::{MachineProfile, Protocol};

    fn csr_profile() -> MachineProfile {
        MachineProfile {
            name: "CSR test".into(),
            protocol: Protocol::Csr,
            units_per_mm: 40.0,
            header: String::new(),
            footer: String::new(),
            ..MachineProfile::default()
        }
    }

    #[test]
    fn commands_are_deltas_not_absolute_positions() {
        // A move to (10,20) from the origin is a delta of 10,20; the next draw to (30,20)
        // is a delta of 20,0 — NOT the absolute 30,20.
        let tp = Toolpath {
            segments: vec![
                Segment::Line {
                    from: Point::new(10.0, 20.0),
                    to: Point::new(30.0, 20.0),
                },
                Segment::Line {
                    from: Point::new(30.0, 20.0),
                    to: Point::new(30.0, 40.0),
                },
            ],
            closed: false,
        };
        let out = emit(&csr_profile(), &[tp]);
        assert!(
            out.contains("R400,800;"),
            "first move should be the delta from origin: {out}"
        );
        assert!(
            out.contains("I800,0;"),
            "first draw should be the delta 20mm in x: {out}"
        );
        assert!(
            out.contains("I0,800;"),
            "second draw should be the delta 20mm in y: {out}"
        );
        assert!(
            !out.contains("I1200,800;"),
            "must not emit absolute coordinates: {out}"
        );
    }

    #[test]
    fn empty_header_and_footer_by_default() {
        let tp = Toolpath {
            segments: vec![Segment::Line {
                from: Point::new(0.0, 0.0),
                to: Point::new(1.0, 0.0),
            }],
            closed: false,
        };
        let out = emit(&csr_profile(), &[tp]);
        assert!(
            out.starts_with('R'),
            "no header, so the stream starts with the first move: {out}"
        );
        assert!(
            out.ends_with("R0,0;"),
            "job should end with a pen-up move, got: {out}"
        );
    }

    #[test]
    fn pen_up_uses_r_and_pen_down_uses_i() {
        let tp = Toolpath {
            segments: vec![Segment::Line {
                from: Point::new(5.0, 0.0),
                to: Point::new(15.0, 0.0),
            }],
            closed: false,
        };
        let down = emit(&csr_profile(), &[tp.clone()]);
        assert!(down.contains("I"), "pen down must draw with I");
        let up = emit_with_pen(&csr_profile(), &[tp], Pen::Up);
        assert!(!up.contains("I"), "pen up must not draw: {up}");
        assert!(up.contains("R"), "pen up must move with R");
    }

    #[test]
    fn overcut_draws_past_the_start() {
        // A closed 10x10 square with a 1mm overcut. The overcut extends past the start
        // point along the FIRST segment's direction — the direction the blade travels when
        // it leaves the start — so the overlap lands on the beginning of the cut. The first
        // segment runs (0,0)->(10,0), so 1mm past the start is (1,0), a delta of +40 in x
        // from the return point. This matches emit_generic's behaviour for every protocol.
        let tp = Toolpath::rectangle(0.0, 0.0, 10.0, 10.0);
        let mut profile = csr_profile();
        profile.overcut_mm = 1.0;
        let out = emit(&profile, &[tp]);
        assert!(
            out.contains("I40,0;"),
            "overcut should overshoot along the first segment: {out}"
        );
    }
}
