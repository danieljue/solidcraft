//! Protocol post-processors. A post-processor turns toolpaths into bytes for
//! one machine family. Add a module here for each protocol.

pub mod csr;
pub mod dmpl;
pub mod gcode;
pub mod gpgl;
pub mod hpgl;

use crate::geom::{Point, Toolpath};
use crate::profile::{MachineProfile, Protocol};

/// Which pen state every segment is emitted in.
///
/// A dry run must not depend on rewriting generated text: a `D` inside a header,
/// footer or coordinate could be corrupted by a string replace. Choosing the
/// command word in the post-processor keeps the byte stream structurally valid.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Pen {
    Down,
    Up,
}

pub fn emit(profile: &MachineProfile, toolpaths: &[Toolpath]) -> Result<String, PostError> {
    emit_with_pen(profile, toolpaths, Pen::Down)
}

/// Like [`emit`], but every draw is emitted as a pen-up move (`pen = Pen::Up`).
/// Use for dry runs: the machine follows the geometry without cutting.
pub fn emit_with_pen(
    profile: &MachineProfile,
    toolpaths: &[Toolpath],
    pen: Pen,
) -> Result<String, PostError> {
    // Travel limits: refuse a job that would drive the carriage past its end stop.
    // The app's roll-fit clamp already keeps a design on the material; this is the
    // second gate, against the machine's own travel, which the roll width cannot know.
    check_limits(profile, toolpaths)?;
    Ok(match profile.protocol {
        Protocol::Dmpl => dmpl::emit_with_pen(profile, toolpaths, pen),
        Protocol::Hpgl => hpgl::emit_with_pen(profile, toolpaths, pen),
        Protocol::Gpgl => gpgl::emit_with_pen(profile, toolpaths, pen),
        Protocol::Gcode => gcode::emit_with_pen(profile, toolpaths, pen),
        Protocol::Csr => csr::emit_with_pen(profile, toolpaths, pen),
    })
}

fn check_limits(profile: &MachineProfile, toolpaths: &[Toolpath]) -> Result<(), PostError> {
    let (max_x, max_y) = match (profile.max_x_mm, profile.max_y_mm) {
        (None, None) => return Ok(()),
        (x, y) => (x, y),
    };
    for tp in toolpaths {
        for seg in &tp.segments {
            let p = seg.end();
            if let Some(mx) = max_x {
                if p.x > mx + 1e-9 {
                    return Err(PostError::OutOfBounds(format!(
                        "design reaches x = {:.2} mm, past the machine's {:.2} mm travel",
                        p.x, mx
                    )));
                }
            }
            if let Some(my) = max_y {
                if p.y > my + 1e-9 {
                    return Err(PostError::OutOfBounds(format!(
                        "design reaches y = {:.2} mm, past the machine's {:.2} mm travel",
                        p.y, my
                    )));
                }
            }
        }
    }
    Ok(())
}

/// Shared emission loop used by every protocol module.
///
/// Structure per toolpath: travel to the contour's start with the blade up, then one
/// continuous pass. A closed contour runs past its own start (overcut) so the entry and
/// exit cuts separate instead of leaving a vinyl hinge. The stream always ends with a
/// single safety lift: a stream that stops on a draw leaves the knife pressed into the
/// material and the operator has to free it by hand.
///
/// `move_cmd` and `draw_cmd` supply the protocol's command words; the geometry and the
/// pen/overcut rules live here so every protocol behaves the same.
pub(crate) fn emit_generic(
    profile: &MachineProfile,
    toolpaths: &[Toolpath],
    pen: Pen,
    move_cmd: &dyn Fn(Point) -> String,
    draw_cmd: &dyn Fn(Point) -> String,
) -> String {
    let mut out = String::new();
    out.push_str(&profile.header);
    out.push_str(&profile.setup_commands());

    let mut cursor = Point::new(0.0, 0.0);
    let mut blade_down = false;

    for tp in toolpaths {
        let Some(first) = tp.segments.first() else {
            continue;
        };
        // A job must never begin with the blade pressed into the material: travel to
        // the contour's start with the pen up, even if the design starts at the origin.
        let start = first.start();
        out.push_str(&move_cmd(start));
        blade_down = false;
        cursor = start;

        for seg in &tp.segments {
            match pen {
                Pen::Down => {
                    out.push_str(&draw_cmd(seg.end()));
                    blade_down = true;
                }
                Pen::Up => {
                    out.push_str(&move_cmd(seg.end()));
                    blade_down = false;
                }
            }
            cursor = seg.end();
        }

        // Overcut: keep cutting past the start point. The final segment already drew
        // back to the start, so the blade stays down through the overlap — lifting here
        // would puncture the vinyl at the one point the overcut is meant to hide.
        if tp.closed && profile.overcut_mm > 0.0 {
            let end = first.end();
            let (dx, dy) = (end.x - start.x, end.y - start.y);
            let len = (dx * dx + dy * dy).sqrt();
            if len > 1e-9 {
                let k = profile.overcut_mm / len;
                let over = Point::new(start.x + dx * k, start.y + dy * k);
                match pen {
                    Pen::Down => {
                        out.push_str(&draw_cmd(over));
                        blade_down = true;
                    }
                    Pen::Up => {
                        out.push_str(&move_cmd(over));
                        blade_down = false;
                    }
                }
                cursor = over;
            }
        }
    }

    // Safety: never hand the machine a stream that ends with the knife in the vinyl.
    if blade_down {
        out.push_str(&move_cmd(cursor));
    }

    out.push_str(&profile.footer);
    out
}

#[derive(Debug, thiserror::Error)]
pub enum PostError {
    #[error("unsupported protocol: {0}")]
    Unsupported(String),
    #[error("design outside machine travel: {0}")]
    OutOfBounds(String),
}
