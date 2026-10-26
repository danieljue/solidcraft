//! Protocol post-processors. A post-processor turns toolpaths into bytes for one machine
//! family. Add a module here for each protocol.
//!
//! Everything protocol-independent lives in [`emit_generic`]: travel to each contour, the
//! pen/overcut rules, flattening of curves a controller cannot express, and the closing
//! safety lift. A protocol module supplies only its command words through [`CommandSet`],
//! which is what keeps a vinyl knife, a plasma torch and a laser emitting the same geometry
//! from the same toolpath code.

pub mod csr;
pub mod dmpl;
pub mod gcode;
pub mod gpgl;
pub mod hpgl;

use crate::geom::{Point, Segment, Toolpath};
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

/// The command words one controller understands.
///
/// A post-processor implements this and nothing else. `arc_to` returns `None` when the
/// controller has no arc command, in which case the shared loop flattens the arc into
/// chords at [`CommandSet::flatten_tol`] — so a format that cannot express a curve still
/// cuts the same shape, it just uploads more points.
pub trait CommandSet {
    /// Travel with the tool clear of the material.
    fn move_to(&self, p: Point) -> String;
    /// Cut or mark from the current position to `p`.
    fn line_to(&self, p: Point) -> String;

    /// A circular arc from `from` to `to` about `center`. `sweep` is in radians and is
    /// positive counter-clockwise, which is the sign convention every controller uses once
    /// it is converted (G-code picks `G2`/`G3`, HPGL takes degrees).
    fn arc_to(
        &self,
        _from: Point,
        _to: Point,
        _center: Point,
        _radius: f64,
        _sweep: f64,
    ) -> Option<String> {
        None
    }

    /// Chord tolerance for curves this format cannot express, in millimetres.
    ///
    /// 0.05 mm is below what a drag knife resolves on vinyl, and for a plasma torch it is
    /// an order of magnitude finer than the kerf.
    fn flatten_tol(&self) -> f64 {
        0.05
    }
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
            // Sample the segment rather than only its end: an arc can bulge past its
            // endpoints, and the carriage would hit the end stop in the middle of it.
            for p in seg.polyline(0.5) {
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
/// The geometry, the pen/overcut rules and curve flattening live here so every protocol
/// behaves the same; `cmds` supplies only the command words.
pub(crate) fn emit_generic(
    profile: &MachineProfile,
    toolpaths: &[Toolpath],
    pen: Pen,
    cmds: &dyn CommandSet,
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
        out.push_str(&cmds.move_to(start));
        blade_down = false;
        cursor = start;

        for seg in &tp.segments {
            let down = pen == Pen::Down;
            out.push_str(&segment_commands(cmds, seg, down));
            blade_down = down;
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
                        out.push_str(&cmds.line_to(over));
                        blade_down = true;
                    }
                    Pen::Up => {
                        out.push_str(&cmds.move_to(over));
                        blade_down = false;
                    }
                }
                cursor = over;
            }
        }
    }

    // Safety: never hand the machine a stream that ends with the knife in the vinyl.
    if blade_down {
        out.push_str(&cmds.move_to(cursor));
    }

    out.push_str(&profile.footer);
    out
}

/// The commands for one segment: a native arc when the controller has one, otherwise the
/// segment flattened to chords. A straight line flattens to a single command, so this costs
/// nothing on the common path.
fn segment_commands(cmds: &dyn CommandSet, seg: &Segment, down: bool) -> String {
    if let Segment::Arc { center, radius, sweep, .. } = *seg {
        // A dry run never uses the arc word: pen state is expressed by the command word in
        // every dialect here, and a flattened move is unambiguous about the tool being up.
        if down {
            if let Some(txt) = cmds.arc_to(seg.start(), seg.end(), center, radius, sweep) {
                return txt;
            }
        }
    }
    let mut out = String::new();
    for p in seg.polyline(cmds.flatten_tol()).into_iter().skip(1) {
        let cmd = if down { cmds.line_to(p) } else { cmds.move_to(p) };
        out.push_str(&cmd);
    }
    out
}

#[derive(Debug, thiserror::Error)]
pub enum PostError {
    #[error("unsupported protocol: {0}")]
    Unsupported(String),
    #[error("design outside machine travel: {0}")]
    OutOfBounds(String),
}
