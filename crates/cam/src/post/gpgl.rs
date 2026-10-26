//! GPGL post-processor (Graphtec / Silhouette family).
//!
//! Commands are prefixed with ETX (0x03). Init is `\x03\x03H`; a move is `\x03M x,y`
//! (pen up) and a draw is `\x03D x,y` (pen down). Speed is `\x03!<v>` and force is
//! `\x03FX<f>,1`. Coordinates are 20 units/mm on the Silhouette family. The profile
//! carries the init/finish strings and the setup-command templates; this module only
//! supplies the move and draw words. Transcribed from Inkcut's `gpgl.py`.

use crate::geom::{Point, Toolpath};
use crate::post::Pen;
use crate::profile::MachineProfile;

pub fn emit(profile: &MachineProfile, toolpaths: &[Toolpath]) -> String {
    emit_with_pen(profile, toolpaths, Pen::Down)
}

pub fn emit_with_pen(profile: &MachineProfile, toolpaths: &[Toolpath], pen: Pen) -> String {
    let u = profile.units_per_mm;
    super::emit_generic(
        profile,
        toolpaths,
        pen,
        &|p: Point| format!("\u{3}M{:.0},{:.0}", p.x * u, p.y * u),
        &|p: Point| format!("\u{3}D{:.0},{:.0}", p.x * u, p.y * u),
    )
}
