//! G-code post-processor. `G0` is a rapid (blade up), `G1` is a feed move (blade down):
//! the "implicit" tool-lift mode that Inkcut's `gcode.py` defaults to. The profile's
//! header carries the unit/mode setup (`G21;G90;`) and the footer carries the park or
//! return sequence. Coordinates are millimetres, so `units_per_mm` is 1.0.

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
        &|p: Point| format!("G0 X{:.2} Y{:.2}\n", p.x * u, p.y * u),
        &|p: Point| format!("G1 X{:.2} Y{:.2}\n", p.x * u, p.y * u),
    )
}
