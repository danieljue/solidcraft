//! JSON machine profiles. These are data, not code: a profile says which
//! protocol a cutter speaks and how it is wired. Do not ship a proprietary
//! vendor database; transcribe protocol parameters from public docs or a
//! profile the user owns.

use serde::{Deserialize, Serialize};

use crate::process::Process;

/// Number formatting for a machine command: whole values stay whole, fractions keep up
/// to two decimals. Cutter firmware rejects `400.0` where it expects `400`.
pub fn fmt_num(v: f64) -> String {
    if v.fract().abs() < 1e-9 {
        format!("{:.0}", v)
    } else {
        let s = format!("{:.2}", v);
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineProfile {
    pub name: String,
    pub protocol: Protocol,
    /// What the machine does to the material. A profile that does not name one is a vinyl
    /// cutter, which is what this crate grew out of; a plasma table, laser or router sets its
    /// own and the same toolpath code compensates for its cut width.
    #[serde(default)]
    pub process: Process,
    pub units_per_mm: f64,
    pub header: String,
    pub footer: String,
    #[serde(default)]
    pub serial: Option<SerialSettings>,
    /// Extra distance to keep cutting past the start point of a closed contour
    /// so the two ends actually separate instead of leaving a hinge of vinyl.
    /// A drag knife leaves a thin bridge where the entry and exit cuts fail to
    /// meet; 0.1–0.3 mm closes it. 0 disables overcut.
    #[serde(default)]
    pub overcut_mm: f64,

    /// Largest allowed deviation, in millimetres, when a curve has to be approximated by
    /// chords because the controller has no arc or Bézier command.
    ///
    /// A drag knife resolves about a tenth of a millimetre on vinyl, so 0.05 mm is
    /// invisible; a plasma torch with a 1 mm kerf does not benefit from anything finer,
    /// and coarse chords make the job far smaller to stream. Profiles that emit native
    /// arcs (`G2`/`G3`, HPGL `AA`) ignore this.
    #[serde(default = "default_arc_tolerance_mm")]
    pub arc_tolerance_mm: f64,

    // ---- Machine setup commands -------------------------------------------------
    // The SC2 family's serial command set uses `!FS%.0f;` for force, `VS%.0f;` for
    // speed and `US%.0f;` for up-speed. `{}` is replaced by the value. A command is emitted only when BOTH template and value
    // are set, so an unconfigured cutter keeps its front-panel settings.
    /// Cutting force/pressure. Template e.g. `!FS{};`.
    #[serde(default)]
    pub force_cmd: Option<String>,
    #[serde(default)]
    pub force: Option<f64>,
    /// Cutting speed. Template e.g. `VS{};`.
    #[serde(default)]
    pub speed_cmd: Option<String>,
    #[serde(default)]
    pub speed: Option<f64>,
    /// Pen-up (travel) speed. Template e.g. `US{};`.
    #[serde(default)]
    pub up_speed_cmd: Option<String>,
    #[serde(default)]
    pub up_speed: Option<f64>,

    // ---- Travel limits ----------------------------------------------------------
    /// Machine travel width in mm. A job whose X extent exceeds this would drive the
    /// carriage into its end stop. `None` means "no declared limit".
    #[serde(default)]
    pub max_x_mm: Option<f64>,
    #[serde(default)]
    pub max_y_mm: Option<f64>,

    // ---- Path quality -----------------------------------------------------------
    /// Drag-knife corner compensation in mm: how far the blade tip trails the pivot.
    /// 0 disables. Typical vinyl blade: 0.25–0.5 mm.
    #[serde(default)]
    pub blade_offset_mm: f64,
    /// Drop line segments shorter than this (mm). Prevents blobs and wasted cycles.
    #[serde(default)]
    pub min_line_mm: f64,
    /// Join open contours whose ends meet within this distance (mm). 0 disables.
    #[serde(default)]
    pub linemerge_tol_mm: f64,
    /// Cut each contour this many times. 1 = once. Thick material needs 2–3.
    #[serde(default = "one_pass")]
    pub repeat_passes: u32,

    // ---- Tool presets -----------------------------------------------------------
    /// Named blade/pen presets. A 30-degree drag knife needs a shorter trail than a
    /// 60-degree blade, and a pen needs neither downforce nor corner compensation.
    /// A single flat `blade_offset_mm` cannot express that, so the operator picks a
    /// preset and its values take effect without editing the profile by hand.
    #[serde(default)]
    pub tool_presets: Vec<ToolPreset>,
    /// Name of the preset in `tool_presets` currently in the holder. Matching is
    /// case-insensitive. `None` means "use the profile's flat values".
    #[serde(default)]
    pub active_tool: Option<String>,
}

/// One blade or pen that can sit in the holder. A field left `None` (or zero for a
/// force/speed) means "leave the machine's own setting alone" — a pen must not be
/// sent a cutting force, and a scoring tool runs with almost none.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ToolPreset {
    pub name: String,
    /// Downforce override. `None` or `0` sends no force command.
    #[serde(default)]
    pub force: Option<f64>,
    /// Cutting-speed override. `None` or `0` sends no speed command.
    #[serde(default)]
    pub speed: Option<f64>,
    /// Drag-knife trail for this blade geometry, in mm. A pen or a crease tool is 0.
    #[serde(default)]
    pub blade_offset_mm: f64,
    /// Passes for this tool. 0 means "inherit the profile's value".
    #[serde(default)]
    pub repeat_passes: u32,
}

fn one_pass() -> u32 {
    1
}

impl MachineProfile {
    /// The preset named by `active_tool`, if one matches. Case-insensitive so a
    /// profile written "60 degree blade" still matches "60 Degree Blade".
    pub fn active_preset(&self) -> Option<&ToolPreset> {
        let name = self.active_tool.as_ref()?;
        self.tool_presets.iter().find(|p| p.name.eq_ignore_ascii_case(name))
    }

    /// Downforce that will actually be sent. The active preset wins; a preset value
    /// of 0 or None means "keep the machine's own setting", so it does NOT override.
    pub fn effective_force(&self) -> Option<f64> {
        match self.active_preset().and_then(|p| p.force) {
            Some(v) if v > 0.0 => Some(v),
            _ => self.force,
        }
    }

    /// Cutting speed that will actually be sent, with the same override rule as force.
    pub fn effective_speed(&self) -> Option<f64> {
        match self.active_preset().and_then(|p| p.speed) {
            Some(v) if v > 0.0 => Some(v),
            _ => self.speed,
        }
    }

    /// Drag-knife trail that will actually be applied. A pen preset declares 0, which
    /// legitimately means "no compensation" — so a preset present-but-zero still wins
    /// over the profile's flat value. That is the whole point of selecting a pen.
    pub fn effective_blade_offset_mm(&self) -> f64 {
        match self.active_preset() {
            Some(p) => p.blade_offset_mm,
            None => self.blade_offset_mm,
        }
    }

    /// Passes that will actually be cut. A preset value of 0 means "inherit".
    pub fn effective_repeat_passes(&self) -> u32 {
        match self.active_preset().map(|p| p.repeat_passes) {
            Some(v) if v > 0 => v,
            _ => self.repeat_passes,
        }
    }

    /// Setup commands emitted once after the header: force, then speed, then pen-up
    /// speed. Empty when the profile declares no values. Order follows the vendor
    /// profile file. Settings must come AFTER `IN;`, which resets a HPGL/DMPL machine
    /// to its defaults and would wipe anything sent before it.
    pub fn setup_commands(&self) -> String {
        let mut out = String::new();
        for (tpl, val) in [(&self.force_cmd, self.effective_force()), (&self.speed_cmd, self.effective_speed()), (&self.up_speed_cmd, self.up_speed)]
        {
            if let (Some(t), Some(v)) = (tpl, val) {
                // Zero means "do not send": a pen tool must not be driven down with a
                // cutting force, and a cutter ignores `!FS0;` on some firmware.
                if v > 0.0 {
                    out.push_str(&t.replace("{}", &fmt_num(v)));
                }
            }
        }
        out
    }
}

/// Chord tolerance when a profile does not name one: fine enough for a knife on vinyl.
fn default_arc_tolerance_mm() -> f64 {
    0.05
}

impl Default for MachineProfile {
    fn default() -> Self {
        Self {
            name: "Unnamed cutter".into(),
            protocol: Protocol::Dmpl,
            process: Process::default(),
            units_per_mm: 40.0,
            header: String::new(),
            footer: String::new(),
            serial: None,
            overcut_mm: 0.0,
            arc_tolerance_mm: default_arc_tolerance_mm(),
            force_cmd: None,
            force: None,
            speed_cmd: None,
            speed: None,
            up_speed_cmd: None,
            up_speed: None,
            max_x_mm: None,
            max_y_mm: None,
            blade_offset_mm: 0.0,
            min_line_mm: 0.0,
            linemerge_tol_mm: 0.0,
            repeat_passes: 1,
            tool_presets: Vec::new(),
            active_tool: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Protocol {
    Dmpl,
    Hpgl,
    Gpgl,
    Gcode,
    /// CSR is a relative-only protocol: `R<dx>,<dy>;` moves the pen without drawing and
    /// `I<dx>,<dy>;` draws to a point relative to where the pen currently is. Vendor
    /// documentation annotates these as MC=R%.0f,%.0f/r/l and LC=I%.0f,%.0f/r/l, where
    /// `/r/l` means relative/line and is an annotation, not part of the bytes sent. There is
    /// no header and no footer. Because every command is a delta, the emitter must track the
    /// carriage's position as it goes.
    Csr,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerialSettings {
    pub port: String,
    pub baud: u32,
    #[serde(default)]
    pub flow: FlowControl,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum FlowControl {
    #[default]
    None,
    RtsCts,
    XonXoff,
}

/// The SC2 profile as shipped, embedded at compile time.
///
/// The app's `Default` used to build this machine's settings as a struct literal while
/// the same settings also lived in `profiles/uscutter_sc2.json`. Two copies of one
/// machine drift: a blade offset or a travel limit added to the JSON never reached the
/// default, so a launch without `--profile` silently ran a weaker configuration than a
/// launch with it. Exposing the embedded text makes the JSON the only source of truth.
pub const USCUTTER_SC2_PROFILE_JSON: &str = include_str!("../profiles/uscutter_sc2.json");

#[cfg(test)]
mod tests {
    use super::*;

    /// Every profile we ship is embedded and parsed here. Serde ignores unknown keys
    /// silently, so a mistyped key (`units_units_per_mm` for `units_per_mm`) or a
    /// field that quietly lost its `#[serde(default)]` would otherwise ship as a
    /// profile that loads with a zeroed or missing value. Loading each one and
    /// asserting the values we depend on catches that at build time.
    #[test]
    fn shipped_profiles_parse_and_keep_their_essentials() {
        let hpgl: MachineProfile = serde_json::from_str(include_str!("../profiles/hpgl.json")).expect("hpgl.json parses");
        assert_eq!(hpgl.protocol, Protocol::Hpgl);
        // A wrong key name here would leave the default 0.0 in place and the profile
        // would emit every coordinate at the origin.
        assert!((hpgl.units_per_mm - 40.0).abs() < 1e-9, "units_per_mm must survive parsing, got {}", hpgl.units_per_mm);
        // An empty header means no absolute-mode init; an empty footer means the
        // machine never feeds the material out.
        assert!(!hpgl.header.is_empty(), "hpgl profile needs a header");
        assert!(!hpgl.footer.is_empty(), "hpgl profile needs a footer");

        // RDGL is HPGL with a Roland-specific init prefix. The protocol stays Hpgl because
        // the move and draw command forms are identical; only the header differs. The header
        // must carry the vendor's escape sequence, or the controller ignores the job.
        let rdgl: MachineProfile = serde_json::from_str(include_str!("../profiles/rdgl.json")).expect("rdgl.json parses");
        assert_eq!(rdgl.protocol, Protocol::Hpgl);
        assert!(rdgl.header.contains("/e/50A"), "RDGL header must keep the vendor init prefix, got {:?}", rdgl.header);
        assert!(!rdgl.footer.is_empty(), "rdgl profile needs a footer");

        // CSR is relative-only. Its header and footer are legitimately empty in the vendor
        // definition, so this one asserts the opposite of the absolute protocols: nothing is
        // allowed to prefix or suffix the stream. A header added here would be sent before
        // the first relative move and most controllers would reject it.
        let csr: MachineProfile = serde_json::from_str(include_str!("../profiles/csr.json")).expect("csr.json parses");
        assert_eq!(csr.protocol, Protocol::Csr);
        assert!(csr.header.is_empty(), "CSR declares no header; a prefix would be rejected: {:?}", csr.header);
        assert!(csr.footer.is_empty(), "CSR declares no footer; a suffix would be rejected: {:?}", csr.footer);
        assert!((csr.units_per_mm - 40.0).abs() < 1e-9, "CSR units_per_mm must survive parsing, got {}", csr.units_per_mm);

        // A plasma table speaks G-code but is not a vinyl cutter: the process is what carries
        // the cut width, and losing it would silently cut every part undersized.
        let plasma: MachineProfile = serde_json::from_str(include_str!("../profiles/plasma_gcode.json")).expect("plasma_gcode.json parses");
        assert_eq!(plasma.protocol, Protocol::Gcode);
        assert_eq!(plasma.process, Process::Plasma { kerf_mm: 1.5 });
        assert!((plasma.process.half_kerf_mm() - 0.75).abs() < 1e-12);
        assert!((plasma.arc_tolerance_mm - 0.2).abs() < 1e-12, "a torch wants coarser chords than a knife");
    }

    #[test]
    fn sc2_profile_parses_with_its_tool_presets() {
        let sc2: MachineProfile = serde_json::from_str(include_str!("../profiles/uscutter_sc2.json")).expect("uscutter_sc2.json parses");
        assert_eq!(sc2.protocol, Protocol::Dmpl);
        assert!(!sc2.tool_presets.is_empty(), "the SC2 profile should carry its blade presets");
        // The active tool must actually resolve, or the flat values silently win and
        // the operator's chosen blade is ignored.
        assert!(sc2.active_preset().is_some(), "active_tool '{}' should match a preset", sc2.active_tool.as_deref().unwrap_or("<none>"));
        // A drag knife needs a non-zero trail; 0 means the blade is assumed centred.
        assert!(sc2.effective_blade_offset_mm() > 0.0, "an active drag-knife preset should produce a non-zero offset");
    }
}
