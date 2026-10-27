//! The VinylCraft application: design state, transforms, emission and the egui frame.
//!
//! This is the mutable model behind the window and the headless `--mcp` server. Every field is
//! `pub(crate)` so the MCP dispatch layer can read and mutate the model without a second copy
//! of the state or a getter for each field.

use eframe::egui;
use solidcraft_cam::boolean::{boolean_paths, BooleanOp, FillMode};
use solidcraft_cam::filters;
use solidcraft_cam::geom::{Point, Toolpath};
use solidcraft_cam::post;
use solidcraft_cam::profile::{FlowControl as CamFlow, MachineProfile, SerialSettings};
use solidcraft_geom::Vec2;
use solidcraft_io::{read_dxf, read_svg_colored, Geom2};
use solidcraft_sketch::text_geometry;
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;
use std::sync::Arc;
use std::sync::mpsc::{self, Receiver};
use std::sync::atomic::Ordering;

use crate::geometry::*;
use crate::serial::*;
use crate::jobs::*;
use crate::control::*;
use crate::dispatch::{state_json, call_vinyl_tool};

/// The part of the model a destructive operation can replace.
///
/// Offset, boolean and text all rewrite `raw_polylines` in place, and `vinyl_clear` throws
/// them away. Snapshotting this trio before each of those lets the operator take the change
/// back; keeping the transform out of it means undo never disturbs pan, zoom or placement.
#[derive(Clone)]
pub(crate) struct DesignSnapshot {
    polylines: Vec<Vec<Vec2>>,
    colors: Vec<Option<String>>,
    bbox: (Vec2, Vec2),
}

pub(crate) struct VinylApp {
    pub(crate) profile: MachineProfile,
    pub(crate) profile_path: String,
    pub(crate) raw_polylines: Vec<Vec<Vec2>>,
    /// Parallel to `raw_polylines`: the stroke/fill colour each polyline was drawn with, or
    /// `None` when the source declares none. An SVG can hold several vinyl colours in one file;
    /// this is what lets `vinyl_job_add` split such a design into one job per colour. DXF has no
    /// colour attribute we read, so its polylines are all `None`.
    pub(crate) raw_colors: Vec<Option<String>>,
    pub(crate) raw_bbox: (Vec2, Vec2),
    // transform
    pub(crate) offset: Vec2,
    pub(crate) scale: f64,
    pub(crate) rotation: f64, // radians
    pub(crate) copies_x: u32,
    pub(crate) copies_y: u32,
    pub(crate) step_x: f64,
    pub(crate) step_y: f64,
    // roll
    pub(crate) roll_width_mm: f64,
    pub(crate) roll_length_mm: f64,
    // view
    pub(crate) zoom: f32,
    pub(crate) pan: egui::Vec2,
    pub(crate) needs_fit: bool,
    pub(crate) drag_design: bool,
    // output
    pub(crate) pen_up_only: bool,
    pub(crate) port_name: String,
    pub(crate) baud: u32,
    pub(crate) status: String,
    pub(crate) log: Vec<String>,
    pub(crate) pos_mm: Vec2,
    /// Where a motion command asked the carriage to go. Committed to `pos_mm` only when the
    /// send reports success. Updating the tracked position before the bytes reach the machine
    /// makes a dead port look like a moving cutter: the app says "now at X" while nothing left
    /// the cable. A failed send must leave `pos_mm` where it was.
    pub(crate) pending_pos: Option<Vec2>,
    pub(crate) job_rx: Option<Receiver<JobMsg>>,
    /// Serial ports the OS reports, refreshed on demand. A hardcoded port name is a
    /// guess: Windows reassigns COM numbers when devices are replugged, so the port
    /// that worked yesterday can be a different board today.
    pub(crate) ports: Vec<PortChoice>,
    /// Requests from the control socket. `None` in headless mode, where no GUI exists to
    /// serve them.
    pub(crate) ctrl_rx: Option<Receiver<CtrlMsg>>,
    /// Port the control socket bound to (0 when it is not running).
    pub(crate) ctrl_port: u16,
    /// Pause/cancel gate shared with the thread that is writing the current job. `pause`
    /// holds the stream where it is; `cancel` abandons the rest of the job.
    pub(crate) send_ctl: Arc<SendControl>,
    /// Registration marks to cut around the design. `None` leaves the design alone.
    pub(crate) marks: Option<solidcraft_cam::marks::MarkConfig>,
    /// Weed-box margin in mm. `Some(m)` cuts a rectangle `m` clear of the design so the
    /// waste peels in one piece; `None` cuts no box.
    pub(crate) weed_box_mm: Option<f64>,
    /// Lift-mark size in mm. `Some(s)` puts a short grab tab at each closed contour;
    /// `None` leaves contours as drawn.
    pub(crate) lift_marks_mm: Option<f64>,
    /// Tiling options. `Some` splits the job into tiles and emits one command stream per
    /// tile; `None` emits the whole design as one job.
    pub(crate) tile_opts: Option<solidcraft_cam::tile::TileOptions>,
    /// Jobs queued for nesting. Each entry is one design already transformed into design
    /// space. Empty until `vinyl_job_add` is used; `vinyl_nest` packs the whole queue onto
    /// the roll in one pass.
    pub(crate) jobs: Vec<Job>,
    /// Snapshots taken before each destructive design edit, newest last.
    pub(crate) undo_stack: Vec<DesignSnapshot>,
    /// Snapshots undone, newest last. Cleared by any new edit.
    pub(crate) redo_stack: Vec<DesignSnapshot>,
}

impl Default for VinylApp {
    fn default() -> Self {
        // Prefer a port whose USB descriptor identifies a cutter bridge. Windows reassigns
        // COM numbers when devices are replugged, so a bare "COM3" is a guess; the USB VID/PID
        // is the identity.
        //
        // The USB descriptor is not the whole story though. Some cutters ship a controller
        // whose VID/PID is the chip vendor's default — an STM32 running native USB CDC reports
        // 0483:5740 and calls itself a Virtual COM Port, which looks exactly like a dev board.
        // Excluding that descriptor outright once made this app ignore the only port the
        // machine was actually on. So: prefer a known bridge, but when no bridge is present
        // and exactly one port exists, use that port. A port that is really there beats a
        // remembered port name whose device is a phantom node.
        const CUTTER_FALLBACK_PORT: &str = "COM3";
        let ports = scan_ports();
        let cutter = ports.iter().find(|p| p.looks_like_cutter());
        let only_port = (ports.len() == 1).then(|| &ports[0]);
        let chosen = cutter.or(only_port);
        let port_name = chosen
            .map(|p| p.name.clone())
            .unwrap_or_else(|| CUTTER_FALLBACK_PORT.into());
        // Say which case we are in up front. A silent "Ready" on a port that cannot cut is
        // how a user spends an afternoon sending jobs to a microcontroller.
        let status = match (&cutter, &only_port, ports.len()) {
            (Some(p), _, _) => format!(
                "Ready. Cutter bridge on {} ({}) selected; the blade will drop when you send a design.",
                p.name, p.label
            ),
            (None, Some(p), _) => format!(
                "Ready. Only one serial port present, so using {} ({}) — verify it is the \
                 cutter before sending a real job.",
                p.name, p.label
            ),
            (None, None, 0) => format!(
                "Ready. No serial ports found — the cutter is not plugged in. \
                 Defaulting to {CUTTER_FALLBACK_PORT}; connect it and refresh."
            ),
            (None, None, _) => format!(
                "Ready. No cutter bridge found; the ports present are not vinyl cutters. \
                 Defaulting to {CUTTER_FALLBACK_PORT} — connect the cutter and refresh."
            ),
        };
        // One source of truth: the shipped JSON profile. This block used to repeat the
        // machine's settings as a struct literal while the same settings also lived in
        // profiles/uscutter_sc2.json. Two copies of one machine drift apart — a blade
        // offset or travel limit added to the JSON never reached a default launch, so
        // running without --profile silently produced a weaker configuration than running
        // with it. Parsing the embedded text keeps the default and the file identical.
        //
        // The parse cannot fail: the cam crate asserts this exact text deserializes. A
        // corrupt build should still start with a usable profile rather than panic on
        // launch, so a failure degrades to the library default.
        let mut profile: MachineProfile =
            serde_json::from_str(solidcraft_cam::USCUTTER_SC2_PROFILE_JSON)
                .unwrap_or_else(|_| MachineProfile::default());
        // The port is the one setting the profile file cannot know, because it is
        // discovered per machine and Windows reassigns COM numbers on replug.
        profile.serial = Some(SerialSettings {
            port: port_name.clone(),
            baud: 57600,
            flow: CamFlow::RtsCts,
        });
        Self {
            profile,
            profile_path: String::new(),
            raw_polylines: Vec::new(),
            raw_colors: Vec::new(),
            raw_bbox: (Vec2::ZERO, Vec2::ZERO),
            offset: Vec2::ZERO,
            scale: 1.0,
            rotation: 0.0,
            copies_x: 1,
            copies_y: 1,
            step_x: 0.0,
            step_y: 0.0,
            roll_width_mm: 610.0,
            roll_length_mm: 2000.0,
            zoom: 0.5,
            pan: egui::Vec2::ZERO,
            needs_fit: true,
            drag_design: false,
            // A cutter's job is to cut. Defaulting to a pen-up dry run means the first
            // "Send design" produces travel and no cut, which reads as a dead machine.
            // The dry run stays one click away for tracing a design without vinyl loaded.
            pen_up_only: false,
            port_name,
            baud: 57600,
            status,
            log: Vec::new(),
            pos_mm: Vec2::ZERO,
            pending_pos: None,
            job_rx: None,
            ports,
            ctrl_rx: None,
            ctrl_port: 0,
            send_ctl: Arc::new(SendControl::default()),
            // Finishing features default to off: a plain design cuts as
            // drawn until the operator asks for marks, a weed box, or tiling.
            marks: None,
            weed_box_mm: None,
            lift_marks_mm: None,
            tile_opts: None,
            jobs: Vec::new(),
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
        }
    }
}

impl VinylApp {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Rescan the OS's serial port list. There is no reliable push notification when a USB
    /// device appears or disappears, so this runs on demand.
    pub(crate) fn refresh_ports(&mut self) {
        self.ports = scan_ports();
        if self.ports.is_empty() {
            self.log("No serial ports found — is the cutter plugged in?");
            return;
        }
        // Name the devices, not just the ports. "COM3" tells you nothing; "USB-SERIAL CH340
        // [1A86:7523]" tells you whether the cutter is actually there.
        let found: Vec<String> = self.ports.iter().map(|p| p.display()).collect();
        self.log(format!("Found {} port(s): {}", found.len(), found.join("; ")));
        let cutters: Vec<&PortChoice> = self.ports.iter().filter(|p| p.looks_like_cutter()).collect();
        if cutters.is_empty() {
            self.log(
                "None of these looks like a cutter (expected a CH340/FTDI/Prolific/CP210x bridge). \
                 If the cutter is plugged in but absent, its USB device has not enumerated — \
                 check the cable and try another USB port.",
            );
        } else if cutters.len() == 1 && self.port_name != cutters[0].name {
            // Follow the hardware when there is exactly one cutter and it moved to a new name.
            let name = cutters[0].name.clone();
            self.log(format!(
                "The only cutter-like device is on {name}; selecting it instead of {}",
                self.port_name
            ));
            self.port_name = name;
        }
    }

    /// Open the port and listen without sending a job. A reply identifies the device; a
    /// silence means either a write-only machine or nothing listening at all — the two
    /// cases "Sent N bytes" cannot tell apart.
    pub(crate) fn probe_port(&mut self) {
        let port = self.port_name.clone();
        if port.trim().is_empty() {
            self.log("No port selected to probe");
            return;
        }
        match write_serial(&port, self.baud, b"") {
            Ok((_, reply)) => {
                let reply = reply.trim();
                if reply.is_empty() {
                    self.log(format!("{port} opened, but the device said nothing"));
                } else {
                    self.log(format!("{port} replied: {reply}"));
                }
            }
            Err(e) => self.log(format!("{port} probe failed: {e}")),
        }
    }

    pub(crate) fn log(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        self.status = msg.clone();
        self.log.push(msg);
        if self.log.len() > 12 {
            self.log.remove(0);
        }
    }

    pub(crate) fn load_profile(&mut self, path: &Path) {
        match fs::read_to_string(path).map_err(|e| e.to_string()).and_then(|s| {
            serde_json::from_str::<MachineProfile>(&s).map_err(|e| e.to_string())
        }) {
            Ok(p) => {
                // A saved profile's port goes stale as soon as Windows renumbers the device,
                // and the shipped SC2 profile names COM3. Follow the hardware when a cutter
                // bridge is actually present; keep the profile's port only as a fallback.
                let scanned = scan_ports();
                let detected = scanned.iter().find(|c| c.looks_like_cutter()).cloned();
                let mut note = String::new();
                if let Some(s) = &p.serial {
                    match &detected {
                        Some(c) if c.name != s.port => {
                            note = format!(
                                "profile port {} is stale; using the detected cutter bridge {} ({})",
                                s.port, c.name, c.label
                            );
                            self.port_name = c.name.clone();
                        }
                        _ => self.port_name = s.port.clone(),
                    }
                    self.baud = s.baud;
                } else if let Some(c) = &detected {
                    self.port_name = c.name.clone();
                }
                self.ports = scanned;
                self.profile = p;
                self.profile_path = path.display().to_string();
                let msg = if note.is_empty() {
                    format!("Loaded profile {}", path.display())
                } else {
                    format!("Loaded profile {} ({note})", path.display())
                };
                self.log(msg);
            }
            Err(e) => self.log(format!("Profile error: {e}")),
        }
    }

    /// The transformed design as an SVG document, in millimetres.
    ///
    /// Import is one-way today: a design can be loaded and cut, but never saved. This writes
    /// the geometry the cutter would follow, so an edit can be exported and reopened rather
    /// than rebuilt. Curves are already flattened by the time they reach `raw_polylines`, so
    /// the output is polylines — accurate to the cut, not to the original beziers.
    ///
    /// Each contour is one `<polyline>`; the stroke is the colour it was drawn with, so a
    /// multi-colour design keeps its grouping when it is read back.
    pub(crate) fn export_svg(&self) -> String {
        let polys = self.transformed();
        let colors = self.transformed_colors();
        let (min, max) = bbox_of(&polys);
        let (w, h) = ((max.x - min.x).max(0.1), (max.y - min.y).max(0.1));
        let mut out = String::new();
        out.push_str(&format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w:.3}mm\" height=\"{h:.3}mm\" \
             viewBox=\"{:.3} {:.3} {w:.3} {h:.3}\">\n",
            min.x, min.y
        ));
        out.push_str("  <g fill=\"none\" stroke-width=\"0.25\">\n");
        for (i, poly) in polys.iter().enumerate() {
            if poly.len() < 2 {
                continue;
            }
            let color = colors
                .get(i)
                .and_then(|c| c.as_deref())
                .unwrap_or("#000000");
            out.push_str(&format!("    <polyline stroke=\"{color}\" points=\""));
            for (k, p) in poly.iter().enumerate() {
                if k > 0 {
                    out.push(' ');
                }
                out.push_str(&format!("{:.3},{:.3}", p.x, p.y));
            }
            out.push_str("\"/>\n");
        }
        out.push_str("  </g>\n</svg>\n");
        out
    }

    /// How many edits can be taken back.
    const UNDO_DEPTH: usize = 32;

    fn snapshot(&self) -> DesignSnapshot {
        DesignSnapshot {
            polylines: self.raw_polylines.clone(),
            colors: self.raw_colors.clone(),
            bbox: self.raw_bbox,
        }
    }

    fn restore(&mut self, s: DesignSnapshot) {
        self.raw_polylines = s.polylines;
        self.raw_colors = s.colors;
        self.raw_bbox = s.bbox;
        self.needs_fit = true;
    }

    /// Record the design as it stands, before an operation replaces it.
    ///
    /// Call this at the top of every method that rewrites `raw_polylines`; the snapshot is
    /// what `undo` restores.
    pub(crate) fn push_undo(&mut self) {
        self.undo_stack.push(self.snapshot());
        if self.undo_stack.len() > Self::UNDO_DEPTH {
            self.undo_stack.remove(0);
        }
        self.redo_stack.clear();
    }

    /// Take back the last design edit. False when there is nothing to undo.
    pub(crate) fn undo(&mut self) -> bool {
        let Some(prev) = self.undo_stack.pop() else {
            return false;
        };
        let now = self.snapshot();
        self.redo_stack.push(now);
        self.restore(prev);
        self.log("Undo");
        true
    }

    /// Reapply the last undone edit. False when there is nothing to redo.
    pub(crate) fn redo(&mut self) -> bool {
        let Some(next) = self.redo_stack.pop() else {
            return false;
        };
        let now = self.snapshot();
        self.undo_stack.push(now);
        self.restore(next);
        self.log("Redo");
        true
    }

    pub(crate) fn load_design(&mut self, path: &Path) {
        let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
        // SVG carries stroke/fill colour; DXF does not. Reading the coloured form for SVG is
        // what lets a multi-colour design split into one cutting job per colour later.
        let result = fs::read_to_string(path)
            .map_err(|e| e.to_string())
            .and_then(|text| match ext.as_str() {
                "svg" => read_svg_colored(&text)
                    .map_err(|e| e.to_string()),
                "dxf" => read_dxf(&text)
                    .map_err(|e| e.to_string())
                    .map(|shapes| {
                        // DXF carries no stroke/fill attribute we read, so every entity is
                        // unlabelled. The parallel colour vector must still match in length.
                        let n = shapes.len();
                        (shapes, vec![None; n])
                    }),
                _ => Err(format!("Unsupported design type .{ext} (use SVG or DXF)")),
            });
        match result {
            Ok((shapes, colors)) => {
                let (mut polys, poly_colors) = flatten_all_colored(&shapes, &colors);
                // SVG's Y axis points down and importers negate it, so a design often lands at
                // negative Y: below the roll origin and off the material. Shift it up/right onto
                // the roll. Positive placements (a pre-arranged multi-sticker layout) are kept.
                let (min, _max) = bbox_of(&polys);
                let shift = Vec2::new((-min.x).max(0.0), (-min.y).max(0.0));
                if shift.x > 0.0 || shift.y > 0.0 {
                    for poly in &mut polys {
                        for p in poly.iter_mut() {
                            p.x += shift.x;
                            p.y += shift.y;
                        }
                    }
                }
                // The file parsed: only now is the previous design about to be replaced,
                // so only now is there anything to take back.
                self.push_undo();
                self.raw_polylines = polys;
                // The colour vector stays parallel to the geometry so `vinyl_job_add` can split
                // a multi-colour design into one job per colour.
                self.raw_colors = poly_colors;
                self.raw_bbox = bbox_of(&self.raw_polylines);
                self.offset = Vec2::ZERO;
                self.scale = 1.0;
                self.rotation = 0.0;
                self.copies_x = 1;
                self.copies_y = 1;
                self.step_x = 0.0;
                self.step_y = 0.0;
                self.needs_fit = true;
                let n = self.raw_polylines.len();
                let b = self.raw_bbox;
                if shift.x > 0.0 || shift.y > 0.0 {
                    self.log(format!(
                        "Loaded {} ({n} paths); shifted {:.1},{:.1} mm onto the roll — now {:.1} × {:.1} mm",
                        path.display(),
                        shift.x,
                        shift.y,
                        b.1.x - b.0.x,
                        b.1.y - b.0.y
                    ));
                } else {
                    self.log(format!("Loaded {} ({n} paths)", path.display()));
                }
            }
            Err(e) => {
                // In a headless `--load ... --emit` run the log has no window to show, so a
                // load failure surfaced only as "nothing to cut" with the real reason lost.
                eprintln!("Design error: {e}");
                self.log(format!("Design error: {e}"));
            }
        }
    }

    pub(crate) fn transformed(&self) -> Vec<Vec<Vec2>> {
        if self.raw_polylines.is_empty() {
            return Vec::new();
        }
        let center = (self.raw_bbox.0 + self.raw_bbox.1) * 0.5;
        let (s, c) = self.rotation.sin_cos();
        let mut out = Vec::new();
        for ix in 0..self.copies_x.max(1) {
            for iy in 0..self.copies_y.max(1) {
                for poly in &self.raw_polylines {
                    let mut v = Vec::with_capacity(poly.len());
                    for p in poly {
                        let x = (p.x - center.x) * self.scale;
                        let y = (p.y - center.y) * self.scale;
                        let rx = x * c - y * s;
                        let ry = x * s + y * c;
                        v.push(Vec2::new(
                            rx + center.x + self.offset.x + ix as f64 * self.step_x,
                            ry + center.y + self.offset.y + iy as f64 * self.step_y,
                        ));
                    }
                    out.push(v);
                }
            }
        }
        out
    }

    /// Colour of each polyline returned by [`Self::transformed`], in the same order.
    ///
    /// `transformed` replicates the design for the copy grid, so this mirrors its loop exactly:
    /// the colour of each copy is the colour of the source polyline. Kept separate rather than
    /// returning pairs so callers that only need geometry (emit, bbox, preview) stay unchanged.
    pub(crate) fn transformed_colors(&self) -> Vec<Option<String>> {
        if self.raw_polylines.is_empty() {
            return Vec::new();
        }
        let mut out = Vec::with_capacity(
            self.raw_polylines.len() * self.copies_x.max(1) as usize * self.copies_y.max(1) as usize,
        );
        for _ix in 0..self.copies_x.max(1) {
            for _iy in 0..self.copies_y.max(1) {
                for i in 0..self.raw_polylines.len() {
                    out.push(self.raw_colors.get(i).cloned().flatten());
                }
            }
        }
        out
    }

    /// Add text to the design as cuttable contours — "type your lettering".
    ///
    /// The string is resolved to glyph outlines by the sketch font engine (the same engine that
    /// outlines imported SVG text), then flattened by the same code that handles imported
    /// geometry, so typed text cuts exactly like traced artwork. Each glyph's outline and its
    /// counters (the holes of A, B, 8) become separate closed contours, which is what the blade
    /// needs to cut a letter and its hole in one pass.
    ///
    /// Placement is in design space: `at` is the baseline's left end, `height_mm` the capital
    /// height and `angle_rad` the rotation about `at`, matching `text_geometry`.
    ///
    /// On a blank canvas the view transform is reset, so text lands exactly where it was typed
    /// instead of wherever a stale offset or scale would have thrown it. A design that already
    /// has content keeps its transform, because silently moving someone's artwork is worse than
    /// making them reposition the text they just added.
    ///
    /// Returns `(contours, segments)`: the number of closed contours added to the design and the
    /// number of raw outline segments the font engine produced. A glyph's outline arrives as a
    /// run of segments that get chained into one contour, so `segments` is normally much larger
    /// than `contours`. A space produces segments but no contours, so the pair distinguishes
    /// "typed nothing cuttable" from "typed nothing at all".
    pub(crate) fn add_text(
        &mut self,
        text: &str,
        at: Vec2,
        height_mm: f64,
        angle_rad: f64,
    ) -> Result<(usize, usize), String> {
        if text.trim().is_empty() {
            return Err("no text to add".into());
        }
        if !(height_mm.is_finite() && height_mm > 0.0) {
            return Err("height_mm must be a positive finite number".into());
        }
        if !(angle_rad.is_finite() && at.is_finite()) {
            return Err("the text's place and angle must be finite".into());
        }
        let outlines = text_geometry(text, at, height_mm, angle_rad)
            .map_err(|e| format!("could not outline text: {e}"))?;
        let segments = outlines.len();
        // Chain the segments into closed contours. `text_geometry` hands back one `LinkGeom` per
        // outline segment — a glyph's outline is a run of lines and curves — so flattening each
        // in isolation would cut every segment as its own path and the blade would lift and drop
        // at every corner. `flatten_all` joins segments whose endpoints meet and flushes each run
        // as soon as it closes, which is exactly the glyph-contour grouping a cutter needs.
        let geoms: Vec<Geom2> = outlines.iter().map(link_to_geom2).collect();
        let mut polys = flatten_all(&geoms);
        // A spline flattened to a single point, or a glyph the font engine reduced to a dot,
        // cannot be cut. Dropping these keeps the contour count honest.
        polys.retain(|p| p.len() >= 2);
        if polys.is_empty() {
            return Err("text produced no cuttable contours".into());
        }
        // Everything that can fail has passed and the design is about to change.
        self.push_undo();
        let was_empty = self.raw_polylines.is_empty();
        let n = polys.len();
        self.raw_polylines.extend(polys);
        // Text arrives with no colour of its own; the operator assigns one per job later. Extend
        // rather than resize so a colour vector that is already longer is never truncated.
        if self.raw_colors.len() < self.raw_polylines.len() {
            self.raw_colors.resize(self.raw_polylines.len(), None);
        }
        self.raw_bbox = bbox_of(&self.raw_polylines);
        if was_empty {
            // Typing on a blank canvas should land where it was typed.
            self.offset = Vec2::ZERO;
            self.scale = 1.0;
            self.rotation = 0.0;
            self.copies_x = 1;
            self.copies_y = 1;
            self.step_x = 0.0;
            self.step_y = 0.0;
        }
        self.needs_fit = true;
        self.log(format!(
            "Added text {text:?}: {n} contours from {segments} outline segments at {:.1},{:.1} mm",
            at.x, at.y
        ));
        Ok((n, segments))
    }

    /// Offset every closed contour in the loaded design: the geometry-level "outline"
    /// operation, not the view transform.
    ///
    /// Positive `distance` grows each contour outward — the classic sticker border, a cut line
    /// running `distance` millimetres outside the artwork. Negative shrinks it inward for an
    /// inline contour. Corners are rounded, matching the usual outline behaviour.
    ///
    /// An open polyline has no interior to offset against, so it is left untouched and counted
    /// in the return value. A contour that collapses under an inward offset (an inward distance
    /// larger than the shape can absorb) is dropped, also counted. The three counts are
    /// `(offset_contours, skipped_open, dropped_collapsed)`.
    ///
    /// With `keep_original` the offset contours are appended after the originals, so a single
    /// cut pass produces both the design and its border. Otherwise the geometry is replaced.
    /// `raw_colors` is rebuilt in lockstep so it stays parallel to `raw_polylines`, which is the
    /// invariant `vinyl_job_add` relies on when it splits a design into one job per colour.
    ///
    /// The distance is in design-space millimetres, measured before the view transform, so a
    /// design scaled up by 2 would have its outline doubled at emit time — the same as scaling
    /// any other geometry in the design.
    pub(crate) fn offset_design(&mut self, distance: f64, keep_original: bool) -> (usize, usize, usize) {
        if self.raw_polylines.is_empty() {
            return (0, 0, 0);
        }
        self.push_undo();
        let mut offset_count = 0usize;
        let mut skipped_open = 0usize;
        let mut dropped = 0usize;

        let old_polys = std::mem::take(&mut self.raw_polylines);
        let old_colors = std::mem::take(&mut self.raw_colors);
        let mut new_polys: Vec<Vec<Vec2>> = Vec::with_capacity(old_polys.len());
        let mut new_colors: Vec<Option<String>> = Vec::with_capacity(old_colors.len());
        // Kept offsets are collected here and spliced in after the main pass, so the design's
        // own contours keep their original order and the border contours land at the end.
        let mut kept_polys: Vec<Vec<Vec2>> = Vec::new();
        let mut kept_colors: Vec<Option<String>> = Vec::new();

        for (i, poly) in old_polys.iter().enumerate() {
            let color = old_colors.get(i).cloned().flatten();
            // Same closure test `polylines_to_toolpaths` uses, so "closed" means the same
            // thing to the offset as it does to the cutter.
            let closed = poly.len() > 2 && (poly[0] - poly[poly.len() - 1]).len() < 1e-6;
            if !closed {
                skipped_open += 1;
                new_polys.push(poly.clone());
                new_colors.push(color);
                continue;
            }
            // One polyline is one contour, so the toolpath slice holds a single entry.
            let src = polylines_to_toolpaths(std::slice::from_ref(poly));
            let Some(tp) = src.first() else {
                new_polys.push(poly.clone());
                new_colors.push(color);
                continue;
            };
            match solidcraft_cam::offset_closed(tp, distance).first() {
                Some(result) => {
                    offset_count += 1;
                    if keep_original {
                        new_polys.push(poly.clone());
                        new_colors.push(color.clone());
                        kept_polys.push(toolpath_to_polyline(result));
                        kept_colors.push(color);
                    } else {
                        new_polys.push(toolpath_to_polyline(result));
                        new_colors.push(color);
                    }
                }
                None => {
                    // The contour collapsed. Keeping the original would cut a shape the
                    // operator asked to remove, so drop it and report the count.
                    dropped += 1;
                }
            }
        }

        new_polys.extend(kept_polys);
        new_colors.extend(kept_colors);
        self.raw_polylines = new_polys;
        self.raw_colors = new_colors;
        self.raw_bbox = bbox_of(&self.raw_polylines);
        self.needs_fit = true;
        let mut note = format!("Offset {distance:.2} mm: {offset_count} contour(s)");
        if skipped_open > 0 {
            note.push_str(&format!(", {skipped_open} open path(s) left as-is"));
        }
        if dropped > 0 {
            note.push_str(&format!(", {dropped} collapsed and removed"));
        }
        self.log(note);
        (offset_count, skipped_open, dropped)
    }

    /// Apply a boolean operation to the design's filled regions: Weld, Trim, Punch,
    /// Intersect and Exclude tools.
    ///
    /// The design is grouped by colour and each group is operated on separately. That is
    /// deliberate: a cutter loads one vinyl colour at a time, so welding a red shape into a blue
    /// one would produce geometry no single cut pass could make. Overlapping shapes of different
    /// colours therefore stay separate, which is what the material requires — the same reason
    /// `vinyl_nest` packs per colour. Colour names are processed in sorted order so a repeated run
    /// on the same design produces the same output ordering.
    ///
    /// Without a `clip` rectangle, [`BooleanOp::Union`] welds each colour group against itself —
    /// merge overlapping contours and resolve self-intersections. That is the usual cleanup before
    /// cutting traced artwork. With a clip `(x, y, w, h)` in design-space millimetres, the
    /// rectangle is the second operand: difference punches it out, intersect trims to it, xor
    /// excludes it. A clip with union merges the rectangle into the design, which is rarely wanted
    /// but is not an error.
    ///
    /// Open polylines have no interior to fill, so they pass through untouched rather than being
    /// dropped — silently deleting part of the design the operator may still want to cut would be
    /// worse than leaving it. Degenerate closed contours that enclose no area are dropped and
    /// counted, matching how the offset tool reports collapses.
    ///
    /// Returns `(contours_out, passed_through_open, skipped_degenerate)`. `contours_out` counts
    /// only the contours the boolean produced; the passed-through open paths are extra geometry in
    /// the design and are reported separately so the count stays honest.
    pub(crate) fn boolean_design(
        &mut self,
        op: BooleanOp,
        fill: FillMode,
        clip: Option<(f64, f64, f64, f64)>,
    ) -> (usize, usize, usize) {
        if self.raw_polylines.is_empty() {
            return (0, 0, 0);
        }
        self.push_undo();

        // The clip rectangle is the same second operand for every colour group. Built once because
        // a rectangle's construction is trivial but rebuilding it per group would invite the
        // groups to drift apart.
        let clip_paths: Vec<Toolpath> = clip
            .map(|(x, y, w, h)| vec![Toolpath::rectangle(x, y, w, h)])
            .unwrap_or_default();

        // Group polyline indices by colour. `raw_colors` may be shorter than `raw_polylines` — the
        // invariant elsewhere is to extend with `None` — so read through `get` and flatten rather
        // than indexing, which would panic on a short colour vector.
        let mut groups: BTreeMap<Option<String>, Vec<usize>> = BTreeMap::new();
        for i in 0..self.raw_polylines.len() {
            let color = self.raw_colors.get(i).cloned().flatten();
            groups.entry(color).or_default().push(i);
        }

        let old_polys = std::mem::take(&mut self.raw_polylines);
        let mut new_polys: Vec<Vec<Vec2>> = Vec::new();
        let mut new_colors: Vec<Option<String>> = Vec::new();
        let mut passed_open = 0usize;
        let mut skipped = 0usize;
        let mut out_count = 0usize;

        for (color, indices) in &groups {
            let mut closed: Vec<Toolpath> = Vec::new();
            for &i in indices {
                let poly = &old_polys[i];
                // Same closure test the offset tool and `polylines_to_toolpaths` use, so "closed"
                // means the same thing here as it does to the cutter.
                let is_closed = poly.len() > 2 && (poly[0] - poly[poly.len() - 1]).len() < 1e-6;
                if !is_closed {
                    passed_open += 1;
                    new_polys.push(poly.clone());
                    new_colors.push(color.clone());
                    continue;
                }
                match polylines_to_toolpaths(std::slice::from_ref(poly))
                    .into_iter()
                    .next()
                {
                    Some(tp) => closed.push(tp),
                    None => skipped += 1,
                }
            }
            if closed.is_empty() {
                continue;
            }
            let results = boolean_paths(&closed, &clip_paths, op, fill);
            // An empty result means the operation consumed the whole group — a legitimate outcome
            // for a difference that removes everything. Nothing to append, and the group's colour
            // disappears with it.
            for tp in results {
                out_count += 1;
                new_polys.push(toolpath_to_polyline(&tp));
                new_colors.push(color.clone());
            }
        }

        let in_count = old_polys.len();
        self.raw_polylines = new_polys;
        self.raw_colors = new_colors;
        self.raw_bbox = bbox_of(&self.raw_polylines);
        self.needs_fit = true;
        let mut note = format!(
            "Boolean {op:?}: {out_count} contour(s) from {in_count} input path(s)"
        );
        if clip.is_some() {
            note.push_str(", clip rectangle applied");
        }
        if passed_open > 0 {
            note.push_str(&format!(", {passed_open} open path(s) left as-is"));
        }
        if skipped > 0 {
            note.push_str(&format!(", {skipped} degenerate contour(s) dropped"));
        }
        self.log(note);
        (out_count, passed_open, skipped)
    }

    /// How far the design has to slide to sit on the material.
    ///
    /// A cutter's origin is the material's leading edge, so a design at negative
    /// coordinates (or past the far edge) is physically impossible — the head would
    /// drive off the vinyl. We slide it back on instead of refusing, because "move it
    /// around" should never dead-end; the shift is reported so the user can see it.
    /// Returns `None` when the design cannot fit the roll at all.
    pub(crate) fn placement_shift(&self) -> Option<Vec2> {
        let b = bbox_of(&self.transformed());
        let w = b.1.x - b.0.x;
        let h = b.1.y - b.0.y;
        if w > self.roll_width_mm + 1e-6 || h > self.roll_length_mm + 1e-6 {
            return None;
        }
        let x = if b.0.x < -1e-9 {
            -b.0.x
        } else if b.1.x > self.roll_width_mm + 1e-9 {
            self.roll_width_mm - b.1.x
        } else {
            0.0
        };
        let y = if b.0.y < -1e-9 {
            -b.0.y
        } else if b.1.y > self.roll_length_mm + 1e-9 {
            self.roll_length_mm - b.1.y
        } else {
            0.0
        };
        Some(Vec2::new(x, y))
    }

    /// How far finishing geometry reaches past the design's own bounding box. The roll-fit
    /// check must reserve this much room or a mark lands past the vinyl edge even though the
    /// design itself fits.
    pub(crate) fn finishing_margin(&self) -> f64 {
        let mut m: f64 = 0.0;
        if let Some(cfg) = &self.marks {
            // A corner L reaches its arm length past the anchor; a cross or circle reaches
            // half its size. The margin is measured from the design edge to the anchor, so
            // the total reach is margin plus the mark's own extent.
            let reach = match cfg.style {
                solidcraft_cam::marks::MarkStyle::CornerL { size_mm } => size_mm,
                solidcraft_cam::marks::MarkStyle::Cross { size_mm } => size_mm * 0.5,
                solidcraft_cam::marks::MarkStyle::CircleCross { diameter_mm, .. } => {
                    diameter_mm * 0.5
                }
            };
            m = m.max(cfg.margin_mm + reach);
        }
        if let Some(w) = self.weed_box_mm {
            m = m.max(w);
        }
        if let Some(s) = self.lift_marks_mm {
            m = m.max(s);
        }
        m
    }

    /// Build the finishing geometry (marks, weed box, lift tabs) around a placed design.
    ///
    /// Generated *after* the design is shifted onto the roll, so the marks follow the design
    /// instead of staying at its pre-shift position. The returned toolpaths are not run
    /// through the profile's path filters: a registration mark is a reference shape, and
    /// blade-offset or min-line would distort it rather than clean it.
    pub(crate) fn finishing_geometry(&self, placed_bbox: (Vec2, Vec2)) -> Vec<Toolpath> {
        let cam_bbox = (
            Point::new(placed_bbox.0.x, placed_bbox.0.y),
            Point::new(placed_bbox.1.x, placed_bbox.1.y),
        );
        let mut out: Vec<Toolpath> = Vec::new();
        if let Some(cfg) = &self.marks {
            out.extend(solidcraft_cam::marks::registration_marks(cam_bbox, cfg));
        }
        if let Some(margin) = self.weed_box_mm {
            out.push(solidcraft_cam::marks::weed_box(cam_bbox, margin));
        }
        if let Some(size) = self.lift_marks_mm {
            // Lift tabs attach to the design's own closed contours, not to the marks.
            let design = polylines_to_toolpaths(&self.transformed());
            out.extend(solidcraft_cam::marks::lift_marks(&design, size));
        }
        out
    }

    pub(crate) fn emit(&self, pen_up_only: bool) -> Result<String, String> {
        let polys = self.transformed();
        if polys.is_empty() {
            return Err("nothing to cut: load a design first".into());
        }
        let design_bbox = bbox_of(&polys);
        // Reserve room for finishing geometry before the fit check. A design that just fits
        // the roll still pushes a 5 mm registration mark past the edge if the mark is not
        // counted, so the inflated box is what has to fit.
        let margin = self.finishing_margin();
        let inflated = (
            Vec2::new(design_bbox.0.x - margin, design_bbox.0.y - margin),
            Vec2::new(design_bbox.1.x + margin, design_bbox.1.y + margin),
        );
        let w = inflated.1.x - inflated.0.x;
        let h = inflated.1.y - inflated.0.y;
        if w > self.roll_width_mm + 1e-6 || h > self.roll_length_mm + 1e-6 {
            return Err(format!(
                "design with finishing geometry is {:.1} × {:.1} mm but the roll is {:.1} × {:.1} mm: scale or rotate it to fit",
                w, h, self.roll_width_mm, self.roll_length_mm
            ));
        }
        // Shift the *inflated* box onto the roll. Shifting only the design would leave the
        // marks hanging off the edge when the design sits against it.
        let shift = Vec2::new(
            if inflated.0.x < -1e-9 {
                -inflated.0.x
            } else if inflated.1.x > self.roll_width_mm + 1e-9 {
                self.roll_width_mm - inflated.1.x
            } else {
                0.0
            },
            if inflated.0.y < -1e-9 {
                -inflated.0.y
            } else if inflated.1.y > self.roll_length_mm + 1e-9 {
                self.roll_length_mm - inflated.1.y
            } else {
                0.0
            },
        );
        let placed: Vec<Vec<Vec2>> = if shift.x == 0.0 && shift.y == 0.0 {
            polys
        } else {
            polys
                .into_iter()
                .map(|p| p.into_iter().map(|q| q + shift).collect())
                .collect()
        };
        let placed_bbox = bbox_of(&placed);
        // Design geometry goes through the path filters. Finishing geometry does not.
        let design = filters::apply(&self.profile, polylines_to_toolpaths(&placed));
        // Marks first: the operator registers the sheet to them, then the design cuts. Cutting
        // the design first would drag the blade across a mark and leave a nick in it.
        let mut all: Vec<Toolpath> = self.finishing_geometry(placed_bbox);
        all.extend(design);
        let pen = if pen_up_only {
            post::Pen::Up
        } else {
            post::Pen::Down
        };
        // Emit structurally: the pen state picks the command word, so a dry run never
        // depends on rewriting text that might contain a coordinate 'D'.
        post::emit_with_pen(&self.profile, &all, pen).map_err(|e| e.to_string())
    }

    /// Emit one command stream per tile.
    ///
    /// The whole-design roll-fit check does not apply here: the point of tiling is that the
    /// design is *bigger* than the roll. Instead each tile must fit on its own, and the
    /// operator reloads the material between tiles. Finishing geometry (marks, weed box) is
    /// attached to the whole design, so it is emitted only on the first tile — repeating a
    /// registration mark on every tile would cut the same mark into each sheet.
    pub(crate) fn emit_tiled(&self, pen_up_only: bool) -> Result<Vec<(solidcraft_cam::tile::Tile, String)>, String> {
        let polys = self.transformed();
        if polys.is_empty() {
            return Err("nothing to cut: load a design first".into());
        }
        let opts = match &self.tile_opts {
            Some(o) => o,
            None => return Err("tiling not configured".into()),
        };
        let design_bbox = bbox_of(&polys);
        let cam_bbox = (
            Point::new(design_bbox.0.x, design_bbox.0.y),
            Point::new(design_bbox.1.x, design_bbox.1.y),
        );
        let design = filters::apply(&self.profile, polylines_to_toolpaths(&polys));
        let tiles = solidcraft_cam::tile::tile(&design, cam_bbox, opts);
        if tiles.is_empty() {
            return Err("tiling produced no tiles — the design may be smaller than one tile".into());
        }
        let pen = if pen_up_only {
            post::Pen::Up
        } else {
            post::Pen::Down
        };
        let mut out = Vec::with_capacity(tiles.len());
        for (i, mut tile) in tiles.into_iter().enumerate() {
            // Tile geometry is in design coordinates. Rebasing to the tile's own origin lets
            // the operator cut each tile from the machine's origin instead of jogging to an
            // absolute position that only makes sense for the whole sheet.
            let origin = tile.origin;
            for tp in &mut tile.toolpaths {
                tp.translate(-origin.x, -origin.y);
            }
            let mut all: Vec<Toolpath> = if i == 0 {
                self.finishing_geometry(design_bbox)
            } else {
                Vec::new()
            };
            all.extend(tile.toolpaths.iter().cloned());
            let code = post::emit_with_pen(&self.profile, &all, pen).map_err(|e| e.to_string())?;
            out.push((tile, code));
        }
        Ok(out)
    }

    /// Pack every queued job onto the roll and emit one command stream.
    ///
    /// Unlike [`Self::emit`], the roll-*length* check does not reject an over-long batch:
    /// nesting exists precisely to consume however much roll the batch needs, and
    /// `used_length_mm` is reported so the operator knows the material cost. The roll *width*
    /// stays a hard limit — a job wider than the vinyl cannot be cut at all and comes back in
    /// `unplaced` rather than being scaled or clipped.
    ///
    /// Filters run per job, before packing. A blade offset or overcut is a property of the
    /// design's own contours; running filters after packing would be equivalent for a pure
    /// offset but would break as soon as a filter needs to know where a contour starts and
    /// ends, because packing leaves several contours in one run.
    ///
    /// Finishing geometry wraps the whole batch: registration marks go around the used area,
    /// not around each decal. Cutting them first lets the operator register the sheet before
    /// the blade starts on a design that would otherwise drag across a mark.
    pub(crate) fn emit_nested(
        &self,
        pen_up_only: bool,
        gap_mm: f64,
        allow_rotation: bool,
    ) -> Result<Vec<NestedGroup>, String> {
        if self.jobs.is_empty() {
            return Err("no jobs queued: add a design with vinyl_job_add first".into());
        }
        // Group job indices by colour, preserving first-seen order so the emitted runs come
        // out in the order the operator built the batch. Unlabelled jobs all fall into the
        // empty-colour group and nest together, which is the single-colour case.
        let mut colors: Vec<String> = Vec::new();
        let mut members: Vec<Vec<usize>> = Vec::new();
        for (i, job) in self.jobs.iter().enumerate() {
            match colors.iter().position(|c| c == &job.color) {
                Some(g) => members[g].push(i),
                None => {
                    colors.push(job.color.clone());
                    members.push(vec![i]);
                }
            }
        }

        let pen = if pen_up_only {
            post::Pen::Up
        } else {
            post::Pen::Down
        };
        let mut groups: Vec<NestedGroup> = Vec::with_capacity(colors.len());

        for (color, group) in colors.iter().zip(members.iter()) {
            // Filters run per job, before packing: a blade offset is a property of the
            // design's own contours, and packing compressed geometry would make neighbouring
            // parts overlap by the offset.
            let jobs: Vec<Vec<Toolpath>> = group
                .iter()
                .map(|&i| {
                    filters::apply(
                        &self.profile,
                        polylines_to_toolpaths(&self.jobs[i].polylines),
                    )
                })
                .collect();
            let opts = solidcraft_cam::nest::NestOptions {
                roll_width_mm: self.roll_width_mm,
                gap_mm: gap_mm.max(0.0),
                allow_rotation,
            };
            let result = solidcraft_cam::nest::nest(&jobs, &opts);

            // A group that fits nothing is reported rather than dropped: the operator needs
            // to know which colour failed. The whole call only fails when NOTHING fits.
            let mut placed: Vec<(String, Vec2, bool)> = Vec::with_capacity(result.placed.len());
            for p in &result.placed {
                let name = group
                    .get(p.index)
                    .map(|&g| self.jobs[g].name.clone())
                    .unwrap_or_default();
                placed.push((name, Vec2::new(p.origin.x, p.origin.y), p.rotated));
            }
            let mut unplaced: Vec<String> = Vec::with_capacity(result.unplaced.len());
            for &local in &result.unplaced {
                if let Some(&g) = group.get(local) {
                    unplaced.push(self.jobs[g].name.clone());
                }
            }

            // Finishing geometry wraps this group's used area. Cutting marks before the
            // design lets the operator register the sheet without the blade dragging over a
            // finished mark.
            let mut placed_paths: Vec<Toolpath> = Vec::new();
            for p in &result.placed {
                placed_paths.extend(p.toolpaths.iter().cloned());
            }
            let mut all: Vec<Toolpath> = Vec::new();
            if let Some(union) = solidcraft_cam::nest::bounds(&placed_paths) {
                if let Some(cfg) = &self.marks {
                    all.extend(solidcraft_cam::marks::registration_marks(union, cfg));
                }
                if let Some(margin) = self.weed_box_mm {
                    all.push(solidcraft_cam::marks::weed_box(union, margin));
                }
                if let Some(size) = self.lift_marks_mm {
                    all.extend(solidcraft_cam::marks::lift_marks(&placed_paths, size));
                }
            }
            all.extend(placed_paths);

            let code = if all.is_empty() {
                String::new()
            } else {
                post::emit_with_pen(&self.profile, &all, pen).map_err(|e| e.to_string())?
            };

            groups.push(NestedGroup {
                color: color.clone(),
                placed,
                unplaced,
                used_length_mm: result.used_length_mm,
                code,
            });
        }

        if groups.iter().all(|g| g.placed.is_empty()) {
            return Err(format!(
                "nothing fit the roll: no job is narrower than {:.1} mm in either orientation",
                self.roll_width_mm
            ));
        }

        Ok(groups)
    }

    pub(crate) fn send_code(&mut self, code: String, label: &str) {
        let port = self.port_name.clone();
        let baud = self.baud;
        let bytes = code.len();
        let tmp = std::env::temp_dir().join("vinyl_last.dmpl");
        let _ = fs::write(&tmp, &code);
        // Say out loud when the target is not a cutter. A wrong port still accepts bytes and
        // still reports "Sent N bytes", so silence here is what makes a dead machine look
        // like a working app. This does not block the send — a rare cutter may use an odd
        // bridge — but it must never be quiet about it.
        if let Some(p) = self.ports.iter().find(|p| p.name == port) {
            if !p.looks_like_cutter() {
                self.log(format!(
                    "WARNING: {port} is \"{}\", which is not a known cutter USB bridge. \
                     A cutter that isn't plugged in leaves its COM name free for the next \
                     device to claim — check the port before trusting a silent job.",
                    p.label
                ));
            }
        } else if !self.ports.is_empty() {
            self.log(format!(
                "WARNING: {port} is not in the scanned port list; it may be stale."
            ));
        }
        self.log(format!(
            "Sending {label} ({bytes} bytes) to {port} @ {baud} 8-N-1; copy at {}",
            tmp.display()
        ));
        // A new job starts unpaused and uncancelled. Leftover flags from an aborted job would
        // otherwise stall this one before its first byte reached the wire.
        self.send_ctl.pause.store(false, Ordering::Relaxed);
        self.send_ctl.cancel.store(false, Ordering::Relaxed);
        let ctl = Arc::clone(&self.send_ctl);
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = write_serial_ctl(&port, baud, code.as_bytes(), Some(&ctl));
            let _ = match result {
                Ok((n, reply)) => {
                    let reply = reply.trim();
                    // A silent device is reported as silence, not as success. "Sent N bytes"
                    // alone reads like the machine acted on them, and that false confidence is
                    // how a wrong COM port gets diagnosed as a broken cutter.
                    let msg = if reply.is_empty() {
                        format!("Sent {n} bytes to {port}; device did not reply")
                    } else {
                        format!("Sent {n} bytes to {port}; device replied: {reply}")
                    };
                    tx.send(JobMsg::Done(msg))
                }
                Err(e) => tx.send(JobMsg::Err(format!("Send failed: {e}"))),
            };
        });
        self.job_rx = Some(rx);
    }

    pub(crate) fn send_simple(&mut self, cmd: &str, label: &str) {
        self.send_code(cmd.to_string(), label);
    }

    pub(crate) fn poll_job(&mut self) {
        let mut done = false;
        let mut msgs = Vec::new();
        if let Some(rx) = &self.job_rx {
            while let Ok(msg) = rx.try_recv() {
                msgs.push(msg);
                done = true;
            }
        }
        for msg in msgs {
            match msg {
                JobMsg::Done(s) => {
                    self.log(s);
                    // The bytes reached the port. Only now does the commanded position become
                    // the tracked position — a "Sent N bytes" with no reply still means the
                    // write succeeded, which is the most the app can know without device ack.
                    if let Some(p) = self.pending_pos.take() {
                        self.pos_mm = p;
                    }
                }
                JobMsg::Err(s) => {
                    self.log(s);
                    // The move did not happen. Leaving pos_mm alone keeps the readout honest;
                    // updating it here is what made a wrong port look like a moving cutter.
                    if self.pending_pos.take().is_some() {
                        self.log("move failed; tracked position unchanged");
                    }
                }
            }
        }
        if done {
            self.job_rx = None;
        }
    }

    /// Start the local control socket. This is what lets a separate process — an MCP client,
    /// a script, another terminal — ask this GUI to report state or close, instead of the
    /// user having to hunt down the window.
    pub(crate) fn attach_control(&mut self, ctx: egui::Context) {
        let (tx, rx) = mpsc::channel();
        match spawn_control_server(tx, ctx) {
            Ok(port) => {
                self.ctrl_port = port;
                self.ctrl_rx = Some(rx);
                // Advertise the port so a client does not have to guess when the default was
                // taken and the listener landed somewhere else.
                let _ = fs::write(control_port_file(), port.to_string());
                self.log(format!(
                    "Control socket on 127.0.0.1:{port} (quit, state, tools/list, tools/call)"
                ));
            }
            Err(e) => self.log(format!("Control socket failed to start: {e}")),
        }
    }

    /// Drain control requests and act on them. Only the UI thread touches `self`, so the
    /// socket thread hands work over on a channel rather than calling in directly.
    pub(crate) fn poll_control(&mut self, ctx: &egui::Context) {
        if QUIT_REQUESTED.load(Ordering::SeqCst) {
            self.log("Closing on control-socket request");
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            return;
        }
        let Some(rx) = self.ctrl_rx.take() else {
            return;
        };
        let mut keep = true;
        loop {
            match rx.try_recv() {
                Ok(CtrlMsg::State(reply)) => {
                    let _ = reply.send(state_json(self));
                }
                Ok(CtrlMsg::Call { name, args, reply }) => {
                    let _ = reply.send(call_vinyl_tool(self, &name, &args));
                }
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => {
                    keep = false;
                    break;
                }
            }
        }
        if keep {
            self.ctrl_rx = Some(rx);
        }
    }

    pub(crate) fn send_design(&mut self) {
        match self.emit(self.pen_up_only) {
            Ok(code) => {
                let label = if self.pen_up_only { "pen-up design" } else { "design" };
                self.send_code(code, label);
            }
            Err(e) => self.log(e),
        }
    }

    pub(crate) fn send_test_square(&mut self) {
        // 10 mm square, pen up only, at 40 units/mm. Safe with no vinyl.
        self.send_simple("IN;PA;U0,0;U400,0;U400,400;U0,400;U0,0;@", "pen-up test square");
    }

    pub(crate) fn home(&mut self) {
        // Ask for the origin. The tracked position is committed only when the send reports
        // back; moving the readout first makes a dead port look like a moving carriage.
        self.pending_pos = Some(Vec2::ZERO);
        self.send_simple("IN;PA;U0,0;@", "home");
    }

    /// Travel limits for manual moves: the profile's declared machine limits when set, else
    /// the loaded roll. A jog is a manual move with no design to bound it, so without this a
    /// held-down arrow walks the carriage off the end of the rail.
    pub(crate) fn travel_limits(&self) -> (f64, f64) {
        (
            self.profile.max_x_mm.unwrap_or(self.roll_width_mm),
            self.profile.max_y_mm.unwrap_or(self.roll_length_mm),
        )
    }

    pub(crate) fn jog(&mut self, dx: f64, dy: f64) {
        let raw = self.pos_mm + Vec2::new(dx, dy);
        let (max_x, max_y) = self.travel_limits();
        let p = Vec2::new(raw.x.clamp(0.0, max_x), raw.y.clamp(0.0, max_y));
        if (p.x - raw.x).abs() > 1e-9 || (p.y - raw.y).abs() > 1e-9 {
            self.log(format!(
                "jog clamped to ({:.1}, {:.1}) mm — requested ({:.1}, {:.1}) is outside the \
                 0..{:.1} × 0..{:.1} mm travel limit",
                p.x, p.y, raw.x, raw.y, max_x, max_y
            ));
        }
        self.pending_pos = Some(p);
        let cmd = format!(
            "IN;PA;U{:.0},{:.0};@",
            p.x * self.profile.units_per_mm,
            p.y * self.profile.units_per_mm
        );
        self.send_simple(&cmd, "jog");
    }

    /// Send a pen-up move to where the carriage already is. In DMPL the `U` command carries
    /// both the pen state and the move, so a zero-length U lifts the blade without moving it.
    pub(crate) fn pen_up(&mut self) {
        let p = self.pos_mm;
        let cmd = format!(
            "IN;PA;U{:.0},{:.0};@",
            p.x * self.profile.units_per_mm,
            p.y * self.profile.units_per_mm
        );
        self.send_simple(&cmd, "pen up");
    }

    /// Send a pen-down move to where the carriage already is: a zero-length `D` drops the
    /// blade without cutting a path. Some firmware ignores a zero-length move, so the blade
    /// may not actuate; check visually before trusting it. The blade stays down until
    /// `pen_up` or `home`, which is why the tool reports that state back to the caller.
    pub(crate) fn pen_down(&mut self) {
        let p = self.pos_mm;
        let cmd = format!(
            "IN;PA;D{:.0},{:.0};@",
            p.x * self.profile.units_per_mm,
            p.y * self.profile.units_per_mm
        );
        self.send_simple(&cmd, "pen down");
    }

    /// Hold the job where it is. The write thread stops between 64-byte chunks; the cutter
    /// keeps going until its own buffer drains, then waits for more. Nothing is lost — the
    /// rest of the stream stays queued and continues on `resume`.
    pub(crate) fn pause_send(&mut self) {
        self.send_ctl.pause.store(true, Ordering::Relaxed);
        self.log("job paused — the cutter will stop once its buffer drains");
    }

    pub(crate) fn resume_send(&mut self) {
        self.send_ctl.pause.store(false, Ordering::Relaxed);
        self.log("job resumed");
    }

    /// Abandon the rest of the stream. The blade stays wherever the last completed command
    /// left it; call `pen_up` or `home` afterwards to lift it clear of the vinyl.
    pub(crate) fn cancel_send(&mut self) {
        self.send_ctl.cancel.store(true, Ordering::Relaxed);
        self.send_ctl.pause.store(false, Ordering::Relaxed);
        self.log("job cancelled — lift the blade with pen up or home before unloading");
    }

    pub(crate) fn send_paused(&self) -> bool {
        self.send_ctl.pause.load(Ordering::Relaxed)
    }

    /// A built-in sticker so the canvas is never empty and "move it around" can be tried at once.
    /// Three closed paths: a 300 × 150 mm rounded-rectangle outline and two concentric circles.
    pub(crate) fn load_demo(&mut self) {
        self.push_undo();
        self.raw_polylines = vec![
            rounded_rect_poly(300.0, 150.0, 20.0, 12),
            circle_poly(Vec2::new(150.0, 75.0), 45.0, 64),
            circle_poly(Vec2::new(150.0, 75.0), 12.0, 32),
        ];
        // The demo is single-colour: one unlabelled entry per polyline keeps the colour vector
        // parallel to the geometry.
        self.raw_colors = vec![None; self.raw_polylines.len()];
        self.raw_bbox = bbox_of(&self.raw_polylines);
        self.offset = Vec2::ZERO;
        self.scale = 1.0;
        self.rotation = 0.0;
        self.copies_x = 1;
        self.copies_y = 1;
        self.step_x = 0.0;
        self.step_y = 0.0;
        self.needs_fit = true;
        let b = self.raw_bbox;
        self.log(format!(
            "Loaded demo sticker: {} paths, {:.1} × {:.1} mm",
            self.raw_polylines.len(),
            b.1.x - b.0.x,
            b.1.y - b.0.y
        ));
    }
}

impl eframe::App for VinylApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.poll_job();
        // Control requests arrive from a socket thread, so they can only be acted on here,
        // on the thread that owns the window. A `quit` request closes the window instead of
        // killing the process, so buffered writes and the port handle get dropped cleanly.
        self.poll_control(ui.ctx());

        egui::containers::Panel::top("top").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("VinylCraft");
                ui.separator();
                ui.label("Profile:");
                ui.label(if self.profile_path.is_empty() { "SC2 default" } else { &self.profile_path });
                if ui.button("Load profile").clicked() {
                    if let Some(p) = rfd::FileDialog::new().add_filter("Profile", &["json"]).pick_file() {
                        self.load_profile(&p);
                    }
                }
                ui.separator();
                ui.label("Port:");
                ui.text_edit_singleline(&mut self.port_name);
                // Port numbers are assigned by the OS and move when devices are replugged.
                // Listing what actually exists turns "COM3" from a guess into a choice.
                if ui
                    .button("⟳")
                    .on_hover_text("Rescan serial ports")
                    .clicked()
                {
                    self.refresh_ports();
                }
                for p in self.ports.clone() {
                    let picked = p.name == self.port_name;
                    let mut text = p.display();
                    if p.looks_like_cutter() {
                        text.push_str("  ✔ cutter bridge");
                    } else {
                        text.push_str("  ⚠ not a cutter bridge");
                    }
                    if ui.selectable_label(picked, text).clicked() {
                        self.port_name = p.name;
                    }
                }
                if ui
                    .button("Probe")
                    .on_hover_text("Open the port and report what it answers")
                    .clicked()
                {
                    self.probe_port();
                }
                ui.label("Baud:");
                ui.add(egui::DragValue::new(&mut self.baud).speed(100.0).range(1200..=115200));
                ui.separator();
                ui.checkbox(&mut self.pen_up_only, "Pen-up dry run (blade stays up — nothing is cut)")
                    .on_hover_text(
                        "Check this to trace the design without cutting: every draw move is sent \
                         as a pen-up travel. Leave it unchecked to drop the blade and cut.",
                    );
                ui.horizontal(|ui| {
                    ui.label("Overcut:");
                    ui.add(
                        egui::DragValue::new(&mut self.profile.overcut_mm)
                            .speed(0.05)
                            .range(0.0..=2.0)
                            .suffix(" mm"),
                    )
                    .on_hover_text(
                        "Cut this far past the start of each closed contour. A drag knife \
                         leaves a thin vinyl bridge wherever the entry and exit cuts fail to \
                         meet; 0.1–0.3 mm closes it. 0 disables overcut.",
                    );
                });

                // ---- Machine setup: force, speed, pen-up speed -----------------------
                // These are sent as the profile's own command templates. A blank value
                // means "leave the cutter's front-panel setting alone", so an operator who
                // has never touched these gets exactly the previous behaviour.
                ui.collapsing("Machine setup", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Force:");
                        let mut set = self.profile.force.is_some();
                        if ui.checkbox(&mut set, "").changed() {
                            self.profile.force = set.then(|| 100.0);
                        }
                        if let Some(v) = &mut self.profile.force {
                            ui.add(egui::DragValue::new(v).speed(1.0).range(1.0..=1000.0));
                        } else {
                            ui.label("(cutter default)");
                        }
                    })
                    .response
                    .on_hover_text(
                        "Blade pressure. Too low leaves the vinyl uncut, too high scores the \
                         backing. Typical 80–160. Needs force_cmd in the profile.",
                    );
                    ui.horizontal(|ui| {
                        ui.label("Speed:");
                        let mut set = self.profile.speed.is_some();
                        if ui.checkbox(&mut set, "").changed() {
                            self.profile.speed = set.then(|| 40.0);
                        }
                        if let Some(v) = &mut self.profile.speed {
                            ui.add(egui::DragValue::new(v).speed(1.0).range(1.0..=1000.0));
                        } else {
                            ui.label("(cutter default)");
                        }
                    })
                    .response
                    .on_hover_text("Cutting speed in machine units. Typical 20–60.");
                    ui.horizontal(|ui| {
                        ui.label("Pen-up speed:");
                        let mut set = self.profile.up_speed.is_some();
                        if ui.checkbox(&mut set, "").changed() {
                            self.profile.up_speed = set.then(|| 60.0);
                        }
                        if let Some(v) = &mut self.profile.up_speed {
                            ui.add(egui::DragValue::new(v).speed(1.0).range(1.0..=1000.0));
                        } else {
                            ui.label("(cutter default)");
                        }
                    })
                    .response
                    .on_hover_text("Travel speed with the blade lifted. Typical 40–80.");
                });

                // ---- Travel limits ---------------------------------------------------
                ui.collapsing("Travel limits", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Max X:");
                        let mut set = self.profile.max_x_mm.is_some();
                        if ui.checkbox(&mut set, "").changed() {
                            self.profile.max_x_mm = set.then_some(635.0);
                        }
                        if let Some(v) = &mut self.profile.max_x_mm {
                            ui.add(egui::DragValue::new(v).speed(1.0).range(1.0..=3000.0))
                                .on_hover_text(
                                    "Machine travel width. A design past this is refused \
                                     instead of driving the carriage into its end stop. \
                                     SC2 widths: 635, 863.6, 1346.2 mm.",
                                );
                        } else {
                            ui.label("(no limit)");
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Max Y:");
                        let mut set = self.profile.max_y_mm.is_some();
                        if ui.checkbox(&mut set, "").changed() {
                            self.profile.max_y_mm = set.then_some(3000.0);
                        }
                        if let Some(v) = &mut self.profile.max_y_mm {
                            ui.add(egui::DragValue::new(v).speed(1.0).range(1.0..=30000.0));
                        } else {
                            ui.label("(no limit)");
                        }
                    });
                });

                // ---- Path quality ----------------------------------------------------
                ui.collapsing("Path quality", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Blade offset:");
                        ui.add(
                            egui::DragValue::new(&mut self.profile.blade_offset_mm)
                                .speed(0.01)
                                .range(0.0..=2.0)
                                .suffix(" mm"),
                        )
                        .on_hover_text(
                            "Drag-knife corner compensation: how far the blade tip trails \
                             the pivot. 0 disables. Typical 0.25–0.5 mm.",
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Min line:");
                        ui.add(
                            egui::DragValue::new(&mut self.profile.min_line_mm)
                                .speed(0.01)
                                .range(0.0..=5.0)
                                .suffix(" mm"),
                        )
                        .on_hover_text(
                            "Drop line segments shorter than this. Prevents blobs and wasted \
                             blade cycles. 0 disables.",
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Merge gap:");
                        ui.add(
                            egui::DragValue::new(&mut self.profile.linemerge_tol_mm)
                                .speed(0.01)
                                .range(0.0..=5.0)
                                .suffix(" mm"),
                        )
                        .on_hover_text(
                            "Join open contours whose ends meet within this distance. \
                             0 disables.",
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Repeat passes:");
                        ui.add(
                            egui::DragValue::new(&mut self.profile.repeat_passes)
                                .speed(1.0)
                                .range(1..=10),
                        )
                        .on_hover_text(
                            "Cut each contour this many times. Thick material needs 2–3.",
                        );
                    });
                });
                ui.separator();
                ui.label(&self.status);
            });
        });

        egui::containers::Panel::left("controls")
            .resizable(true)
            .default_size(320.0)
            .show(ui, |ui| {
                ui.heading("Design");
                ui.horizontal(|ui| {
                    if ui.button("Load SVG/DXF").clicked() {
                        if let Some(p) = rfd::FileDialog::new()
                            .add_filter("Designs", &["svg", "dxf"])
                            .pick_file()
                        {
                            self.load_design(&p);
                        }
                    }
                    if ui.button("Clear").clicked() {
                        self.raw_polylines.clear();
                        self.raw_colors.clear();
                        self.raw_bbox = (Vec2::ZERO, Vec2::ZERO);
                        self.needs_fit = true;
                        self.log("Cleared design");
                    }
                });
                ui.label(format!(
                    "Paths: {}  BBox: {:.1} × {:.1} mm",
                    self.raw_polylines.len(),
                    self.raw_bbox.1.x - self.raw_bbox.0.x,
                    self.raw_bbox.1.y - self.raw_bbox.0.y
                ));

                ui.separator();
                ui.heading("Roll");
                ui.horizontal(|ui| {
                    ui.label("Width (mm)");
                    ui.add(egui::DragValue::new(&mut self.roll_width_mm).speed(1.0).range(1.0..=3000.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Length (mm)");
                    ui.add(egui::DragValue::new(&mut self.roll_length_mm).speed(10.0).range(1.0..=20000.0));
                });
                if ui.button("Fit roll to view").clicked() {
                    self.needs_fit = true;
                }

                ui.separator();
                ui.heading("Transform");
                ui.horizontal(|ui| {
                    ui.label("Move X");
                    ui.add(egui::DragValue::new(&mut self.offset.x).speed(1.0));
                    ui.label("Y");
                    ui.add(egui::DragValue::new(&mut self.offset.y).speed(1.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Scale");
                    ui.add(egui::DragValue::new(&mut self.scale).speed(0.01).range(0.01..=100.0));
                    ui.label("Rotate °");
                    let mut deg = self.rotation.to_degrees();
                    if ui.add(egui::DragValue::new(&mut deg).speed(1.0)).changed() {
                        self.rotation = deg.to_radians();
                    }
                });
                if ui.button("Reset transform").clicked() {
                    self.offset = Vec2::ZERO;
                    self.scale = 1.0;
                    self.rotation = 0.0;
                }

                ui.separator();
                ui.heading("Step & repeat");
                ui.horizontal(|ui| {
                    ui.label("Copies X");
                    ui.add(egui::DragValue::new(&mut self.copies_x).speed(1.0).range(1..=100));
                    ui.label("Y");
                    ui.add(egui::DragValue::new(&mut self.copies_y).speed(1.0).range(1..=100));
                });
                ui.horizontal(|ui| {
                    ui.label("Step X");
                    ui.add(egui::DragValue::new(&mut self.step_x).speed(1.0));
                    ui.label("Y");
                    ui.add(egui::DragValue::new(&mut self.step_y).speed(1.0));
                });

                ui.separator();
                ui.heading("Cutter");
                ui.horizontal(|ui| {
                    if ui.button("Send design").clicked() {
                        self.send_design();
                    }
                    if ui.button("Test square").clicked() {
                        self.send_test_square();
                    }
                    if ui.button("Home").clicked() {
                        self.home();
                    }
                });
                ui.horizontal(|ui| {
                    if ui.button("◀ 10").clicked() {
                        self.jog(-10.0, 0.0);
                    }
                    if ui.button("▶ 10").clicked() {
                        self.jog(10.0, 0.0);
                    }
                    if ui.button("▲ 10").clicked() {
                        self.jog(0.0, 10.0);
                    }
                    if ui.button("▼ 10").clicked() {
                        self.jog(0.0, -10.0);
                    }
                });
                ui.label(format!("Tracked pos: {:.1}, {:.1} mm", self.pos_mm.x, self.pos_mm.y));

                ui.separator();
                ui.heading("Log");
                for line in &self.log {
                    ui.label(line);
                }
            });

        egui::containers::CentralPanel::no_frame().show(ui, |ui| {
            let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());
            let rect = response.rect;

            painter.rect_filled(rect, egui::CornerRadius::ZERO, egui::Color32::from_rgb(24, 26, 30));

            if self.needs_fit {
                // Fit the roll width across the view; put the roll's origin near the bottom-left
                // so a design drawn in +X/+Y is visible immediately.
                let fit = (rect.width() as f64 / self.roll_width_mm.max(1.0)) as f32 * 0.9;
                self.zoom = fit.max(0.02);
                self.pan = egui::vec2(24.0, rect.height() - 24.0);
                self.needs_fit = false;
            }

            // Scroll zoom around the pointer.
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 && response.hovered() {
                let old = self.zoom;
                self.zoom = (self.zoom * (1.0 + scroll * 0.0015)).clamp(0.02, 40.0);
                if let Some(c) = response.hover_pos() {
                    let p = c - rect.min;
                    self.pan = p - (p - self.pan) * (self.zoom / old);
                }
            }

            let polys = self.transformed();
            let design_bbox = bbox_of(&polys);

            // Drag: secondary button pans; primary on the design moves it; otherwise pans.
            if response.drag_started() {
                self.drag_design = false;
                if let Some(p) = response.interact_pointer_pos() {
                    let mm = screen_to_mm(p, rect, self.pan, self.zoom);
                    self.drag_design = mm.x >= design_bbox.0.x
                        && mm.x <= design_bbox.1.x
                        && mm.y >= design_bbox.0.y
                        && mm.y <= design_bbox.1.y;
                }
            }
            if response.dragged() {
                let d = response.drag_delta();
                if response.dragged_by(egui::PointerButton::Secondary) || !self.drag_design {
                    self.pan += d;
                } else {
                    self.offset.x += d.x as f64 / self.zoom as f64;
                    self.offset.y -= d.y as f64 / self.zoom as f64;
                }
            }

            // Roll outline.
            let roll = [
                mm_to_screen(Vec2::new(0.0, 0.0), rect, self.pan, self.zoom),
                mm_to_screen(Vec2::new(self.roll_width_mm, 0.0), rect, self.pan, self.zoom),
                mm_to_screen(Vec2::new(self.roll_width_mm, self.roll_length_mm), rect, self.pan, self.zoom),
                mm_to_screen(Vec2::new(0.0, self.roll_length_mm), rect, self.pan, self.zoom),
            ];
            let edge = egui::Stroke::new(1.5, egui::Color32::from_rgb(120, 130, 150));
            for i in 0..4 {
                painter.line_segment([roll[i], roll[(i + 1) % 4]], edge);
            }

            // 100 mm grid inside the roll.
            let grid = egui::Stroke::new(0.5, egui::Color32::from_rgb(45, 50, 60));
            let mut x = 0.0;
            while x <= self.roll_width_mm {
                let a = mm_to_screen(Vec2::new(x, 0.0), rect, self.pan, self.zoom);
                let b = mm_to_screen(Vec2::new(x, self.roll_length_mm), rect, self.pan, self.zoom);
                painter.line_segment([a, b], grid);
                x += 100.0;
            }
            let mut y = 0.0;
            while y <= self.roll_length_mm {
                let a = mm_to_screen(Vec2::new(0.0, y), rect, self.pan, self.zoom);
                let b = mm_to_screen(Vec2::new(self.roll_width_mm, y), rect, self.pan, self.zoom);
                painter.line_segment([a, b], grid);
                y += 100.0;
            }

            // Origin marker.
            let origin = mm_to_screen(Vec2::new(0.0, 0.0), rect, self.pan, self.zoom);
            painter.circle_filled(origin, 3.0, egui::Color32::from_rgb(230, 180, 60));

            // Design.
            for poly in &polys {
                if poly.len() < 2 {
                    continue;
                }
                let pts: Vec<egui::Pos2> = poly
                    .iter()
                    .map(|p| mm_to_screen(*p, rect, self.pan, self.zoom))
                    .collect();
                let closed = poly.len() > 2 && (poly[0] - poly[poly.len() - 1]).len() < 1e-6;
                let color = if closed {
                    egui::Color32::from_rgb(90, 220, 140)
                } else {
                    egui::Color32::from_rgb(120, 190, 255)
                };
                painter.add(egui::Shape::line(pts, egui::Stroke::new(1.4, color)));
            }

            // Design bbox hint.
            if !polys.is_empty() {
                let a = mm_to_screen(design_bbox.0, rect, self.pan, self.zoom);
                let b = mm_to_screen(design_bbox.1, rect, self.pan, self.zoom);
                let r = egui::Rect::from_two_pos(a, b);
                painter.rect_stroke(
                    r,
                    egui::CornerRadius::ZERO,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(80, 90, 110)),
                    egui::StrokeKind::Inside,
                );
            }

            painter.text(
                rect.left_top() + egui::vec2(12.0, 12.0),
                egui::Align2::LEFT_TOP,
                format!(
                    "Roll {:.0} × {:.0} mm  zoom {:.2}×  drag design to move; scroll to zoom; right-drag to pan",
                    self.roll_width_mm, self.roll_length_mm, self.zoom
                ),
                egui::FontId::proportional(13.0),
                egui::Color32::from_rgb(170, 180, 200),
            );
        });
    }
}
