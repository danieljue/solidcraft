//! VinylCraft: a 2D vector design + vinyl cutter control prototype.
//!
//! This is the "MANUFACTURE" vision as a standalone app first: a roll-aware
//! canvas, SVG/DXF import, transforms, and DMPL output to a US Cutter SC2.
//! The CAM crate stays UI-free so it can be lifted into SolidCraft's
//! MANUFACTURE tab later.

use eframe::egui;
use solidcraft_cam::align::{solve as solve_alignment, Similarity};
use solidcraft_cam::boolean::{boolean_paths, BooleanOp, FillMode};
use solidcraft_cam::filters;
use solidcraft_cam::geom::{Point, Segment, Toolpath};
use solidcraft_cam::post;
use solidcraft_cam::profile::{
    FlowControl as CamFlow, MachineProfile, SerialSettings, ToolPreset,
};
use serde_json::{Value, json};
use solidcraft_geom::Vec2;
use solidcraft_io::{read_dxf, read_svg_colored, Geom2};
// Text primitives arrive from SVG/DXF as Geom2::Text with no outlines. The sketch crate's font
// engine is what turns a string into cuttable contours, so a text primitive is resolved here
// rather than being silently dropped at flatten time.
use solidcraft_sketch::{text_geometry, LinkGeom};
use std::collections::{BTreeMap, HashSet};
use std::fs;
use std::io::{BufRead, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::time::Duration;

/// Pause/cancel gate for a job that is already being written to the port. A vinyl cutter
/// buffers a few hundred bytes to a few kilobytes, so withholding the rest of the stream
/// stops it once that buffer drains — this is the only pause a write-only serial protocol
/// offers. The operator can hold the job, let the machine catch up, then continue without
/// aborting and starting the contour over.
#[derive(Default)]
struct SendControl {
    pause: AtomicBool,
    cancel: AtomicBool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut app = VinylApp::new();
    if let Some(p) = arg_value(&args, "--profile") {
        app.load_profile(Path::new(&p));
    }
    if let Some(p) = arg_value(&args, "--port") {
        app.port_name = p;
    }
    if let Some(b) = arg_value(&args, "--baud").and_then(|b| b.parse::<u32>().ok()) {
        app.baud = b;
    }
    if let Some(p) = arg_value(&args, "--load") {
        app.load_design(Path::new(&p));
    }
    if args.iter().any(|a| a == "--demo") {
        app.load_demo();
    }

    // Remote control of a running GUI. This is the "close the program" path: the GUI is a
    // separate process, so the request travels over its control socket instead of through
    // this process's own state.
    let control_port = arg_value(&args, "--control-port")
        .and_then(|p| p.parse::<u16>().ok())
        .or_else(|| {
            fs::read_to_string(control_port_file())
                .ok()
                .and_then(|s| s.trim().parse::<u16>().ok())
        })
        .unwrap_or(CONTROL_PORT_DEFAULT);
    if args.iter().any(|a| a == "--quit") {
        match control_request(control_port, json!({"jsonrpc": "2.0", "id": 1, "method": "quit"})) {
            Ok(v) => {
                println!("{v}");
                return Ok(());
            }
            Err(e) => {
                eprintln!("control socket unavailable ({e}); terminating the matching process by path");
                quit_by_process_name()?;
                return Ok(());
            }
        }
    }
    if args.iter().any(|a| a == "--state") {
        match control_request(control_port, json!({"jsonrpc": "2.0", "id": 1, "method": "state"})) {
            Ok(v) => {
                println!("{v}");
                return Ok(());
            }
            Err(e) => return Err(e.into()),
        }
    }

    // Headless JSON-RPC (MCP-shaped) server: drive the same transform/emit path the GUI uses.
    if args.iter().any(|a| a == "--mcp") {
        run_mcp(app)?;
        return Ok(());
    }

    // Headless one-shot emit: prove move -> coordinates without opening a window.
    if args.iter().any(|a| a == "--emit") {
        // `--out` is the documented flag; keep accepting a value after `--emit` for old
        // command lines. The previous order looked up `--emit` first, so the documented
        // `--emit --out file.dmpl` silently wrote to the default name instead.
        let out = arg_value(&args, "--out")
            .or_else(|| arg_value(&args, "--emit"))
            .unwrap_or_else(|| "vinyl_out.dmpl".into());
        let pen_up = !args.iter().any(|a| a == "--cut");
        let code = app.emit(pen_up)?;
        fs::write(&out, &code)?;
        let d = bbox_of(&app.transformed());
        println!(
            "wrote {} bytes to {} | design bbox x {:.2}..{:.2} y {:.2}..{:.2} mm | paths {}",
            code.len(),
            out,
            d.0.x,
            d.1.x,
            d.0.y,
            d.1.y,
            app.raw_polylines.len()
        );
        return Ok(());
    }

    // A blank canvas is a poor first run: show the demo sticker unless a design was loaded.
    if app.raw_polylines.is_empty() {
        app.load_demo();
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 860.0])
            .with_min_inner_size([900.0, 600.0])
            .with_title("VinylCraft — 2D design and cutter control"),
        ..Default::default()
    };
    eframe::run_native(
        "VinylCraft",
        options,
        Box::new(move |cc| {
            // The window exists from here on, so this is the first point where a control
            // client can reach a live UI. Attaching before the first frame means `--quit`
            // works immediately after launch instead of racing startup.
            app.attach_control(cc.egui_ctx.clone());
            Ok(Box::new(app))
        }),
    )
    .map_err(|e| e.to_string())?;
    Ok(())
}

/// The value after `key` on the command line, when it is not another flag.
fn arg_value(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1))
        .filter(|v| !v.starts_with("--"))
        .cloned()
}

#[derive(Debug, Clone)]
enum JobMsg {
    Done(String),
    Err(String),
}

/// Commands a running GUI accepts over its local control socket.
///
/// The GUI is a separate process from any `--mcp` invocation, so "close the program" cannot
/// travel through an app's own state — it has to cross a process boundary. This enum is the
/// payload of that crossing.
enum CtrlMsg {
    State(mpsc::Sender<Value>),
    Call {
        name: String,
        args: Value,
        reply: mpsc::Sender<Result<Value, String>>,
    },
}

/// Set when a control client asks the app to close. A static is the simplest handoff between
/// the socket thread and the UI thread: the GUI polls it each frame and sends the close
/// command, so nothing has to be threaded through the app's own state.
static QUIT_REQUESTED: AtomicBool = AtomicBool::new(false);

const CONTROL_PORT_DEFAULT: u16 = 47777;

/// Where the GUI advertises the port it actually bound. The default port is the common case,
/// but a second instance or a taken port pushes the listener to an ephemeral one; without a
/// hint file a client would have no way to find it.
fn control_port_file() -> std::path::PathBuf {
    std::env::temp_dir().join("vinylcraft.ctl")
}

/// A serial port the OS reports, annotated with the USB identity of whatever is behind it.
///
/// A bare port name is not an identity. Windows reuses COM numbers in enumeration order, so
/// when a device is unplugged its name becomes free and the next device to appear can take
/// it. Sending a cut job to "COM3" because it used to be the cutter is how a job gets shouted
/// into an unrelated microcontroller while the UI reports success.
#[derive(Debug, Clone)]
struct PortChoice {
    name: String,
    /// USB vendor/product id, when the port is a USB device.
    usb: Option<(u16, u16)>,
    /// Human description, e.g. "USB-SERIAL CH340".
    label: String,
}

impl PortChoice {
    /// USB bridges that vinyl cutters actually ship with: WCH CH340/CH341, FTDI FT232,
    /// Prolific PL2303, Silicon Labs CP210x. An STM32 native CDC port (`0483:5740`) is a
    /// dev board, not a cutter — it must not be treated as one just because it has a COM name.
    fn looks_like_cutter(&self) -> bool {
        match self.usb {
            Some((vid, pid)) => matches!(
                (vid, pid),
                (0x1A86, 0x7523)   // CH340
                    | (0x1A86, 0x5523) // CH341
                    | (0x0403, 0x6001) // FT232R
                    | (0x0403, 0x6015) // FT231X
                    | (0x067B, 0x2303) // PL2303
                    | (0x10C4, 0xEA60) // CP2102
            ),
            None => false,
        }
    }

    fn display(&self) -> String {
        match self.usb {
            Some((vid, pid)) => format!("{} — {} [{:04X}:{:04X}]", self.name, self.label, vid, pid),
            None => format!("{} — {}", self.name, self.label),
        }
    }
}

/// One design queued for nesting.
///
/// The polylines are already in design space: `vinyl_job_add` bakes the current transform
/// (offset, scale, rotation, copies) in when the job is queued. That keeps a job's shape
/// frozen at the moment it was added, so changing the live design afterwards cannot silently
/// move a job that is already in the batch.
struct Job {
    name: String,
    polylines: Vec<Vec<Vec2>>,
    /// Colour label used to group jobs. Roll nesting packs only the jobs that
    /// share a colour into one queue, because a cutter holds one vinyl colour at a time:
    /// mixing colours in one pass would cut a single-colour design twice. Empty means "no
    /// colour declared" and nests with the other unlabelled jobs.
    color: String,
}

impl Job {
    /// Bounding box of this job's polylines, as `(min, max)`.
    fn bbox(&self) -> (Vec2, Vec2) {
        bbox_of(&self.polylines)
    }
}

/// One colour group after nesting: the packed placements and the command stream for that
/// colour.
///
/// A group is one run of the machine. Roll nesting puts same-colour jobs in a
/// single queue because the cutter holds one vinyl colour at a time — a multi-colour design
/// becomes several runs, each cut on its own vinyl.
struct NestedGroup {
    color: String,
    /// `(job name, origin in mm, rotated)` for each placed job.
    placed: Vec<(String, Vec2, bool)>,
    /// Job names that did not fit the roll in this group.
    unplaced: Vec<String>,
    used_length_mm: f64,
    code: String,
}

struct VinylApp {
    profile: MachineProfile,
    profile_path: String,
    raw_polylines: Vec<Vec<Vec2>>,
    /// Parallel to `raw_polylines`: the stroke/fill colour each polyline was drawn with, or
    /// `None` when the source declares none. An SVG can hold several vinyl colours in one file;
    /// this is what lets `vinyl_job_add` split such a design into one job per colour. DXF has no
    /// colour attribute we read, so its polylines are all `None`.
    raw_colors: Vec<Option<String>>,
    raw_bbox: (Vec2, Vec2),
    // transform
    offset: Vec2,
    scale: f64,
    rotation: f64, // radians
    copies_x: u32,
    copies_y: u32,
    step_x: f64,
    step_y: f64,
    // roll
    roll_width_mm: f64,
    roll_length_mm: f64,
    // view
    zoom: f32,
    pan: egui::Vec2,
    needs_fit: bool,
    drag_design: bool,
    // output
    pen_up_only: bool,
    port_name: String,
    baud: u32,
    status: String,
    log: Vec<String>,
    pos_mm: Vec2,
    /// Where a motion command asked the carriage to go. Committed to `pos_mm` only when the
    /// send reports success. Updating the tracked position before the bytes reach the machine
    /// makes a dead port look like a moving cutter: the app says "now at X" while nothing left
    /// the cable. A failed send must leave `pos_mm` where it was.
    pending_pos: Option<Vec2>,
    job_rx: Option<Receiver<JobMsg>>,
    /// Serial ports the OS reports, refreshed on demand. A hardcoded port name is a
    /// guess: Windows reassigns COM numbers when devices are replugged, so the port
    /// that worked yesterday can be a different board today.
    ports: Vec<PortChoice>,
    /// Requests from the control socket. `None` in headless mode, where no GUI exists to
    /// serve them.
    ctrl_rx: Option<Receiver<CtrlMsg>>,
    /// Port the control socket bound to (0 when it is not running).
    ctrl_port: u16,
    /// Pause/cancel gate shared with the thread that is writing the current job. `pause`
    /// holds the stream where it is; `cancel` abandons the rest of the job.
    send_ctl: Arc<SendControl>,
    /// Registration marks to cut around the design. `None` leaves the design alone.
    marks: Option<solidcraft_cam::marks::MarkConfig>,
    /// Weed-box margin in mm. `Some(m)` cuts a rectangle `m` clear of the design so the
    /// waste peels in one piece; `None` cuts no box.
    weed_box_mm: Option<f64>,
    /// Lift-mark size in mm. `Some(s)` puts a short grab tab at each closed contour;
    /// `None` leaves contours as drawn.
    lift_marks_mm: Option<f64>,
    /// Tiling options. `Some` splits the job into tiles and emits one command stream per
    /// tile; `None` emits the whole design as one job.
    tile_opts: Option<solidcraft_cam::tile::TileOptions>,
    /// Jobs queued for nesting. Each entry is one design already transformed into design
    /// space. Empty until `vinyl_job_add` is used; `vinyl_nest` packs the whole queue onto
    /// the roll in one pass.
    jobs: Vec<Job>,
}

/// Enumerate serial ports with the USB identity of each one.
///
/// `available_ports` is the only place the OS tells us *what* is behind a COM name; without
/// it a port is just a string and "is the cutter plugged in?" becomes unanswerable.
fn scan_ports() -> Vec<PortChoice> {
    use serialport::SerialPortType;
    match serialport::available_ports() {
        Ok(list) => list
            .into_iter()
            .map(|p| {
                let (usb, label) = match p.port_type {
                    SerialPortType::UsbPort(info) => {
                        let label = match (&info.manufacturer, &info.product) {
                            (Some(m), Some(d)) => format!("{m} {d}"),
                            (None, Some(d)) => d.clone(),
                            (Some(m), None) => m.clone(),
                            (None, None) => "USB serial".into(),
                        };
                        (Some((info.vid, info.pid)), label)
                    }
                    SerialPortType::BluetoothPort => (None, "Bluetooth serial".into()),
                    SerialPortType::PciPort => (None, "PCI serial".into()),
                    SerialPortType::Unknown => (None, "unknown".into()),
                };
                PortChoice {
                    name: p.port_name,
                    usb,
                    label,
                }
            })
            .collect(),
        Err(_) => Vec::new(),
    }
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
        }
    }
}

impl VinylApp {
    fn new() -> Self {
        Self::default()
    }

    /// Rescan the OS's serial port list. There is no reliable push notification when a USB
    /// device appears or disappears, so this runs on demand.
    fn refresh_ports(&mut self) {
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
    fn probe_port(&mut self) {
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

    fn log(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        self.status = msg.clone();
        self.log.push(msg);
        if self.log.len() > 12 {
            self.log.remove(0);
        }
    }

    fn load_profile(&mut self, path: &Path) {
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

    fn load_design(&mut self, path: &Path) {
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

    fn transformed(&self) -> Vec<Vec<Vec2>> {
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
    fn transformed_colors(&self) -> Vec<Option<String>> {
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
    fn add_text(
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
    fn offset_design(&mut self, distance: f64, keep_original: bool) -> (usize, usize, usize) {
        if self.raw_polylines.is_empty() {
            return (0, 0, 0);
        }
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
    fn boolean_design(
        &mut self,
        op: BooleanOp,
        fill: FillMode,
        clip: Option<(f64, f64, f64, f64)>,
    ) -> (usize, usize, usize) {
        if self.raw_polylines.is_empty() {
            return (0, 0, 0);
        }

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
    fn placement_shift(&self) -> Option<Vec2> {
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
    fn finishing_margin(&self) -> f64 {
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
    fn finishing_geometry(&self, placed_bbox: (Vec2, Vec2)) -> Vec<Toolpath> {
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

    fn emit(&self, pen_up_only: bool) -> Result<String, String> {
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
    fn emit_tiled(&self, pen_up_only: bool) -> Result<Vec<(solidcraft_cam::tile::Tile, String)>, String> {
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
    fn emit_nested(
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

    fn send_code(&mut self, code: String, label: &str) {
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

    fn send_simple(&mut self, cmd: &str, label: &str) {
        self.send_code(cmd.to_string(), label);
    }

    fn poll_job(&mut self) {
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
    fn attach_control(&mut self, ctx: egui::Context) {
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
    fn poll_control(&mut self, ctx: &egui::Context) {
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

    fn send_design(&mut self) {
        match self.emit(self.pen_up_only) {
            Ok(code) => {
                let label = if self.pen_up_only { "pen-up design" } else { "design" };
                self.send_code(code, label);
            }
            Err(e) => self.log(e),
        }
    }

    fn send_test_square(&mut self) {
        // 10 mm square, pen up only, at 40 units/mm. Safe with no vinyl.
        self.send_simple("IN;PA;U0,0;U400,0;U400,400;U0,400;U0,0;@", "pen-up test square");
    }

    fn home(&mut self) {
        // Ask for the origin. The tracked position is committed only when the send reports
        // back; moving the readout first makes a dead port look like a moving carriage.
        self.pending_pos = Some(Vec2::ZERO);
        self.send_simple("IN;PA;U0,0;@", "home");
    }

    /// Travel limits for manual moves: the profile's declared machine limits when set, else
    /// the loaded roll. A jog is a manual move with no design to bound it, so without this a
    /// held-down arrow walks the carriage off the end of the rail.
    fn travel_limits(&self) -> (f64, f64) {
        (
            self.profile.max_x_mm.unwrap_or(self.roll_width_mm),
            self.profile.max_y_mm.unwrap_or(self.roll_length_mm),
        )
    }

    fn jog(&mut self, dx: f64, dy: f64) {
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
    fn pen_up(&mut self) {
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
    fn pen_down(&mut self) {
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
    fn pause_send(&mut self) {
        self.send_ctl.pause.store(true, Ordering::Relaxed);
        self.log("job paused — the cutter will stop once its buffer drains");
    }

    fn resume_send(&mut self) {
        self.send_ctl.pause.store(false, Ordering::Relaxed);
        self.log("job resumed");
    }

    /// Abandon the rest of the stream. The blade stays wherever the last completed command
    /// left it; call `pen_up` or `home` afterwards to lift it clear of the vinyl.
    fn cancel_send(&mut self) {
        self.send_ctl.cancel.store(true, Ordering::Relaxed);
        self.send_ctl.pause.store(false, Ordering::Relaxed);
        self.log("job cancelled — lift the blade with pen up or home before unloading");
    }

    fn send_paused(&self) -> bool {
        self.send_ctl.pause.load(Ordering::Relaxed)
    }

    /// A built-in sticker so the canvas is never empty and "move it around" can be tried at once.
    /// Three closed paths: a 300 × 150 mm rounded-rectangle outline and two concentric circles.
    fn load_demo(&mut self) {
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

/// Convert plain polylines into cutter toolpaths: drop degenerate runs and mark a
/// contour closed when its last point coincides with its first.
fn polylines_to_toolpaths(polys: &[Vec<Vec2>]) -> Vec<Toolpath> {
    polys
        .iter()
        .filter(|p| p.len() >= 2)
        .map(|p| {
            let closed = p.len() > 2 && (p[0] - p[p.len() - 1]).len() < 1e-6;
            let segments = p
                .windows(2)
                .map(|w| Segment::Line {
                    from: Point::new(w[0].x, w[0].y),
                    to: Point::new(w[1].x, w[1].y),
                })
                .collect();
            Toolpath { segments, closed }
        })
        .collect()
}

/// Convert a cutter toolpath back into a plain design polyline.
///
/// A closed toolpath is a ring: its segments chain end-to-start without repeating the first
/// point. A design polyline closes the other way — by repeating its first point as its last —
/// which is the convention [`polylines_to_toolpaths`] reads when it marks a contour closed. So
/// a closed path gets its first point appended, and an open path gets its final end point
/// appended to complete the last segment.
fn toolpath_to_polyline(tp: &Toolpath) -> Vec<Vec2> {
    let mut pts: Vec<Vec2> = tp
        .segments
        .iter()
        .map(|s| Vec2::new(s.start().x, s.start().y))
        .collect();
    if tp.closed {
        if let Some(first) = pts.first().copied() {
            pts.push(first);
        }
    } else if let Some(last) = tp.segments.last() {
        pts.push(Vec2::new(last.end().x, last.end().y));
    }
    pts
}

/// A rounded rectangle outline, counter-clockwise from the bottom-right corner, closed.
fn rounded_rect_poly(w: f64, h: f64, r: f64, seg: usize) -> Vec<Vec2> {
    let r = r.max(0.0).min(w * 0.5).min(h * 0.5);
    let centers = [
        (w - r, r),     // bottom-right
        (w - r, h - r), // top-right
        (r, h - r),     // top-left
        (r, r),         // bottom-left
    ];
    let starts = [
        -std::f64::consts::FRAC_PI_2,
        0.0,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
    ];
    let mut pts = Vec::with_capacity(4 * (seg + 1) + 1);
    for (i, (cx, cy)) in centers.iter().enumerate() {
        for k in 0..=seg {
            let a = starts[i] + std::f64::consts::FRAC_PI_2 * (k as f64 / seg as f64);
            pts.push(Vec2::new(cx + r * a.cos(), cy + r * a.sin()));
        }
    }
    pts.push(pts[0]);
    pts
}

/// A circle as a closed polyline.
fn circle_poly(c: Vec2, r: f64, n: usize) -> Vec<Vec2> {
    let n = n.max(8);
    let mut pts = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = i as f64 / n as f64 * std::f64::consts::TAU;
        pts.push(Vec2::new(c.x + r * t.cos(), c.y + r * t.sin()));
    }
    pts
}

/// Wait for the background serial thread to report, up to ~5 s (short commands only).
fn wait_for_job(app: &mut VinylApp) {
    for _ in 0..200 {
        app.poll_job();
        if app.job_rx.is_none() {
            return;
        }
        std::thread::sleep(Duration::from_millis(25));
    }
}

fn rpc_error(id: Value, code: i64, message: impl Into<String>) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message.into()}})
}

fn str_prop(desc: &str) -> Value {
    json!({"type": "string", "description": desc})
}

fn num_prop(desc: &str) -> Value {
    json!({"type": "number", "description": desc})
}

fn tool_schema(props: Value, required: &[&str]) -> Value {
    if required.is_empty() {
        json!({"type": "object", "properties": props})
    } else {
        json!({"type": "object", "properties": props, "required": required})
    }
}

fn vinyl_tools() -> Vec<Value> {
    vec![
        json!({
            "name": "vinyl_state",
            "description": "Read the app's full state: profile, port, roll size, transform, raw and transformed bounding boxes, tracked cutter position. Read-only.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_demo",
            "description": "Load a built-in 300 x 150 mm sticker design (rounded rectangle plus concentric circles) so there is a shape to move and cut.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_load",
            "description": "Load an SVG or DXF design from disk. Resets the transform.",
            "inputSchema": tool_schema(json!({"path": str_prop("Path to a .svg or .dxf file")}), &["path"])
        }),
        json!({
            "name": "vinyl_transform",
            "description": "Move, scale, rotate or tile the design. Only the fields given change. offset_x/offset_y are millimetres, rotation_deg in degrees, copies 1-100.",
            "inputSchema": tool_schema(json!({
                "offset_x": num_prop("X offset in mm"),
                "offset_y": num_prop("Y offset in mm"),
                "scale": num_prop("Scale factor"),
                "rotation_deg": num_prop("Rotation in degrees"),
                "copies_x": {"type": "integer", "description": "Copies along X"},
                "copies_y": {"type": "integer", "description": "Copies along Y"},
                "step_x": num_prop("Spacing between X copies in mm"),
                "step_y": num_prop("Spacing between Y copies in mm")
            }), &[])
        }),
        json!({
            "name": "vinyl_offset",
            "description": "Outline operation: offset every closed contour in the loaded design by distance_mm. Positive grows a contour outward (a cut border around the design); negative shrinks it inward (an inline contour). Corners are rounded. Open polylines are left untouched and reported, since they have no interior to offset against. With keep_original true the offsets are added alongside the originals so one cut pass produces both design and border; otherwise the geometry is replaced. The distance is in design-space millimetres, before the view transform. Returns the contour counts and the updated state.",
            "inputSchema": tool_schema(json!({
                "distance_mm": num_prop("Offset distance in mm; positive grows outward, negative shrinks inward"),
                "keep_original": {"type": "boolean", "description": "Keep the original contours and append the offsets (default false, which replaces the geometry)"}
            }), &["distance_mm"])
        }),
        json!({
            "name": "vinyl_boolean",
            "description": "Boolean operation on the design's filled regions — Weld, Trim, Punch, Intersect and Exclude. The design is grouped by colour and each colour group is operated on separately, because a cutter loads one vinyl colour at a time; welding a red shape into a blue one would produce geometry no single cut pass could make. Open polylines have no interior and pass through untouched. Without a clip rectangle, `union` welds each colour group against itself — merging overlaps and resolving self-intersections, the usual cleanup before cutting traced artwork. Give clip_x/clip_y/clip_width/clip_height to supply a rectangular second operand: difference punches it out, intersect trims to it, xor excludes it. The operation replaces the design geometry.",
            "inputSchema": tool_schema(json!({
                "operation": {
                    "type": "string",
                    "enum": ["union", "difference", "intersect", "xor"],
                    "description": "Which boolean to apply. union (weld) merges, difference (trim/punch) subtracts the clip, intersect keeps only the overlap, xor (exclude) keeps the non-overlapping parts."
                },
                "fill_mode": {
                    "type": "string",
                    "enum": ["nonzero", "evenodd"],
                    "description": "How contours fill. nonzero (default) fills where winding is non-zero — holes must wind opposite their outer contour, which is how glyph outlines already work. evenodd fills alternate rings regardless of winding; use it for imported art whose winding is inconsistent."
                },
                "clip_x": num_prop("Clip rectangle left edge in mm (design space)"),
                "clip_y": num_prop("Clip rectangle top edge in mm (design space)"),
                "clip_width": num_prop("Clip rectangle width in mm"),
                "clip_height": num_prop("Clip rectangle height in mm")
            }), &["operation"])
        }),
        json!({
            "name": "vinyl_text",
            "description": "Type lettering into the design. The string is outlined by an embedded font engine (Liberation Sans by default) and added to the design as cuttable contours: each glyph's outline and its counters (the holes of A, B, 8) become separate closed paths. Placement is in design space — x/y is the baseline's left end in mm and height_mm the capital height. On a blank canvas the view transform resets so the text lands exactly where it was typed; on a design that already has content the text is appended and the current transform applies to it along with everything else. Call vinyl_text on an empty design to start a sign from scratch. Newline characters start a new line.",
            "inputSchema": tool_schema(json!({
                "text": str_prop("The lettering to cut; a newline starts a new line"),
                "x": num_prop("Baseline start, X in mm (design space)"),
                "y": num_prop("Baseline start, Y in mm (design space)"),
                "height_mm": num_prop("Capital height in mm (default 10)"),
                "angle_deg": num_prop("Rotation about the baseline start, in degrees")
            }), &["text"])
        }),
        json!({
            "name": "vinyl_emit",
            "description": "Generate the cutter command stream (DMPL by default) from the current transform. Writes the file when `path` is given and returns byte count, preview and transformed bbox. pen_up true replaces draws with moves (dry run).",
            "inputSchema": tool_schema(json!({
                "path": str_prop("Where to write the command stream"),
                "pen_up": {"type": "boolean", "description": "Dry run: convert draws to pen-up moves (default: the app's Pen-up dry run setting)"},
                "overcut_mm": num_prop("Cut this far past the start of each closed contour in mm (default: profile setting)")
            }), &[])
        }),
        json!({
            "name": "vinyl_profile",
            "description": "Read or change machine settings: force, speed, pen-up speed, overcut, blade offset, min line length, contour merge gap, repeat passes, travel limits. Only the fields given change. Send null to clear a setting back to the cutter's own default. Returns the full state.",
            "inputSchema": tool_schema(json!({
                "force": num_prop("Blade pressure (null clears to the cutter's default)"),
                "speed": num_prop("Cutting speed in machine units"),
                "up_speed": num_prop("Pen-up travel speed"),
                "overcut_mm": num_prop("Cut this far past the start of each closed contour"),
                "blade_offset_mm": num_prop("Drag-knife corner compensation"),
                "min_line_mm": num_prop("Drop segments shorter than this"),
                "linemerge_tol_mm": num_prop("Join open contours whose ends meet within this distance"),
                "repeat_passes": {"type": "integer", "description": "Cut each contour this many times (1-10)"},
                "max_x_mm": num_prop("Machine travel width limit (null clears)"),
                "max_y_mm": num_prop("Machine travel length limit (null clears)"),
                "active_tool": {
                    "type": ["string", "null"],
                    "description": "Name of the blade or pen currently in the holder, matched case-insensitively against the profile's tool_presets. The selected preset's force, speed, blade offset and pass count override the flat profile values. Send null to fall back to the profile's flat values."
                },
                "tool_presets": {
                    "type": "array",
                    "description": "Replace the profile's whole preset list. Each entry needs a name; force, speed, blade_offset_mm and repeat_passes are optional. Omit this key to leave the list untouched.",
                    "items": {
                        "type": "object",
                        "properties": {
                            "name": {"type": "string"},
                            "force": num_prop("Downforce for this tool; null or 0 leaves the machine setting alone"),
                            "speed": num_prop("Cutting speed for this tool; null or 0 leaves the machine setting alone"),
                            "blade_offset_mm": num_prop("Drag-knife trail for this blade geometry; a pen or crease tool is 0"),
                            "repeat_passes": {"type": "integer", "description": "Passes for this tool; 0 inherits the profile's value"}
                        },
                        "required": ["name"]
                    }
                }
            }), &[])
        }),
        json!({
            "name": "vinyl_send",
            "description": "Send the current design to the cutter over the configured serial port. pen_up overrides the app's dry-run setting.",
            "inputSchema": tool_schema(json!({
                "pen_up": {"type": "boolean", "description": "Send pen-up moves only (dry run)"}
            }), &[])
        }),
        json!({
            "name": "vinyl_jog",
            "description": "Move the cutter carriage by dx, dy millimetres (pen up) and report the tracked position.",
            "inputSchema": tool_schema(json!({"dx": num_prop("X move in mm"), "dy": num_prop("Y move in mm")}), &["dx", "dy"])
        }),
        json!({
            "name": "vinyl_home",
            "description": "Send the carriage to the origin (0, 0), pen up.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_test_square",
            "description": "Send a 10 mm pen-up test square (safe with no vinyl loaded).",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_pen_up",
            "description": "Lift the blade at the carriage's current position. Always safe to send.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_pen_down",
            "description": "Drop the blade at the carriage's current position without cutting a path. The blade stays down until pen_up or home. Some firmware ignores a zero-length move, so verify visually.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_pause",
            "description": "Hold the job currently being sent. The write thread stops feeding bytes; the cutter finishes what is already in its buffer and waits. Resume with vinyl_resume, or abandon with vinyl_cancel.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_resume",
            "description": "Continue a paused job from where it stopped.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_cancel",
            "description": "Abandon the rest of a job in progress. The blade stays where the last completed command left it — follow with vinyl_pen_up or vinyl_home.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_set_port",
            "description": "Change the serial port the app sends to, without restarting. Use vinyl_ports first to see which ports exist and whether each is a cutter bridge. Returns the port and whether it looks like a cutter.",
            "inputSchema": tool_schema(json!({
                "port": str_prop("Port name, e.g. COM4"),
                "baud": {"type": "integer", "description": "Baud rate (default: keep current)"}
            }), &["port"])
        }),
        json!({
            "name": "vinyl_ports",
            "description": "List serial ports the OS reports, with USB identity and whether each looks like a cutter bridge. Use this to find the real cutter port instead of guessing.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_clear",
            "description": "Remove the loaded design.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_marks",
            "description": "Turn registration marks on or off. Marks cut BEFORE the design so the operator can align the sheet without the blade later dragging across a finished mark. `enabled: false` clears them; omitting `enabled` leaves the current setting and only changes the style. The marks are counted in the roll-fit check, so a design that just fits still leaves room for them.",
            "inputSchema": tool_schema(json!({
                "enabled": {"type": "boolean", "description": "false clears marks; omit to keep them on and just adjust style"},
                "style": {"type": "string", "description": "corner_l (default), cross, or circle_cross"},
                "size_mm": num_prop("Mark size in mm (corner arm length, cross arm, or circle diameter)"),
                "margin_mm": num_prop("Distance from the design's edge to the mark's anchor"),
                "placement": {"type": "string", "description": "corners (default), edge_midpoints, or centre"},
                "segments": {"type": "integer", "description": "Circle segment count for circle_cross (4-128)"}
            }), &[])
        }),
        json!({
            "name": "vinyl_weed_box",
            "description": "Cut a rectangle around the design so the waste vinyl peels away in one piece. `enabled: false` clears it. The box is counted in the roll-fit check.",
            "inputSchema": tool_schema(json!({
                "enabled": {"type": "boolean", "description": "false clears the weed box"},
                "margin_mm": num_prop("How far outside the design's bounding box to cut the box")
            }), &[])
        }),
        json!({
            "name": "vinyl_lift_marks",
            "description": "Add a short grab tab at each closed contour so the finished piece can be lifted without a knife. `enabled: false` clears them.",
            "inputSchema": tool_schema(json!({
                "enabled": {"type": "boolean", "description": "false clears lift marks"},
                "size_mm": num_prop("Length of each lift tab in mm")
            }), &[])
        }),
        json!({
            "name": "vinyl_tile",
            "description": "Split the design into tiles so a piece larger than the cutter's travel can still be cut. Each tile is emitted as its own job; neighbouring tiles overlap so the seams join. `enabled: false` clears tiling. Note: vinyl_emit reports the whole design as one stream — tiling changes how the job is divided, so check the tile count in the response.",
            "inputSchema": tool_schema(json!({
                "enabled": {"type": "boolean", "description": "false clears tiling"},
                "tile_w": num_prop("Tile width in mm"),
                "tile_h": num_prop("Tile height in mm"),
                "overlap_mm": num_prop("Overlap between neighbouring tiles in mm"),
                "box_around_tile": {"type": "boolean", "description": "Cut a box around each tile to make alignment visible"},
                "box_margin_mm": num_prop("Margin for the tile box")
            }), &[])
        }),
        json!({
            "name": "vinyl_align",
            "description": "Solve the similarity transform (uniform scale, rotation, translation) that maps design-space reference points onto observed machine-space points. This is the manual alignment workflow: print registration marks, jog the blade to each mark in turn with vinyl_jog, then pass the design position and the machine position of each mark here. The computed transform is applied to the design's offset/scale/rotation fields so the design lands on the printed artwork. Needs at least one point; one point gives translation only, two give rotation, three or more also fit scale. Returns the computed scale, rotation in degrees, and translation.",
            "inputSchema": tool_schema(json!({
                "design_points": {
                    "type": "array",
                    "description": "Reference points in design space (x, y in mm), in the order they were printed. Usually the corners of the printed artwork's bounding box.",
                    "items": {
                        "type": "object",
                        "properties": {
                            "x": num_prop("X in mm"),
                            "y": num_prop("Y in mm")
                        },
                        "required": ["x", "y"]
                    }
                },
                "machine_points": {
                    "type": "array",
                    "description": "Matching points in machine space (x, y in mm) as jogged to each mark, in the same order as design_points. The order must match: design_points[0] pairs with machine_points[0].",
                    "items": {
                        "type": "object",
                        "properties": {
                            "x": num_prop("X in mm"),
                            "y": num_prop("Y in mm")
                        },
                        "required": ["x", "y"]
                    }
                }
            }), &["design_points", "machine_points"])
        }),
        json!({
            "name": "vinyl_job_add",
            "description": "Queue the currently loaded design as a job for nesting. The design's transform (offset, scale, rotation, copies) is baked in at the moment it is added, so later edits to the live design do not move a job already in the batch. Add several designs in turn to build a batch, then call vinyl_nest to pack them all onto one roll. Jobs that share a `color` are nested and cut together as one run; jobs of different colours become separate runs, because a cutter holds one vinyl colour at a time.",
            "inputSchema": tool_schema(json!({
                "name": {"type": "string", "description": "Label for this job. Defaults to job_N."},
                "color": {"type": "string", "description": "Vinyl colour for this job. Same-colour jobs are nested into one run; different colours become separate runs. Omit for a single-colour batch."}
            }), &[])
        }),
        json!({
            "name": "vinyl_job_list",
            "description": "List the jobs queued for nesting: name, colour, and bounding-box size of each. Returns an empty list when no jobs are queued.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_job_clear",
            "description": "Empty the nesting queue. The currently loaded design is not affected.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_nest",
            "description": "Pack the queued jobs onto the roll. This is roll nesting: bounding boxes are arranged on shelves across the roll width to minimise the roll length consumed. Jobs are ordered tallest-first; a job wider than the roll is turned 90 degrees if that makes it fit. Jobs are grouped by their `color` and each group is nested and emitted as its own run — roll nesting likewise packs only same-colour jobs together, because a cutter holds one vinyl colour at a time. Finishing geometry (registration marks, weed box, lift tabs) wraps each group's used area. Unlike vinyl_emit, the roll LENGTH is not a hard limit — a run may be as long as it needs, and each group reports its own used_length_mm. Write one stream per group to `path` when given (single colour keeps the exact path; multiple colours get a colour suffix). Jobs that could not fit the roll are listed per group in `unplaced`.",
            "inputSchema": tool_schema(json!({
                "gap_mm": num_prop("Clearance between jobs and between shelves in mm (default 2). Also serves as the kerf allowance."),
                "allow_rotation": {"type": "boolean", "description": "Turn a job 90 degrees when it only fits the roll that way (default true)"},
                "pen_up": {"type": "boolean", "description": "Emit a dry run (pen up) instead of cutting"},
                "path": {"type": "string", "description": "Write the combined command stream to this file"}
            }), &[])
        }),
        json!({
            "name": "vinyl_quit",
            "description": "Close the running app. In GUI mode this closes the window; in a headless --mcp session it ends the server. The GUI is a separate process, so reach it over its control socket (`--quit` does this for you).",
            "inputSchema": tool_schema(json!({}), &[])
        }),
    ]
}

fn state_json(app: &VinylApp) -> Value {
    let d = bbox_of(&app.transformed());
    // Build the preset array outside the outer json! block. Nesting a map that itself
    // invokes json! inside json! pushed the macro past the compiler's recursion limit.
    let presets: Vec<Value> = app
        .profile
        .tool_presets
        .iter()
        .map(|p| {
            json!({
                "name": p.name,
                "force": p.force,
                "speed": p.speed,
                "blade_offset_mm": p.blade_offset_mm,
                "repeat_passes": p.repeat_passes,
            })
        })
        .collect();
    // The finishing settings are also built outside the macro: each carries its own
    // json! invocation and match, and keeping them inline pushed the whole block past
    // the compiler's macro recursion limit.
    let marks: Option<Value> = app.marks.as_ref().map(|c| {
        json!({
            "style": match c.style {
                solidcraft_cam::marks::MarkStyle::CornerL { size_mm } => json!({"kind": "corner_l", "size_mm": size_mm}),
                solidcraft_cam::marks::MarkStyle::Cross { size_mm } => json!({"kind": "cross", "size_mm": size_mm}),
                solidcraft_cam::marks::MarkStyle::CircleCross { diameter_mm, segments } => json!({"kind": "circle_cross", "diameter_mm": diameter_mm, "segments": segments}),
            },
            "placement": match c.placement {
                solidcraft_cam::marks::MarkPlacement::Corners => "corners",
                solidcraft_cam::marks::MarkPlacement::EdgeMidpoints => "edge_midpoints",
                solidcraft_cam::marks::MarkPlacement::Centre => "centre",
            },
            "margin_mm": c.margin_mm,
        })
    });
    let tiling: Option<Value> = app.tile_opts.as_ref().map(|t| {
        json!({
            "tile_w": t.tile_w,
            "tile_h": t.tile_h,
            "overlap_mm": t.overlap_mm,
            "box_around_tile": t.box_around_tile,
            "box_margin_mm": t.box_margin_mm,
        })
    });
    // The init/feed commands are inserted AFTER the json! block rather than named inline.
    // Adding them inline pushed the macro past the compiler's expansion recursion limit —
    // the block already carries ~40 fields and each one costs a level of expansion. Two
    // extra fields was enough to tip it over. Inserting into the finished Value costs
    // nothing and keeps the block compiling.
    let header = app.profile.header.clone();
    let footer = app.profile.footer.clone();
    let mut state = json!({
        "profile": app.profile.name,
        "protocol": format!("{:?}", app.profile.protocol),
        "units_per_mm": app.profile.units_per_mm,
        "overcut_mm": app.profile.overcut_mm,
        "force": app.profile.force,
        "speed": app.profile.speed,
        "up_speed": app.profile.up_speed,
        "blade_offset_mm": app.profile.blade_offset_mm,
        "min_line_mm": app.profile.min_line_mm,
        "linemerge_tol_mm": app.profile.linemerge_tol_mm,
        "repeat_passes": app.profile.repeat_passes,
        "max_x_mm": app.profile.max_x_mm,
        "max_y_mm": app.profile.max_y_mm,
        // Tool presets: the operator needs to see which blade is in the holder and what it
        // actually does. `effective_*` is what the cut will really use — a selected preset
        // overrides the flat profile values, and reporting only the flat ones would hide
        // that a pen tool is about to run with no corner compensation.
        "active_tool": app.profile.active_tool,
        "effective_force": app.profile.effective_force(),
        "effective_speed": app.profile.effective_speed(),
        "effective_blade_offset_mm": app.profile.effective_blade_offset_mm(),
        "effective_repeat_passes": app.profile.effective_repeat_passes(),
        "tool_presets": presets,
        "port": app.port_name,
        "baud": app.baud,
        "pen_up_only": app.pen_up_only,
        "paths": app.raw_polylines.len(),
        "raw_bbox": [app.raw_bbox.0.x, app.raw_bbox.0.y, app.raw_bbox.1.x, app.raw_bbox.1.y],
        "design_bbox": [d.0.x, d.0.y, d.1.x, d.1.y],
        "offset": [app.offset.x, app.offset.y],
        "scale": app.scale,
        "rotation_deg": app.rotation.to_degrees(),
        "copies": [app.copies_x, app.copies_y],
        "step": [app.step_x, app.step_y],
        "roll_mm": [app.roll_width_mm, app.roll_length_mm],
        "pos_mm": [app.pos_mm.x, app.pos_mm.y],
        "pending_pos": app.pending_pos.map(|p| [p.x, p.y]),
        // Finishing settings must be visible in state: without them an operator cannot tell
        // whether a later emit will cut marks, a weed box, or tiles.
        "marks": marks,
        "weed_box_mm": app.weed_box_mm,
        "lift_marks_mm": app.lift_marks_mm,
        "tiling": tiling,
        "paused": app.send_paused(),
        "status": app.status,
        "log": app.log,
    });
    // Inserted here, not inline in the macro: the init and feed/eject commands are what an
    // operator needs when a machine ignores the design or leaves the blade parked in the
    // vinyl. Reporting only the protocol name hid whether a profile declared a header at
    // all. A JSON object is a map, so insertion order does not matter.
    state["header"] = json!(header);
    state["footer"] = json!(footer);
    // Inserted after the macro for the same reason as the header/footer: the json! block is
    // already at the compiler's expansion recursion limit. The queue depth is what tells a
    // caller whether vinyl_nest has anything to pack.
    state["jobs"] = json!(app.jobs.len());
    state
}

/// Parse an array of `{x, y}` objects into points. A malformed point names its own index so a
/// caller who sent forty corners knows which one to fix.
fn points_from_json(v: Option<&Value>) -> Result<Vec<Point>, String> {
    let Some(arr) = v.and_then(Value::as_array) else {
        return Err("expected an array of {x, y} objects".into());
    };
    arr.iter()
        .enumerate()
        .map(|(i, item)| {
            let x = item
                .get("x")
                .and_then(Value::as_f64)
                .ok_or_else(|| format!("point {i} is missing a numeric x"))?;
            let y = item
                .get("y")
                .and_then(Value::as_f64)
                .ok_or_else(|| format!("point {i} is missing a numeric y"))?;
            Ok(Point::new(x, y))
        })
        .collect()
}

fn call_vinyl_tool(app: &mut VinylApp, name: &str, args: &Value) -> Result<Value, String> {
    match name {
        "vinyl_state" => Ok(state_json(app)),
        "vinyl_demo" => {
            app.load_demo();
            Ok(state_json(app))
        }
        "vinyl_clear" => {
            app.raw_polylines.clear();
            app.raw_colors.clear();
            app.raw_bbox = (Vec2::ZERO, Vec2::ZERO);
            app.needs_fit = true;
            app.log("Cleared design");
            Ok(state_json(app))
        }
        "vinyl_load" => {
            let path = args.get("path").and_then(Value::as_str).ok_or("missing `path`")?;
            app.load_design(Path::new(path));
            Ok(state_json(app))
        }
        "vinyl_transform" => {
            if let Some(v) = args.get("offset_x").and_then(Value::as_f64) {
                app.offset.x = v;
            }
            if let Some(v) = args.get("offset_y").and_then(Value::as_f64) {
                app.offset.y = v;
            }
            if let Some(v) = args.get("scale").and_then(Value::as_f64) {
                app.scale = v;
            }
            if let Some(v) = args.get("rotation_deg").and_then(Value::as_f64) {
                app.rotation = v.to_radians();
            }
            if let Some(v) = args.get("copies_x").and_then(Value::as_u64) {
                app.copies_x = v.clamp(1, 100) as u32;
            }
            if let Some(v) = args.get("copies_y").and_then(Value::as_u64) {
                app.copies_y = v.clamp(1, 100) as u32;
            }
            if let Some(v) = args.get("step_x").and_then(Value::as_f64) {
                app.step_x = v;
            }
            if let Some(v) = args.get("step_y").and_then(Value::as_f64) {
                app.step_y = v;
            }
            Ok(state_json(app))
        }
        "vinyl_offset" => {
            // Required: an outline with no distance is meaningless. A caller that omits it gets
            // a named error rather than a silent zero-offset no-op.
            let distance = args
                .get("distance_mm")
                .and_then(Value::as_f64)
                .ok_or("missing `distance_mm`")?;
            let keep = args.get("keep_original").and_then(Value::as_bool).unwrap_or(false);
            let (offset_count, skipped_open, dropped) = app.offset_design(distance, keep);
            // The full state goes back plus the offset diagnostics, flat rather than nested so
            // a caller reads the counts the same way it reads every other tool's fields.
            let mut state = state_json(app);
            if let Some(obj) = state.as_object_mut() {
                obj.insert("offset_contours".into(), json!(offset_count));
                obj.insert("skipped_open_contours".into(), json!(skipped_open));
                obj.insert("dropped_collapsed_contours".into(), json!(dropped));
                obj.insert("keep_original".into(), json!(keep));
                obj.insert("offset_distance_mm".into(), json!(distance));
            }
            Ok(state)
        }
        "vinyl_text" => {
            // Required: an empty string produces no geometry, so name the error rather than
            // returning an empty state that looks like success.
            let text = args.get("text").and_then(Value::as_str).ok_or("missing `text`")?;
            let x = args.get("x").and_then(Value::as_f64).unwrap_or(0.0);
            let y = args.get("y").and_then(Value::as_f64).unwrap_or(0.0);
            let height = args.get("height_mm").and_then(Value::as_f64).unwrap_or(10.0);
            let angle_deg = args.get("angle_deg").and_then(Value::as_f64).unwrap_or(0.0);
            let (contours, segments) =
                app.add_text(text, Vec2::new(x, y), height, angle_deg.to_radians())?;
            let mut state = state_json(app);
            if let Some(obj) = state.as_object_mut() {
                obj.insert("text_contours".into(), json!(contours));
                obj.insert("text_segments".into(), json!(segments));
            }
            Ok(state)
        }
        "vinyl_boolean" => {
            // Required: without an operation there is nothing to apply. Name the error instead of
            // defaulting to a union the caller may not have wanted.
            let op_name = args
                .get("operation")
                .and_then(Value::as_str)
                .ok_or("missing `operation`")?;
            // Accept the sign-shop vocabulary as well as the engine's: "weld" and "trim" are what
            // sign shops call union and difference, and refusing them would make the
            // tool harder to use than the thing it replaces.
            let op = match op_name {
                "union" | "weld" => BooleanOp::Union,
                "difference" | "trim" | "punch" => BooleanOp::Difference,
                "intersect" => BooleanOp::Intersect,
                "xor" | "exclude" => BooleanOp::Xor,
                other => {
                    return Err(format!(
                        "unknown operation {other:?}; expected union, difference, intersect or xor"
                    ));
                }
            };
            let fill = match args.get("fill_mode").and_then(Value::as_str) {
                Some("evenodd") | Some("even_odd") | Some("even-odd") => FillMode::EvenOdd,
                // NonZero is the default because glyph outlines and vector editors fill by it. An
                // absent or unrecognised value falls back rather than failing: a caller who only
                // cares about the operation should not be blocked by the fill rule.
                _ => FillMode::NonZero,
            };
            // The clip rectangle is all-or-nothing. Three sides without the fourth is a mistake,
            // not a default, so it is named rather than silently completed from zero.
            let clip_keys = ["clip_x", "clip_y", "clip_width", "clip_height"];
            let present: Vec<Option<f64>> = clip_keys
                .iter()
                .map(|k| args.get(*k).and_then(Value::as_f64))
                .collect();
            let given = present.iter().filter(|v| v.is_some()).count();
            let clip = match given {
                0 => None,
                4 => Some((
                    present[0].unwrap_or(0.0),
                    present[1].unwrap_or(0.0),
                    present[2].unwrap_or(0.0),
                    present[3].unwrap_or(0.0),
                )),
                _ => {
                    return Err(
                        "clip needs all of clip_x, clip_y, clip_width and clip_height, or none"
                            .into(),
                    );
                }
            };
            let (out_count, passed_open, skipped) = app.boolean_design(op, fill, clip);
            let mut state = state_json(app);
            if let Some(obj) = state.as_object_mut() {
                obj.insert("boolean_operation".into(), json!(op_name));
                obj.insert("boolean_contours".into(), json!(out_count));
                obj.insert("boolean_passed_through_open".into(), json!(passed_open));
                obj.insert("boolean_skipped_degenerate".into(), json!(skipped));
                obj.insert("boolean_clip_applied".into(), json!(clip.is_some()));
            }
            Ok(state)
        }
        "vinyl_profile" => {
            // Only the keys actually present change. A JSON null clears the setting back to
            // the cutter's own default, which is why the Option fields read `is_some`/`None`
            // rather than a plain f64: "not set" is a real state, not zero.
            let mut changed: Vec<&str> = Vec::new();
            macro_rules! set_opt {
                ($key:literal, $field:expr) => {
                    if let Some(v) = args.get($key) {
                        if v.is_null() {
                            $field = None;
                        } else if let Some(n) = v.as_f64() {
                            $field = Some(n);
                        }
                        changed.push($key);
                    }
                };
            }
            set_opt!("force", app.profile.force);
            set_opt!("speed", app.profile.speed);
            set_opt!("up_speed", app.profile.up_speed);
            set_opt!("max_x_mm", app.profile.max_x_mm);
            set_opt!("max_y_mm", app.profile.max_y_mm);
            if let Some(v) = args.get("overcut_mm").and_then(Value::as_f64) {
                app.profile.overcut_mm = v.clamp(0.0, 5.0);
                changed.push("overcut_mm");
            }
            if let Some(v) = args.get("blade_offset_mm").and_then(Value::as_f64) {
                app.profile.blade_offset_mm = v.clamp(0.0, 2.0);
                changed.push("blade_offset_mm");
            }
            if let Some(v) = args.get("min_line_mm").and_then(Value::as_f64) {
                app.profile.min_line_mm = v.clamp(0.0, 5.0);
                changed.push("min_line_mm");
            }
            if let Some(v) = args.get("linemerge_tol_mm").and_then(Value::as_f64) {
                app.profile.linemerge_tol_mm = v.clamp(0.0, 5.0);
                changed.push("linemerge_tol_mm");
            }
            if let Some(v) = args.get("repeat_passes").and_then(Value::as_u64) {
                app.profile.repeat_passes = (v as u32).clamp(1, 10);
                changed.push("repeat_passes");
            }
            // Tool selection. Naming a preset that does not exist is an error rather than a
            // silent no-op: an operator who asks for "60 degree blade" and gets a 45 degree
            // trail without being told is cutting the wrong geometry.
            if let Some(v) = args.get("active_tool") {
                if v.is_null() {
                    app.profile.active_tool = None;
                    changed.push("active_tool");
                } else if let Some(name) = v.as_str() {
                    let known = app
                        .profile
                        .tool_presets
                        .iter()
                        .any(|p| p.name.eq_ignore_ascii_case(name));
                    if !known && !app.profile.tool_presets.is_empty() {
                        let available: Vec<&str> = app
                            .profile
                            .tool_presets
                            .iter()
                            .map(|p| p.name.as_str())
                            .collect();
                        return Err(format!(
                            "no tool preset named {name:?}; available: {}",
                            available.join(", ")
                        ));
                    }
                    app.profile.active_tool = Some(name.to_string());
                    changed.push("active_tool");
                }
            }
            // Replacing the whole preset list. Each entry needs a name; the rest is optional.
            // A profile with no presets is legitimate (flat settings only), so an empty array
            // clears the list rather than being rejected.
            if let Some(v) = args.get("tool_presets") {
                if let Some(arr) = v.as_array() {
                    let mut presets: Vec<ToolPreset> = Vec::with_capacity(arr.len());
                    for (i, item) in arr.iter().enumerate() {
                        let Some(name) = item.get("name").and_then(Value::as_str) else {
                            return Err(format!("tool_presets[{i}] is missing a name"));
                        };
                        presets.push(ToolPreset {
                            name: name.to_string(),
                            force: item.get("force").and_then(Value::as_f64),
                            speed: item.get("speed").and_then(Value::as_f64),
                            blade_offset_mm: item
                                .get("blade_offset_mm")
                                .and_then(Value::as_f64)
                                .unwrap_or(0.0)
                                .clamp(0.0, 2.0),
                            repeat_passes: item
                                .get("repeat_passes")
                                .and_then(Value::as_u64)
                                .unwrap_or(0)
                                .min(10) as u32,
                        });
                    }
                    app.profile.tool_presets = presets;
                    changed.push("tool_presets");
                } else if v.is_null() {
                    app.profile.tool_presets.clear();
                    changed.push("tool_presets");
                }
            }
            let mut out = state_json(app);
            out["changed"] = json!(changed);
            Ok(out)
        }
        "vinyl_emit" => {
            // An explicit overcut in the call wins over the profile's stored value; without
            // this an agent could read the setting but never change it.
            if let Some(v) = args.get("overcut_mm").and_then(Value::as_f64) {
                app.profile.overcut_mm = v.clamp(0.0, 5.0);
            }
            let pen_up = args.get("pen_up").and_then(Value::as_bool).unwrap_or(app.pen_up_only);
            // Tiling changes the shape of the answer: one stream per tile, each rebased to
            // its own origin. Returning a single concatenated blob would hide the tile count
            // and the per-tile origin the operator needs to reload material correctly.
            if app.tile_opts.is_some() {
                let tiles = app.emit_tiled(pen_up)?;
                let dir = args.get("path").and_then(Value::as_str).map(Path::new);
                let mut rows: Vec<Value> = Vec::with_capacity(tiles.len());
                for (tile, code) in &tiles {
                    let mut row = json!({
                        "index": [tile.index.0, tile.index.1],
                        "origin_mm": [tile.origin.x, tile.origin.y],
                        "bytes": code.len(),
                        "preview": code.chars().take(120).collect::<String>(),
                    });
                    if let Some(dir) = dir {
                        let stem = dir
                            .file_stem()
                            .map(|s| s.to_string_lossy().to_string())
                            .unwrap_or_else(|| "tile".into());
                        let ext = dir
                            .extension()
                            .map(|s| s.to_string_lossy().to_string())
                            .unwrap_or_else(|| "dmpl".into());
                        let name = format!("{stem}_{}_{}.{ext}", tile.index.0, tile.index.1);
                        let p = dir.with_file_name(name);
                        fs::write(&p, code).map_err(|e| format!("{}: {e}", p.display()))?;
                        row["path"] = json!(p.display().to_string());
                    }
                    rows.push(row);
                }
                return Ok(json!({
                    "tiled": true,
                    "tile_count": rows.len(),
                    "tiles": rows,
                    "state": state_json(app),
                }));
            }
            let code = app.emit(pen_up)?;
            let shift = app.placement_shift().unwrap_or(Vec2::ZERO);
            // The shift is applied to the emitted code only; `state` still describes the
            // design as placed by the user. Report where the geometry actually cut so a
            // caller never mistakes the pre-shift bbox for the machine coordinates.
            let placed = bbox_of(&app.transformed());
            let mut out = json!({
                "bytes": code.len(),
                "pen_up": pen_up,
                // Non-zero when the design sat off the material and was slid back on
                // so the head never drives past the vinyl edge.
                "shift_mm": [shift.x, shift.y],
                "placed_bbox_mm": [
                    placed.0.x + shift.x,
                    placed.0.y + shift.y,
                    placed.1.x + shift.x,
                    placed.1.y + shift.y
                ],
                "preview": code.chars().take(240).collect::<String>(),
                "state": state_json(app),
            });
            if let Some(path) = args.get("path").and_then(Value::as_str) {
                fs::write(path, &code).map_err(|e| format!("{path}: {e}"))?;
                out["path"] = json!(path);
            }
            Ok(out)
        }
        "vinyl_send" => {
            if let Some(v) = args.get("pen_up").and_then(Value::as_bool) {
                app.pen_up_only = v;
            }
            app.send_design();
            wait_for_job(app);
            Ok(json!({"status": app.status, "log": app.log}))
        }
        "vinyl_jog" => {
            let dx = args.get("dx").and_then(Value::as_f64).unwrap_or(0.0);
            let dy = args.get("dy").and_then(Value::as_f64).unwrap_or(0.0);
            app.jog(dx, dy);
            wait_for_job(app);
            Ok(json!({"pos_mm": [app.pos_mm.x, app.pos_mm.y], "status": app.status}))
        }
        "vinyl_home" => {
            app.home();
            wait_for_job(app);
            Ok(json!({"pos_mm": [app.pos_mm.x, app.pos_mm.y], "status": app.status}))
        }
        "vinyl_test_square" => {
            app.send_test_square();
            wait_for_job(app);
            Ok(json!({"status": app.status}))
        }
        "vinyl_pen_up" => {
            app.pen_up();
            wait_for_job(app);
            Ok(json!({"status": app.status, "pen": "up", "pos_mm": [app.pos_mm.x, app.pos_mm.y]}))
        }
        "vinyl_pen_down" => {
            app.pen_down();
            wait_for_job(app);
            Ok(json!({
                "status": app.status,
                "pen": "down",
                "pos_mm": [app.pos_mm.x, app.pos_mm.y],
                "warning": "the blade stays down until vinyl_pen_up or vinyl_home"
            }))
        }
        "vinyl_pause" => {
            app.pause_send();
            Ok(json!({
                "paused": app.send_paused(),
                "status": app.status,
                "note": "the cutter stops once its own buffer drains; nothing is lost"
            }))
        }
        "vinyl_resume" => {
            app.resume_send();
            Ok(json!({"paused": app.send_paused(), "status": app.status}))
        }
        "vinyl_cancel" => {
            app.cancel_send();
            Ok(json!({
                "paused": app.send_paused(),
                "status": app.status,
                "warning": "the blade is still down where the last command left it — send vinyl_pen_up or vinyl_home"
            }))
        }
        "vinyl_marks" => {
            // `enabled: false` clears marks; omitting it leaves the current setting alone.
            match args.get("enabled").and_then(Value::as_bool) {
                Some(false) => {
                    app.marks = None;
                    app.log("registration marks off");
                }
                _ => {
                    let mut cfg = app
                        .marks
                        .clone()
                        .unwrap_or_else(solidcraft_cam::marks::MarkConfig::default);
                    if let Some(style) = args.get("style").and_then(Value::as_str) {
                        let size = args.get("size_mm").and_then(Value::as_f64).unwrap_or(5.0);
                        cfg.style = match style {
                            "cross" => solidcraft_cam::marks::MarkStyle::Cross { size_mm: size },
                            "circle_cross" => {
                                let segments = args
                                    .get("segments")
                                    .and_then(Value::as_u64)
                                    .unwrap_or(16)
                                    .clamp(4, 128) as usize;
                                solidcraft_cam::marks::MarkStyle::CircleCross {
                                    diameter_mm: size,
                                    segments,
                                }
                            }
                            _ => solidcraft_cam::marks::MarkStyle::CornerL { size_mm: size },
                        };
                    } else if let Some(size) = args.get("size_mm").and_then(Value::as_f64) {
                        // Size without a style keeps the current style but resizes it.
                        cfg.style = match cfg.style {
                            solidcraft_cam::marks::MarkStyle::CornerL { .. } => {
                                solidcraft_cam::marks::MarkStyle::CornerL { size_mm: size }
                            }
                            solidcraft_cam::marks::MarkStyle::Cross { .. } => {
                                solidcraft_cam::marks::MarkStyle::Cross { size_mm: size }
                            }
                            solidcraft_cam::marks::MarkStyle::CircleCross { segments, .. } => {
                                solidcraft_cam::marks::MarkStyle::CircleCross {
                                    diameter_mm: size,
                                    segments,
                                }
                            }
                        };
                    }
                    if let Some(p) = args.get("placement").and_then(Value::as_str) {
                        cfg.placement = match p {
                            "edge_midpoints" => solidcraft_cam::marks::MarkPlacement::EdgeMidpoints,
                            "centre" | "center" => solidcraft_cam::marks::MarkPlacement::Centre,
                            _ => solidcraft_cam::marks::MarkPlacement::Corners,
                        };
                    }
                    if let Some(m) = args.get("margin_mm").and_then(Value::as_f64) {
                        cfg.margin_mm = m.clamp(0.0, 100.0);
                    }
                    app.marks = Some(cfg);
                    app.log("registration marks on");
                }
            }
            Ok(state_json(app))
        }
        "vinyl_weed_box" => {
            match args.get("enabled").and_then(Value::as_bool) {
                Some(false) => {
                    app.weed_box_mm = None;
                    app.log("weed box off");
                }
                _ => {
                    let m = args
                        .get("margin_mm")
                        .and_then(Value::as_f64)
                        .unwrap_or(3.0)
                        .clamp(0.0, 100.0);
                    app.weed_box_mm = Some(m);
                    app.log(format!("weed box on at {m:.1} mm"));
                }
            }
            Ok(state_json(app))
        }
        "vinyl_lift_marks" => {
            match args.get("enabled").and_then(Value::as_bool) {
                Some(false) => {
                    app.lift_marks_mm = None;
                    app.log("lift marks off");
                }
                _ => {
                    let s = args
                        .get("size_mm")
                        .and_then(Value::as_f64)
                        .unwrap_or(2.0)
                        .clamp(0.1, 20.0);
                    app.lift_marks_mm = Some(s);
                    app.log(format!("lift marks on at {s:.1} mm"));
                }
            }
            Ok(state_json(app))
        }
        "vinyl_tile" => {
            match args.get("enabled").and_then(Value::as_bool) {
                Some(false) => {
                    app.tile_opts = None;
                    app.log("tiling off");
                }
                _ => {
                    let mut opts = app
                        .tile_opts
                        .clone()
                        .unwrap_or_else(solidcraft_cam::tile::TileOptions::default);
                    if let Some(v) = args.get("tile_w").and_then(Value::as_f64) {
                        opts.tile_w = v.clamp(10.0, 10000.0);
                    }
                    if let Some(v) = args.get("tile_h").and_then(Value::as_f64) {
                        opts.tile_h = v.clamp(10.0, 10000.0);
                    }
                    if let Some(v) = args.get("overlap_mm").and_then(Value::as_f64) {
                        opts.overlap_mm = v.clamp(0.0, 20.0);
                    }
                    if let Some(v) = args.get("box_around_tile").and_then(Value::as_bool) {
                        opts.box_around_tile = v;
                    }
                    if let Some(v) = args.get("box_margin_mm").and_then(Value::as_f64) {
                        opts.box_margin_mm = v.clamp(0.0, 20.0);
                    }
                    app.tile_opts = Some(opts);
                    app.log("tiling on");
                }
            }
            Ok(state_json(app))
        }
        "vinyl_job_add" => {
            let polys = app.transformed();
            if polys.is_empty() {
                return Err("nothing to queue: load a design first".into());
            }
            let colors = app.transformed_colors();
            let base_name = args
                .get("name")
                .and_then(Value::as_str)
                .map(|s| s.to_string())
                .unwrap_or_else(|| format!("job_{}", app.jobs.len() + 1));
            let explicit_color = args
                .get("color")
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string();

            // A cutter holds one vinyl colour at a time, so a design that carries several
            // colours must become several jobs: one per colour. A single-colour design (or one
            // whose colours the operator overrode with an explicit `color`) stays as one job.
            let has_multiple_colors = {
                let mut seen: HashSet<&Option<String>> = HashSet::new();
                for c in &colors {
                    seen.insert(c);
                }
                seen.len() > 1
            };
            let should_split = explicit_color.is_empty() && has_multiple_colors;

            let mut queued_names: Vec<String> = Vec::new();
            if should_split {
                // Group geometry by colour, preserving document order within each group.
                let mut groups: BTreeMap<Option<String>, Vec<Vec<Vec2>>> = BTreeMap::new();
                for (i, poly) in polys.into_iter().enumerate() {
                    let c = colors.get(i).cloned().flatten();
                    groups.entry(c).or_default().push(poly);
                }
                for (c_opt, group_polys) in groups {
                    let label = c_opt.unwrap_or_else(|| "unlabelled".into());
                    let name = format!("{base_name} [{label}]");
                    let b = bbox_of(&group_polys);
                    app.jobs.push(Job {
                        name: name.clone(),
                        polylines: group_polys,
                        color: label.clone(),
                    });
                    app.log(format!(
                        "queued job {name} ({:.1} x {:.1} mm); {} job(s) in the batch",
                        b.1.x - b.0.x,
                        b.1.y - b.0.y,
                        app.jobs.len()
                    ));
                    queued_names.push(name);
                }
            } else {
                // Single job: either an explicit colour override or an unlabelled design.
                let b = bbox_of(&polys);
                app.jobs.push(Job {
                    name: base_name.clone(),
                    polylines: polys,
                    color: explicit_color.clone(),
                });
                app.log(format!(
                    "queued job {base_name}{} ({:.1} x {:.1} mm); {} job(s) in the batch",
                    if explicit_color.is_empty() {
                        String::new()
                    } else {
                        format!(" [{}]", explicit_color)
                    },
                    b.1.x - b.0.x,
                    b.1.y - b.0.y,
                    app.jobs.len()
                ));
                queued_names.push(base_name);
            }

            let mut out = state_json(app);
            // `queued` stays a single string for callers written against the old contract;
            // `queued_all` is the full list, which matters when one design split into several
            // colour jobs.
            out["queued"] = json!(queued_names.first().cloned().unwrap_or_default());
            out["queued_all"] = json!(queued_names);
            Ok(out)
        }
        "vinyl_job_list" => {
            let jobs: Vec<Value> = app
                .jobs
                .iter()
                .map(|j| {
                    let b = j.bbox();
                    json!({
                        "name": j.name,
                        "color": j.color,
                        "width_mm": b.1.x - b.0.x,
                        "height_mm": b.1.y - b.0.y,
                        "paths": j.polylines.len(),
                    })
                })
                .collect();
            Ok(json!({"count": jobs.len(), "jobs": jobs}))
        }
        "vinyl_job_clear" => {
            let n = app.jobs.len();
            app.jobs.clear();
            app.log(format!("cleared {n} queued job(s)"));
            Ok(json!({"cleared": n, "state": state_json(app)}))
        }
        "vinyl_nest" => {
            let gap = args
                .get("gap_mm")
                .and_then(Value::as_f64)
                .unwrap_or(2.0)
                .clamp(0.0, 20.0);
            let allow_rotation = args
                .get("allow_rotation")
                .and_then(Value::as_bool)
                .unwrap_or(true);
            let pen_up = args
                .get("pen_up")
                .and_then(Value::as_bool)
                .unwrap_or(app.pen_up_only);
            let groups = app.emit_nested(pen_up, gap, allow_rotation)?;
            // One command stream per colour group. Roll nesting packs only the
            // Cutfiles that share a colour into one queue, because a cutter holds one vinyl
            // colour at a time. Each group is reported separately so the operator knows how
            // much vinyl each colour consumes and where each part sits.
            let dir = args.get("path").and_then(Value::as_str).map(Path::new);
            let stem = dir
                .and_then(|d| d.file_stem().map(|s| s.to_string_lossy().to_string()))
                .unwrap_or_else(|| "nest".into());
            let ext = dir
                .and_then(|d| d.extension().map(|s| s.to_string_lossy().to_string()))
                .unwrap_or_else(|| "dmpl".into());
            let mut rows: Vec<Value> = Vec::with_capacity(groups.len());
            let mut total_placed = 0usize;
            let mut total_unplaced = 0usize;
            let mut total_length_mm = 0.0f64;
            for g in &groups {
                total_placed += g.placed.len();
                total_unplaced += g.unplaced.len();
                // Colours are cut as separate runs, so the vinyl consumed is the sum of the
                // runs, not the longest one. Loading the roll once per colour means each run
                // starts fresh.
                total_length_mm += g.used_length_mm;
                let placements: Vec<Value> = g
                    .placed
                    .iter()
                    .map(|(name, origin, rotated)| {
                        json!({
                            "name": name,
                            "origin_mm": [origin.x, origin.y],
                            "rotated": rotated,
                        })
                    })
                    .collect();
                let mut row = json!({
                    "color": g.color,
                    "placed_count": g.placed.len(),
                    "unplaced_count": g.unplaced.len(),
                    "used_length_mm": g.used_length_mm,
                    "placements": placements,
                    "unplaced": g.unplaced,
                    "bytes": g.code.len(),
                    "preview": g.code.chars().take(120).collect::<String>(),
                });
                if let Some(dir) = dir {
                    // One file per colour so each run can be sent to the cutter on its own
                    // vinyl. A single-colour batch keeps the caller's exact path; multiple
                    // colours get a suffix so no run overwrites another.
                    let name = if groups.len() == 1 && g.color.is_empty() {
                        format!("{stem}.{ext}")
                    } else {
                        let tag = sanitize_filename(&g.color);
                        format!("{stem}_{tag}.{ext}")
                    };
                    let p = dir.with_file_name(name);
                    fs::write(&p, &g.code).map_err(|e| format!("{}: {e}", p.display()))?;
                    row["path"] = json!(p.display().to_string());
                }
                rows.push(row);
            }
            Ok(json!({
                "group_count": groups.len(),
                "placed_count": total_placed,
                "unplaced_count": total_unplaced,
                "total_used_length_mm": total_length_mm,
                "roll_width_mm": app.roll_width_mm,
                "groups": rows,
                "state": state_json(app),
            }))
        }
        "vinyl_ports" => {
            let ports: Vec<Value> = app
                .ports
                .iter()
                .map(|p| {
                    json!({
                        "name": p.name,
                        "label": p.label,
                        "usb": p.usb.map(|(v, d)| format!("{v:04X}:{d:04X}")),
                        "cutter": p.looks_like_cutter(),
                        "active": p.name == app.port_name,
                    })
                })
                .collect();
            Ok(json!({"ports": ports, "active": app.port_name}))
        }
        "vinyl_set_port" => {
            let port = args
                .get("port")
                .and_then(Value::as_str)
                .ok_or_else(|| "vinyl_set_port needs `port`".to_string())?;
            let baud = args.get("baud").and_then(Value::as_u64).map(|b| b as u32);
            // Rescan now: the cutter verdict must reflect what is plugged in at the moment of
            // the call, not what was plugged in when the app started.
            app.ports = scan_ports();
            let known = app.ports.iter().find(|p| p.name == port).cloned();
            let cutter = known.as_ref().map(|p| p.looks_like_cutter()).unwrap_or(false);
            app.port_name = port.to_string();
            if let Some(b) = baud {
                app.baud = b;
            }
            if let Some(s) = &mut app.profile.serial {
                s.port = port.to_string();
                if let Some(b) = baud {
                    s.baud = b;
                }
            }
            let msg = match &known {
                Some(p) if cutter => {
                    format!("port set to {} ({}) — cutter bridge", p.name, p.label)
                }
                Some(p) => format!(
                    "port set to {} ({}) — WARNING: not a known cutter bridge",
                    p.name, p.label
                ),
                None => format!(
                    "port set to {port} — WARNING: not in the scanned port list (may be a phantom device)"
                ),
            };
            app.log(msg.clone());
            Ok(json!({"port": app.port_name, "baud": app.baud, "cutter": cutter, "status": msg}))
        }
        "vinyl_align" => {
            let design = points_from_json(args.get("design_points"))?;
            let machine = points_from_json(args.get("machine_points"))?;
            let xf: Similarity = solve_alignment(&design, &machine)?;
            // The app's transformed() rotates and scales about the design's bounding-box
            // centre, so the solver's translation must be re-expressed as an offset from
            // that same centre. Otherwise the design lands in the right place but swings
            // around the wrong pivot on the next rotation.
            let center = (app.raw_bbox.0 + app.raw_bbox.1) * 0.5;
            let a = xf.a;
            let b = xf.b;
            app.scale = xf.scale();
            app.rotation = xf.rotation_rad();
            app.offset = Vec2::new(
                xf.tx - center.x + a * center.x - b * center.y,
                xf.ty - center.y + b * center.x + a * center.y,
            );
            app.log(format!(
                "Alignment applied from {} marks: scale {:.4}, rotation {:.2}°, offset ({:.2}, {:.2})",
                design.len(),
                app.scale,
                app.rotation.to_degrees(),
                app.offset.x,
                app.offset.y
            ));
            Ok(json!({
                "ok": true,
                "marks": design.len(),
                "scale": app.scale,
                "rotation_deg": app.rotation.to_degrees(),
                "translation_mm": [xf.tx, xf.ty],
                "offset_mm": [app.offset.x, app.offset.y],
                "status": "alignment applied — the design now lands on the observed marks",
            }))
        }
        "vinyl_quit" => {
            // Flag it and let whichever loop is running act on it: the GUI closes its window,
            // the headless server breaks its read loop. Doing the exit here would strand the
            // caller without a reply.
            QUIT_REQUESTED.store(true, Ordering::SeqCst);
            Ok(json!({"ok": true, "message": "quit requested"}))
        }
        other => Err(format!("unknown tool `{other}`")),
    }
}

/// Serve MCP-shaped JSON-RPC 2.0 over stdin/stdout until input closes.
/// Diagnostics go to stderr so stdout carries protocol messages only.
fn run_mcp(mut app: VinylApp) -> std::io::Result<()> {
    eprintln!("vinylcraft mcp: ready ({} tools)", vinyl_tools().len());
    let stdin = std::io::stdin();
    let mut input = stdin.lock();
    let stdout = std::io::stdout();
    let mut output = stdout.lock();
    let mut line = String::new();
    loop {
        line.clear();
        if input.read_line(&mut line)? == 0 {
            break;
        }
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let msg: Value = match serde_json::from_str(trimmed) {
            Ok(v) => v,
            Err(e) => {
                writeln!(output, "{}", rpc_error(Value::Null, -32700, format!("parse error: {e}")))?;
                output.flush()?;
                continue;
            }
        };
        let Some(id) = msg.get("id").cloned() else {
            // A notification (initialized, cancelled): no reply.
            continue;
        };
        let method = msg.get("method").and_then(Value::as_str).unwrap_or("");
        let params = msg.get("params").cloned().unwrap_or(Value::Null);
        let reply: Result<Value, (i64, String)> = match method {
            "initialize" => {
                let v = params
                    .get("protocolVersion")
                    .and_then(Value::as_str)
                    .unwrap_or("2025-06-18");
                Ok(json!({
                    "protocolVersion": v,
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": "vinylcraft", "version": env!("CARGO_PKG_VERSION")}
                }))
            }
            "ping" => Ok(json!({})),
            "tools/list" => Ok(json!({"tools": vinyl_tools()})),
            "tools/call" => {
                let name = params.get("name").and_then(Value::as_str).unwrap_or("");
                let arguments = params.get("arguments").cloned().unwrap_or(json!({}));
                match call_vinyl_tool(&mut app, name, &arguments) {
                    Ok(v) => Ok(json!({"content": [{"type": "text", "text": v.to_string()}]})),
                    Err(e) => Ok(json!({
                        "content": [{"type": "text", "text": format!("error: {e}")}],
                        "isError": true
                    })),
                }
            }
            other => Err((-32601, format!("unknown method `{other}`"))),
        };
        match reply {
            Ok(result) => writeln!(output, "{}", json!({"jsonrpc": "2.0", "id": id, "result": result}))?,
            Err((code, message)) => writeln!(output, "{}", rpc_error(id, code, message))?,
        }
        output.flush()?;
        // `vinyl_quit` sets the flag; a headless session has no window to close, so the
        // server's own loop is what ends.
        if QUIT_REQUESTED.load(Ordering::SeqCst) {
            break;
        }
    }
    Ok(())
}

/// Serve control requests on 127.0.0.1. Binds the default port when free, otherwise an
/// ephemeral one; the port is reported in the app log so a client can find it.
fn spawn_control_server(tx: mpsc::Sender<CtrlMsg>, ctx: egui::Context) -> std::io::Result<u16> {
    let listener = TcpListener::bind(("127.0.0.1", CONTROL_PORT_DEFAULT))
        .or_else(|_| TcpListener::bind(("127.0.0.1", 0)))?;
    let port = listener.local_addr()?.port();
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let tx = tx.clone();
            let ctx = ctx.clone();
            std::thread::spawn(move || {
                let _ = handle_control_client(stream, tx, ctx);
            });
        }
    });
    Ok(port)
}

/// One client connection: newline-delimited JSON-RPC, the same shape as the stdin server so a
/// client can speak either transport without changing its parser.
fn handle_control_client(
    stream: TcpStream,
    tx: mpsc::Sender<CtrlMsg>,
    ctx: egui::Context,
) -> std::io::Result<()> {
    let mut writer = stream.try_clone()?;
    let mut reader = std::io::BufReader::new(stream);
    let mut line = String::new();
    loop {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            break;
        }
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let msg: Value = match serde_json::from_str(trimmed) {
            Ok(v) => v,
            Err(e) => {
                writeln!(writer, "{}", rpc_error(Value::Null, -32700, format!("parse error: {e}")))?;
                writer.flush()?;
                continue;
            }
        };
        let id = msg.get("id").cloned().unwrap_or(Value::Null);
        let method = msg.get("method").and_then(Value::as_str).unwrap_or("");
        let params = msg.get("params").cloned().unwrap_or(Value::Null);
        let reply: Result<Value, (i64, String)> = match method {
            "initialize" => Ok(json!({
                "protocolVersion": "2025-06-18",
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "vinylcraft-gui", "version": env!("CARGO_PKG_VERSION")}
            })),
            "ping" => Ok(json!({})),
            // Closing is a flag flip plus a repaint request: the UI thread owns the window and
            // is the only place allowed to send the close command.
            "quit" => {
                QUIT_REQUESTED.store(true, Ordering::SeqCst);
                ctx.request_repaint();
                Ok(json!({"ok": true, "message": "closing"}))
            }
            "state" => {
                let (rtx, rrx) = mpsc::channel();
                let _ = tx.send(CtrlMsg::State(rtx));
                ctx.request_repaint();
                match rrx.recv_timeout(Duration::from_secs(5)) {
                    Ok(v) => Ok(v),
                    Err(_) => Err((-32603, "app did not respond".into())),
                }
            }
            "tools/list" => Ok(json!({"tools": vinyl_tools()})),
            "tools/call" => {
                let name = params.get("name").and_then(Value::as_str).unwrap_or("").to_string();
                let arguments = params.get("arguments").cloned().unwrap_or(json!({}));
                let (rtx, rrx) = mpsc::channel();
                let _ = tx.send(CtrlMsg::Call {
                    name: name.clone(),
                    args: arguments,
                    reply: rtx,
                });
                ctx.request_repaint();
                match rrx.recv_timeout(Duration::from_secs(30)) {
                    Ok(Ok(v)) => Ok(json!({"content": [{"type": "text", "text": v.to_string()}]})),
                    Ok(Err(e)) => Ok(json!({
                        "content": [{"type": "text", "text": format!("error: {e}")}],
                        "isError": true
                    })),
                    Err(_) => Err((-32603, format!("tool `{name}` timed out"))),
                }
            }
            other => Err((-32601, format!("unknown method `{other}`"))),
        };
        match reply {
            Ok(result) => writeln!(writer, "{}", json!({"jsonrpc": "2.0", "id": id, "result": result}))?,
            Err((code, message)) => writeln!(writer, "{}", rpc_error(id, code, message))?,
        }
        writer.flush()?;
    }
    Ok(())
}

/// Send one request to a running GUI's control socket and return its reply.
fn control_request(port: u16, req: Value) -> Result<Value, String> {
    let addr: std::net::SocketAddr = ([127, 0, 0, 1], port).into();
    let mut stream = TcpStream::connect_timeout(&addr, Duration::from_secs(2))
        .map_err(|e| format!("connect 127.0.0.1:{port}: {e}"))?;
    stream.set_read_timeout(Some(Duration::from_secs(10))).ok();
    stream.set_write_timeout(Some(Duration::from_secs(2))).ok();
    let mut line = req.to_string();
    line.push('\n');
    stream.write_all(line.as_bytes()).map_err(|e| e.to_string())?;
    stream.flush().ok();
    let mut reader = std::io::BufReader::new(stream);
    let mut reply = String::new();
    reader.read_line(&mut reply).map_err(|e| e.to_string())?;
    serde_json::from_str(reply.trim()).map_err(|e| format!("bad reply: {e}"))
}

/// Last resort when no control socket answers: kill by PID, and only after confirming the
/// process's ExecutablePath is *this* binary. This cannot flush state — it is the blunt path —
/// so it only runs after the graceful request fails.
///
/// Image-name matching is case-insensitive on Windows. This binary's former image name
/// collided with an unrelated program of the same name installed on this machine, so
/// `/IM <name>` would have killed that one too. It is `vinylcraft.exe` now. Even so,
/// identity is the executable path, never the image name.
fn quit_by_process_name() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(windows)]
    {
        let me = std::env::current_exe()
            .ok()
            .and_then(|p| p.canonicalize().ok());
        // tasklist cannot show a process's path; WMI can. Ask for both id and path so each
        // candidate is checked before anything is terminated.
        let query = "Get-CimInstance Win32_Process -Filter \"Name='vinylcraft.exe'\" | \
                     ForEach-Object { $_.ProcessId.ToString() + \"`t\" + $_.ExecutablePath }";
        let out = std::process::Command::new("powershell")
            .args(["-NoProfile", "-Command", query])
            .output()
            .map_err(|e| format!("powershell: {e}"))?;
        let text = String::from_utf8_lossy(&out.stdout);
        let mut killed = 0usize;
        for line in text.lines() {
            let Some((pid, path)) = line.trim().split_once('\t') else {
                continue;
            };
            let Ok(pid) = pid.trim().parse::<u32>() else {
                continue;
            };
            // The process running `--quit` is itself a vinylcraft.exe from this path; never
            // target it.
            if pid == std::process::id() {
                continue;
            }
            let same = match (&me, path.trim()) {
                (Some(m), p) if !p.is_empty() => std::path::Path::new(p)
                    .canonicalize()
                    .map(|q| &q == m)
                    .unwrap_or(false),
                _ => false,
            };
            if !same {
                eprintln!("skipping PID {pid} ({path}): not this executable");
                continue;
            }
            let status = std::process::Command::new("taskkill")
                .args(["/PID", &pid.to_string(), "/F"])
                .status()
                .map_err(|e| format!("taskkill: {e}"))?;
            if status.success() {
                println!("terminated vinylcraft.exe (PID {pid})");
                killed += 1;
            } else {
                return Err(format!("taskkill exited with {status}").into());
            }
        }
        if killed == 0 {
            return Err(
                "no running vinylcraft.exe matched this executable's path; close the window manually"
                    .into(),
            );
        }
        Ok(())
    }
    #[cfg(not(windows))]
    {
        // On Unix, match on this executable's full path rather than a bare name that other
        // software may also use.
        let me = std::env::current_exe()
            .ok()
            .and_then(|p| p.to_str().map(str::to_owned))
            .unwrap_or_else(|| "vinylcraft".into());
        let status = std::process::Command::new("pkill")
            .args(["-f", &me])
            .status()
            .map_err(|e| format!("pkill: {e}"))?;
        if status.success() {
            println!("terminated {me}");
            Ok(())
        } else {
            Err(format!("pkill exited with {status}").into())
        }
    }
}

/// Write bytes and read back whatever the device says.
///
/// The reply matters as much as the write. A cutter that accepted a job usually answers
/// with a status or error string; a silent port means either a write-only machine or the
/// wrong device entirely. Without this, "Sent N bytes" is indistinguishable from "shouted
/// into a disconnected cable", which is exactly the failure mode that looks like a
/// working app.
fn write_serial(port_name: &str, baud: u32, bytes: &[u8]) -> Result<(usize, String), String> {
    write_serial_ctl(port_name, baud, bytes, None)
}

/// Send `bytes` in small chunks, holding the stream while the operator has paused the job.
///
/// The cutter buffers a few hundred bytes to a few kilobytes, so withholding the rest of the
/// stream stops the carriage once that buffer drains. That is the only pause a write-only
/// protocol offers: there is no pause command in DMPL to send. Chunking also keeps a long
/// contour from being pushed into a small buffer in one blocking write.
fn write_serial_ctl(
    port_name: &str,
    baud: u32,
    bytes: &[u8],
    ctl: Option<&Arc<SendControl>>,
) -> Result<(usize, String), String> {
    use serialport::FlowControl;
    // Many vinyl cutters are 3-wire (TX/RX/GND) and never drive CTS. With hardware flow
    // control the driver buffers the whole job and then waits for a clear-to-send that never
    // arrives — the app logs "Sent N bytes" while the machine sits still. Prefer the
    // configured handshake, but fall back to none so a 3-wire cable still cuts.
    //
    // Fall back only when the port cannot be OPENED with hardware flow control. If the open
    // succeeds and the write fails partway, retrying from the start would re-send bytes that
    // already left the cable and cut part of the job twice.
    let mut port = match open_port(port_name, baud, FlowControl::Hardware) {
        Ok(p) => p,
        Err(first) => open_port(port_name, baud, FlowControl::None)
            .map_err(|e| format!("{first}; without handshake: {e}"))?,
    };
    let mut sent = 0usize;
    for chunk in bytes.chunks(64) {
        if let Some(c) = ctl {
            while c.pause.load(Ordering::Relaxed) {
                if c.cancel.load(Ordering::Relaxed) {
                    return Err("cancelled by operator".into());
                }
                std::thread::sleep(Duration::from_millis(25));
            }
            if c.cancel.load(Ordering::Relaxed) {
                return Err("cancelled by operator".into());
            }
        }
        port.write_all(chunk).map_err(|e| e.to_string())?;
        sent += chunk.len();
    }
    let _ = port.flush();
    // Long jobs are paced by hardware flow control; this is a courtesy pause so short
    // commands reach the wire before we start listening for an answer.
    std::thread::sleep(Duration::from_millis(250));
    port.set_timeout(Duration::from_millis(400))
        .map_err(|e| e.to_string())?;
    let mut reply = Vec::new();
    let mut buf = [0u8; 256];
    let deadline = std::time::Instant::now() + Duration::from_millis(600);
    while std::time::Instant::now() < deadline {
        match port.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                reply.extend_from_slice(&buf[..n]);
                if reply.len() >= 4096 {
                    break;
                }
            }
            // A read timeout is the normal end of a quiet machine, not an error.
            Err(_) => break,
        }
    }
    let text = String::from_utf8_lossy(&reply).to_string();
    Ok((sent, text))
}

/// Open the port with a given handshake. Separated from writing so the fallback can change
/// handshake without touching bytes that may already be on the wire.
fn open_port(
    port_name: &str,
    baud: u32,
    flow: serialport::FlowControl,
) -> Result<Box<dyn serialport::SerialPort>, String> {
    use serialport::{DataBits, Parity, StopBits};
    serialport::new(port_name, baud)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(flow)
        .timeout(Duration::from_secs(5))
        .open()
        .map_err(|e| e.to_string())
}

/// Join imported entities into continuous polylines, one polyline per contour.
///
/// An SVG `<path>` is a single contour, but `Geom2` carries it as a run of separate
/// entities: `M p0 L p1 L p2 Z` arrives as three `Line`s. Pushing each entity as its own
/// polyline makes every edge a separate toolpath, so the blade lifts and re-drops at every
/// vertex — a 10-point star emitted 10 `U`s and 10 `D`s instead of one contour. Join
/// entities whose endpoints meet, and flush the run as soon as it returns to its own start
/// so consecutive subpaths do not get glued together.
fn flatten_all(shapes: &[Geom2]) -> Vec<Vec<Vec2>> {
    // Millimetres. 1 µm is far below cutting precision and far above the rounding noise an
    // affine transform leaves on a vertex that two entities share. Matches the closure
    // tolerance `polylines_to_toolpaths` uses, so a joined run is also seen as closed.
    const JOIN_EPS: f64 = 1e-6;
    let mut out: Vec<Vec<Vec2>> = Vec::new();
    let mut chain: Vec<Vec2> = Vec::new();
    for g in shapes {
        // Flatten one entity into its own buffer so it can be appended to the run in order.
        let mut piece: Vec<Vec<Vec2>> = Vec::new();
        flatten(g, &mut piece);
        for poly in piece {
            if poly.len() < 2 {
                continue;
            }
            if chain.is_empty() {
                chain = poly;
                // A circle or ellipse arrives already closed; flush it rather than gluing the
                // next contour onto its start point.
                if closed_enough(&chain, JOIN_EPS) {
                    out.push(std::mem::take(&mut chain));
                }
                continue;
            }
            let end = chain[chain.len() - 1];
            if (end - poly[0]).len() < JOIN_EPS {
                // Shared vertex: drop the duplicate start and extend the run.
                chain.extend_from_slice(&poly[1..]);
            } else if (end - poly[poly.len() - 1]).len() < JOIN_EPS {
                // A clockwise arc is stored with its endpoints swapped so it can be
                // expressed counter-clockwise; its flattened run arrives reversed. Reverse
                // it back so the contour stays continuous and the corner is cut in the
                // direction the drawing went — same curve, opposite traversal.
                chain.extend(poly.iter().rev().skip(1).copied());
            } else if (chain[0] - poly[0]).len() < JOIN_EPS {
                // The run itself is backwards (it began with a reversed arc). Flip it and
                // join this piece onto its new end.
                chain.reverse();
                chain.extend_from_slice(&poly[1..]);
            } else {
                out.push(std::mem::take(&mut chain));
                chain = poly;
            }
            if closed_enough(&chain, JOIN_EPS) {
                out.push(std::mem::take(&mut chain));
            }
        }
    }
    if !chain.is_empty() {
        out.push(chain);
    }
    out
}

/// Like [`flatten_all`], but keeps each polyline's colour and never joins across a colour
/// change.
///
/// A multi-colour SVG holds several vinyl colours in one file. Chaining geometry is what turns
/// loose entities into cuttable contours, but a red contour must not be fused to a blue one:
/// the cutter holds one colour at a time, so fused geometry would be cut in whichever colour
/// happened to be loaded. This flushes the run whenever the colour changes and returns the
/// colour of each finished run in a parallel vector.
fn flatten_all_colored(
    shapes: &[Geom2],
    colors: &[Option<String>],
) -> (Vec<Vec<Vec2>>, Vec<Option<String>>) {
    const JOIN_EPS: f64 = 1e-6;
    let mut out: Vec<Vec<Vec2>> = Vec::new();
    let mut out_colors: Vec<Option<String>> = Vec::new();
    let mut chain: Vec<Vec2> = Vec::new();
    let mut chain_color: Option<String> = None;

    for (i, g) in shapes.iter().enumerate() {
        // Missing colour entries are treated as unlabelled rather than truncating the zip.
        let color = colors.get(i).cloned().flatten();
        if !chain.is_empty() && color != chain_color {
            out.push(std::mem::take(&mut chain));
            out_colors.push(chain_color.clone());
        }
        if chain.is_empty() {
            chain_color = color.clone();
        }
        // Flatten one entity into its own buffer so it can be appended to the run in order.
        let mut piece: Vec<Vec<Vec2>> = Vec::new();
        flatten(g, &mut piece);
        for poly in piece {
            if poly.len() < 2 {
                continue;
            }
            if chain.is_empty() {
                chain = poly;
                if closed_enough(&chain, JOIN_EPS) {
                    out.push(std::mem::take(&mut chain));
                    out_colors.push(chain_color.clone());
                }
                continue;
            }
            let end = chain[chain.len() - 1];
            if (end - poly[0]).len() < JOIN_EPS {
                chain.extend_from_slice(&poly[1..]);
            } else if (end - poly[poly.len() - 1]).len() < JOIN_EPS {
                chain.extend(poly.iter().rev().skip(1).copied());
            } else if (chain[0] - poly[0]).len() < JOIN_EPS {
                chain.reverse();
                chain.extend_from_slice(&poly[1..]);
            } else {
                out.push(std::mem::take(&mut chain));
                out_colors.push(chain_color.clone());
                chain = poly;
            }
            if closed_enough(&chain, JOIN_EPS) {
                out.push(std::mem::take(&mut chain));
                out_colors.push(chain_color.clone());
            }
        }
    }
    if !chain.is_empty() {
        out.push(chain);
        out_colors.push(chain_color);
    }
    (out, out_colors)
}

/// True when a run has enough points to be a contour and its ends meet.
fn closed_enough(poly: &[Vec2], eps: f64) -> bool {
    poly.len() > 2 && (poly[0] - poly[poly.len() - 1]).len() < eps
}

/// Map a sketch-crate `LinkGeom` onto the IO crate's `Geom2`, so the glyph outlines the sketch
/// font engine produces can be flattened by the same code that handles imported geometry.
///
/// The variants line up one-for-one except for the spline's `periodic` flag, which `Geom2` does
/// not carry. That flag only says whether a spline closes back on itself; a text glyph's spline
/// is always a finite segment of its outline, so dropping it changes nothing about the cut path.
fn link_to_geom2(g: &LinkGeom) -> Geom2 {
    match g {
        LinkGeom::Point(p) => Geom2::Point(*p),
        LinkGeom::Line(a, b) => Geom2::Line(*a, *b),
        LinkGeom::Circle(c, r) => Geom2::Circle(*c, *r),
        LinkGeom::Arc { c, a, b } => Geom2::Arc {
            c: *c,
            a: *a,
            b: *b,
        },
        LinkGeom::Conic { a, apex, b, rho } => Geom2::Conic {
            a: *a,
            apex: *apex,
            b: *b,
            rho: *rho,
        },
        LinkGeom::Ellipse { c, major, minor } => Geom2::Ellipse {
            c: *c,
            major: *major,
            minor: *minor,
        },
        LinkGeom::Spline {
            pts,
            control,
            degree,
            ..
        } => Geom2::Spline {
            pts: pts.clone(),
            control: *control,
            degree: *degree,
        },
    }
}

fn flatten(g: &Geom2, out: &mut Vec<Vec<Vec2>>) {
    match g {
        Geom2::Point(_) => {}
        Geom2::Line(a, b) => out.push(vec![*a, *b]),
        Geom2::Circle(c, r) => {
            let n = segments_for_radius(*r);
            let mut v = Vec::with_capacity(n + 1);
            for i in 0..=n {
                let t = i as f64 / n as f64 * std::f64::consts::TAU;
                v.push(Vec2::new(c.x + r * t.cos(), c.y + r * t.sin()));
            }
            out.push(v);
        }
        Geom2::Arc { c, a, b } => {
            let r = (*a - *c).len();
            if r < 1e-9 {
                return;
            }
            let a0 = (*a - *c).angle();
            let mut a1 = (*b - *c).angle();
            if a1 <= a0 {
                a1 += std::f64::consts::TAU;
            }
            let sweep = a1 - a0;
            let n = ((sweep.abs() / std::f64::consts::TAU) * 96.0).ceil().max(8.0) as usize;
            let mut v = Vec::with_capacity(n + 1);
            for i in 0..=n {
                let t = a0 + sweep * (i as f64 / n as f64);
                v.push(Vec2::new(c.x + r * t.cos(), c.y + r * t.sin()));
            }
            out.push(v);
        }
        Geom2::Ellipse { c, major, minor } => {
            let u = *major - *c;
            let a = u.len();
            if a < 1e-9 || *minor < 1e-9 {
                return;
            }
            let vdir = u.perp().normalized().unwrap_or(Vec2::Y) * *minor;
            let n = segments_for_radius(a.max(*minor));
            let mut v = Vec::with_capacity(n + 1);
            for i in 0..=n {
                let t = i as f64 / n as f64 * std::f64::consts::TAU;
                v.push(Vec2::new(c.x + u.x * t.cos() + vdir.x * t.sin(), c.y + u.y * t.cos() + vdir.y * t.sin()));
            }
            out.push(v);
        }
        Geom2::Spline { pts, control, .. } => {
            if pts.len() < 2 {
                return;
            }
            if !*control {
                out.push(pts.clone());
            } else {
                let mut v = Vec::new();
                for i in 0..pts.len() - 1 {
                    let p0 = if i == 0 { pts[0] } else { pts[i - 1] };
                    let p1 = pts[i];
                    let p2 = pts[i + 1];
                    let p3 = if i + 2 < pts.len() { pts[i + 2] } else { p2 };
                    for s in 0..16 {
                        let t = s as f64 / 16.0;
                        v.push(catmull_rom(p0, p1, p2, p3, t));
                    }
                }
                v.push(*pts.last().unwrap());
                out.push(v);
            }
        }
        Geom2::Conic { a, apex, b, rho } => {
            let n = 48;
            let mut v = Vec::with_capacity(n + 1);
            for i in 0..=n {
                let t = i as f64 / n as f64;
                v.push(rational_quad(*a, *apex, *b, *rho, t));
            }
            out.push(v);
        }
        Geom2::Text {
            text,
            at,
            height,
            angle,
        } => {
            // Resolve the string to glyph outlines with the sketch crate's font engine. A
            // glyph's outline and its counters (the holes in A, B, 8) arrive as separate chains,
            // so each is flattened as its own contour and the blade cuts the holes as separate
            // closed paths — which is what a vinyl cutter does with them anyway.
            match text_geometry(text, *at, *height, *angle) {
                Ok(outlines) => {
                    for outline in &outlines {
                        flatten(&link_to_geom2(outline), out);
                    }
                }
                Err(e) => {
                    // A text primitive the font engine rejects (a non-positive height, or more
                    // characters than it will set) must not abort the rest of the design: the
                    // remaining geometry still cuts. Report it so the missing lettering is
                    // visible rather than silently absent from the job.
                    eprintln!("vinylcraft: could not outline text {text:?}: {e}");
                }
            }
        }
    }
}

fn segments_for_radius(r: f64) -> usize {
    ((r.abs() * 2.0).max(16.0).min(256.0)) as usize
}

fn catmull_rom(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: f64) -> Vec2 {
    let t2 = t * t;
    let t3 = t2 * t;
    let a = p1 * 2.0;
    let b = (p2 - p0) * t;
    let c = (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2;
    let d = (p1 * 3.0 - p0 - p2 * 3.0 + p3) * t3;
    (a + b + c + d) * 0.5
}

fn rational_quad(a: Vec2, apex: Vec2, b: Vec2, rho: f64, t: f64) -> Vec2 {
    let mt = 1.0 - t;
    let w0 = mt * mt;
    let w1 = 2.0 * rho * mt * t;
    let w2 = t * t;
    let denom = w0 + w1 + w2;
    (a * w0 + apex * w1 + b * w2) / denom
}

fn bbox_of(polys: &[Vec<Vec2>]) -> (Vec2, Vec2) {
    let mut min = Vec2::new(f64::INFINITY, f64::INFINITY);
    let mut max = Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for poly in polys {
        for p in poly {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
        }
    }
    if !min.is_finite() || !max.is_finite() {
        (Vec2::ZERO, Vec2::ZERO)
    } else {
        (min, max)
    }
}

/// Turn a colour label into something safe to use as part of a file name.
///
/// Vinyl colour names reach here as operator-typed text ("Oracal 651 White", "red/blue"), so
/// they can carry spaces and path separators. A separator would make `fs::write` create or
/// target the wrong directory; whitespace makes an awkward file name. Everything that is not
/// ASCII alphanumeric becomes `_`, runs collapse, and an empty result falls back to "colour"
/// so the name is never empty.
fn sanitize_filename(color: &str) -> String {
    let mut out = String::with_capacity(color.len());
    let mut last_underscore = false;
    for ch in color.chars() {
        if ch.is_ascii_alphanumeric() {
            out.push(ch.to_ascii_lowercase());
            last_underscore = false;
        } else if !last_underscore {
            out.push('_');
            last_underscore = true;
        }
    }
    let trimmed = out.trim_matches('_').to_string();
    if trimmed.is_empty() {
        "colour".into()
    } else {
        trimmed
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

fn mm_to_screen(p: Vec2, rect: egui::Rect, pan: egui::Vec2, zoom: f32) -> egui::Pos2 {
    egui::pos2(
        rect.left() + pan.x + p.x as f32 * zoom,
        rect.top() + pan.y - p.y as f32 * zoom,
    )
}

fn screen_to_mm(p: egui::Pos2, rect: egui::Rect, pan: egui::Vec2, zoom: f32) -> Vec2 {
    Vec2::new(
        ((p.x - rect.left() - pan.x) / zoom) as f64,
        ((rect.top() + pan.y - p.y) / zoom) as f64,
    )
}
