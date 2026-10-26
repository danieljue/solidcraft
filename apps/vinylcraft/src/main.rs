//! VinylCraft: a 2D vector design + vinyl cutter control prototype.
//!
//! This is the "MANUFACTURE" vision as a standalone app first: a roll-aware
//! canvas, SVG/DXF import, transforms, and DMPL output to a US Cutter SC2.
//! The CAM crate stays UI-free so it can be lifted into SolidCraft's
//! MANUFACTURE tab later.

use eframe::egui;
use serde_json::json;
use std::fs;
use std::path::Path;

mod geometry;
mod tools;
mod serial;
mod jobs;
mod control;
mod app;
mod dispatch;
use geometry::*;
use control::*;
use app::VinylApp;
use dispatch::*;


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

