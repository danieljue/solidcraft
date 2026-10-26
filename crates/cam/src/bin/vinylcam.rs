//! Minimal CAM CLI: read a machine profile, emit a DMPL/HPGL square.
//!
//! Usage:
//!   vinylcam <profile.json> <out.dmpl|out.hpgl> [--square <mm>]

use solidcraft_cam::geom::Toolpath;
use solidcraft_cam::post;
use solidcraft_cam::profile::MachineProfile;
use std::error::Error;
use std::fs;

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: vinylcam <profile.json> <out> [--square <mm>]");
        std::process::exit(2);
    }

    let profile: MachineProfile = serde_json::from_str(&fs::read_to_string(&args[1])?)?;
    let size = args
        .iter()
        .position(|a| a == "--square")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(10.0);

    let toolpath = Toolpath::rectangle(0.0, 0.0, size, size);
    let out = post::emit(&profile, &[toolpath])?;
    fs::write(&args[2], &out)?;
    println!("wrote {} bytes to {}", out.len(), args[2]);
    Ok(())
}
