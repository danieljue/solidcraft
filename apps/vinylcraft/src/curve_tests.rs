//! Coverage that needs both crates at once: a curve read from a file must still be a
//! curve when it reaches a post-processor, and must flatten cleanly when the controller
//! has no arc word.
//!
//! These live in the app because it is the only crate that depends on both `solidcraft-io`
//! (which reads the file) and `solidcraft-cam` (which emits the job). A test in either
//! crate alone would have to fake the other half.

use solidcraft_cam::geom::Toolpath;
use solidcraft_cam::post::{Pen, dmpl, gcode};
use solidcraft_cam::profile::{MachineProfile, Protocol};
use solidcraft_io::{Geom2, read_svg};

const SVG: &str = r#"<svg xmlns="http://www.w3.org/2000/svg" width="40mm" height="40mm" viewBox="0 0 40 40">
  <path d="M 30 20 A 10 10 0 0 1 20 30" fill="none"/>
  <circle cx="20" cy="20" r="5" fill="none"/>
</svg>"#;

fn profile(protocol: Protocol) -> MachineProfile {
    MachineProfile {
        name: "curve test".into(),
        protocol,
        units_per_mm: 1.0,
        header: String::new(),
        footer: String::new(),
        overcut_mm: 0.0,
        ..MachineProfile::default()
    }
}

/// Import the test drawing as exact curves, the way a curve-aware caller would.
fn toolpaths() -> Vec<Toolpath> {
    read_svg(SVG)
        .expect("svg parses")
        .iter()
        .filter_map(|g| {
            let segments = g.to_segments();
            if segments.is_empty() {
                return None;
            }
            Some(Toolpath { segments, closed: matches!(g, Geom2::Circle(..)) })
        })
        .collect()
}

#[test]
fn an_svg_arc_reaches_gcode_as_an_arc() {
    let out = gcode::emit(&profile(Protocol::Gcode), &toolpaths());
    // One quarter arc from the path, two half arcs from the circle.
    assert_eq!(out.matches("G3 ").count(), 3, "expected three arcs in:\n{out}");
    assert!(!out.contains("G1 "), "nothing should be flattened for a controller that has G2/G3:\n{out}");
}

#[test]
fn the_same_toolpaths_flatten_for_a_dialect_without_arcs() {
    let out = profile(Protocol::Dmpl);
    let text = dmpl::emit_with_pen(&out, &toolpaths(), Pen::Down);
    assert!(!text.contains("AA"), "DM/PL has no arc word, so none may appear:\n{text}");
    // Two half circles plus a quarter arc: a real curve needs many chords, so if the arcs
    // had collapsed to their endpoints there would be only a handful of draws.
    let draws = text.matches('D').count();
    assert!(draws > 20, "a circle flattened to {draws} draws, expected many more");
}

#[test]
fn the_circle_is_two_arcs_and_not_a_polygon() {
    let tps = toolpaths();
    let circles: Vec<&Toolpath> = tps.iter().filter(|t| t.closed).collect();
    assert_eq!(circles.len(), 1, "the circle should be its own contour");
    assert_eq!(circles[0].segments.len(), 2, "a circle is two half arcs");
}
