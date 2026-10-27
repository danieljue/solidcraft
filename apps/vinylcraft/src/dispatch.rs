//! MCP dispatch: map a tool name and its JSON arguments onto the app's operations.
//!
//! The same dispatch serves the stdin `--mcp` server and, over the control socket, a running
//! GUI. Keeping it separate from the model makes the tool surface easy to read and test.

use serde_json::{Value, json};
use solidcraft_cam::align::{solve as solve_alignment, Similarity};
use solidcraft_cam::boolean::{BooleanOp, FillMode};
use solidcraft_cam::profile::ToolPreset;
use solidcraft_geom::Vec2;
use std::collections::{BTreeMap, HashSet};
use std::fs;
use std::io::{BufRead, Write};
use std::path::Path;
use std::sync::atomic::Ordering;
use std::time::Duration;

use crate::app::VinylApp;
use crate::geometry::*;
use crate::tools::*;
use crate::jobs::*;
use crate::control::*;
use crate::serial::scan_ports;

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

pub(crate) fn state_json(app: &VinylApp) -> Value {
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
    // How many edits can still be taken back or reapplied: the GUI shows these as the
    // enablement of Undo/Redo, and a caller needs them to know whether an edit is
    // recoverable. Set here rather than inline, for the same recursion-limit reason.
    state["undo_depth"] = json!(app.undo_stack.len());
    state["redo_depth"] = json!(app.redo_stack.len());
    state
}

pub(crate) fn call_vinyl_tool(app: &mut VinylApp, name: &str, args: &Value) -> Result<Value, String> {
    match name {
        "vinyl_state" => Ok(state_json(app)),
        "vinyl_demo" => {
            app.load_demo();
            Ok(state_json(app))
        }
        "vinyl_clear" => {
            app.push_undo();
            app.raw_polylines.clear();
            app.raw_colors.clear();
            app.raw_bbox = (Vec2::ZERO, Vec2::ZERO);
            app.needs_fit = true;
            app.log("Cleared design");
            Ok(state_json(app))
        }
        "vinyl_undo" => {
            let done = app.undo();
            Ok(json!({
                "undone": done,
                "undo_depth": app.undo_stack.len(),
                "redo_depth": app.redo_stack.len(),
                "paths": app.raw_polylines.len(),
                "status": if done { "reverted the last edit" } else { "nothing to undo" },
                "state": state_json(app),
            }))
        }
        "vinyl_redo" => {
            let done = app.redo();
            Ok(json!({
                "redone": done,
                "undo_depth": app.undo_stack.len(),
                "redo_depth": app.redo_stack.len(),
                "paths": app.raw_polylines.len(),
                "status": if done { "reapplied the last edit" } else { "nothing to redo" },
                "state": state_json(app),
            }))
        }
        "vinyl_export" => {
            let path = args.get("path").and_then(Value::as_str).ok_or("missing `path`")?;
            let format = args
                .get("format")
                .and_then(Value::as_str)
                .unwrap_or("svg")
                .to_ascii_lowercase();
            if format != "svg" {
                return Err(format!(
                    "unsupported format {format:?}; only \"svg\" is supported"
                ));
            }
            let svg = app.export_svg();
            fs::write(path, &svg).map_err(|e| format!("{path}: {e}"))?;
            // The file holds the transformed design, so report that box, not the raw one.
            let (lo, hi) = crate::geometry::bbox_of(&app.transformed());
            Ok(json!({
                "path": path,
                "format": "svg",
                "bytes": svg.len(),
                "paths": app.raw_polylines.len(),
                "bbox": [lo.x, lo.y, hi.x, hi.y],
            }))
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
pub(crate) fn run_mcp(mut app: VinylApp) -> std::io::Result<()> {
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
