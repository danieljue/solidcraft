//! MCP tool definitions for VinylCraft.
//!
//! The JSON schemas the app advertises to an MCP client, plus the small helpers that
//! build them. Pure data: no application state, no UI.

use serde_json::{Value, json};
use solidcraft_cam::geom::Point;

pub(crate) fn rpc_error(id: Value, code: i64, message: impl Into<String>) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message.into()}})
}

pub(crate) fn str_prop(desc: &str) -> Value {
    json!({"type": "string", "description": desc})
}

pub(crate) fn num_prop(desc: &str) -> Value {
    json!({"type": "number", "description": desc})
}

pub(crate) fn tool_schema(props: Value, required: &[&str]) -> Value {
    if required.is_empty() {
        json!({"type": "object", "properties": props})
    } else {
        json!({"type": "object", "properties": props, "required": required})
    }
}

pub(crate) fn vinyl_tools() -> Vec<Value> {
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
            "name": "vinyl_undo",
            "description": "Take back the last design edit (load, clear, offset, boolean, text, demo). Up to 32 steps. Does not touch the transform, the roll, or the queued jobs.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_redo",
            "description": "Reapply the last edit taken back by vinyl_undo. Any new edit clears the redo history.",
            "inputSchema": tool_schema(json!({}), &[])
        }),
        json!({
            "name": "vinyl_export",
            "description": "Write the current (transformed) design to disk as SVG, in millimetres, one polyline per contour with its stroke colour preserved. Curves are flattened. Use this to save an edit so it can be reopened instead of rebuilt.",
            "inputSchema": tool_schema(json!({
                "path": str_prop("Destination .svg path"),
                "format": str_prop("Output format; only \"svg\" is supported")
            }), &["path"])
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

/// Parse an array of `{x, y}` objects into points. A malformed point names its own index so a
/// caller who sent forty corners knows which one to fix.
pub(crate) fn points_from_json(v: Option<&Value>) -> Result<Vec<Point>, String> {
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
