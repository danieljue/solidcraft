//! Geometry and path helpers for the VinylCraft app.
//!
//! Pure 2D math with no UI or application state: polyline <-> toolpath conversion,
//! primitive shapes, flattening of `Geom2` entities (including text glyph outlines)
//! into polylines, contour chaining, and screen/mm coordinate mapping.
//!
//! Everything here is a free function so it can be unit-tested and reused without
//! constructing a `VinylApp`.

use solidcraft_cam::geom::{Point, Segment, Toolpath};
use solidcraft_geom::Vec2;
use solidcraft_io::Geom2;
use solidcraft_sketch::{text_geometry, LinkGeom};
use eframe::egui;

/// Convert plain polylines into cutter toolpaths: drop degenerate runs and mark a
/// contour closed when its last point coincides with its first.
pub(crate) fn polylines_to_toolpaths(polys: &[Vec<Vec2>]) -> Vec<Toolpath> {
    polys
        .iter()
        .filter(|p| p.len() >= 2)
        .map(|p| {
            let closed = p.len() > 2 && (p[0] - p[p.len() - 1]).len() < 1e-6;
            let segments = p
                .windows(2)
                .map(|w| Segment::Line {
                    a: Point::new(w[0].x, w[0].y),
                    b: Point::new(w[1].x, w[1].y),
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
pub(crate) fn toolpath_to_polyline(tp: &Toolpath) -> Vec<Vec2> {
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
pub(crate) fn rounded_rect_poly(w: f64, h: f64, r: f64, seg: usize) -> Vec<Vec2> {
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
pub(crate) fn circle_poly(c: Vec2, r: f64, n: usize) -> Vec<Vec2> {
    let n = n.max(8);
    let mut pts = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = i as f64 / n as f64 * std::f64::consts::TAU;
        pts.push(Vec2::new(c.x + r * t.cos(), c.y + r * t.sin()));
    }
    pts
}

/// Join imported entities into continuous polylines, one polyline per contour.
///
/// An SVG `<path>` is a single contour, but `Geom2` carries it as a run of separate
/// entities: `M p0 L p1 L p2 Z` arrives as three `Line`s. Pushing each entity as its own
/// polyline makes every edge a separate toolpath, so the blade lifts and re-drops at every
/// vertex — a 10-point star emitted 10 `U`s and 10 `D`s instead of one contour. Join
/// entities whose endpoints meet, and flush the run as soon as it returns to its own start
/// so consecutive subpaths do not get glued together.
pub(crate) fn flatten_all(shapes: &[Geom2]) -> Vec<Vec<Vec2>> {
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
pub(crate) fn flatten_all_colored(
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
pub(crate) fn closed_enough(poly: &[Vec2], eps: f64) -> bool {
    poly.len() > 2 && (poly[0] - poly[poly.len() - 1]).len() < eps
}

/// Map a sketch-crate `LinkGeom` onto the IO crate's `Geom2`, so the glyph outlines the sketch
/// font engine produces can be flattened by the same code that handles imported geometry.
///
/// The variants line up one-for-one except for the spline's `periodic` flag, which `Geom2` does
/// not carry. That flag only says whether a spline closes back on itself; a text glyph's spline
/// is always a finite segment of its outline, so dropping it changes nothing about the cut path.
pub(crate) fn link_to_geom2(g: &LinkGeom) -> Geom2 {
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

pub(crate) fn flatten(g: &Geom2, out: &mut Vec<Vec<Vec2>>) {
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

pub(crate) fn segments_for_radius(r: f64) -> usize {
    ((r.abs() * 2.0).max(16.0).min(256.0)) as usize
}

pub(crate) fn catmull_rom(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: f64) -> Vec2 {
    let t2 = t * t;
    let t3 = t2 * t;
    let a = p1 * 2.0;
    let b = (p2 - p0) * t;
    let c = (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2;
    let d = (p1 * 3.0 - p0 - p2 * 3.0 + p3) * t3;
    (a + b + c + d) * 0.5
}

pub(crate) fn rational_quad(a: Vec2, apex: Vec2, b: Vec2, rho: f64, t: f64) -> Vec2 {
    let mt = 1.0 - t;
    let w0 = mt * mt;
    let w1 = 2.0 * rho * mt * t;
    let w2 = t * t;
    let denom = w0 + w1 + w2;
    (a * w0 + apex * w1 + b * w2) / denom
}

pub(crate) fn bbox_of(polys: &[Vec<Vec2>]) -> (Vec2, Vec2) {
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
pub(crate) fn sanitize_filename(color: &str) -> String {
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

pub(crate) fn mm_to_screen(p: Vec2, rect: egui::Rect, pan: egui::Vec2, zoom: f32) -> egui::Pos2 {
    egui::pos2(
        rect.left() + pan.x + p.x as f32 * zoom,
        rect.top() + pan.y - p.y as f32 * zoom,
    )
}

pub(crate) fn screen_to_mm(p: egui::Pos2, rect: egui::Rect, pan: egui::Vec2, zoom: f32) -> Vec2 {
    Vec2::new(
        ((p.x - rect.left() - pan.x) / zoom) as f64,
        ((rect.top() + pan.y - p.y) / zoom) as f64,
    )
}
