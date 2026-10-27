//! SVG (W3C SVG 1.1) geometry for sketches: `path` (all commands), `line`, `rect`, `circle`,
//! `ellipse`, `polyline`, `polygon` and `text`, with `transform`s on elements and groups. User
//! units are CSS pixels (1/96 in) unless the root's `width`/`height` carry units with a
//! `viewBox`; y is flipped so the drawing reads upright in the sketch.

use std::collections::HashMap;

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use solidcraft_geom::Vec2;

use crate::sketch2d::{Geom2, MAX_DRAWING_BYTES, MAX_ENTITIES};
use crate::{IoError, Result};

const MAX_DEPTH: usize = 256;
const PX_MM: f64 = 25.4 / 96.0;

/// Affine map [a c e; b d f].
#[derive(Clone, Copy, Debug)]
struct Tf([f64; 6]);

impl Tf {
    const ID: Tf = Tf([1.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    fn mul(self, o: Tf) -> Tf {
        let [a, b, c, d, e, f] = self.0;
        let [a2, b2, c2, d2, e2, f2] = o.0;
        Tf([a * a2 + c * b2, b * a2 + d * b2, a * c2 + c * d2, b * c2 + d * d2, a * e2 + c * f2 + e, b * e2 + d * f2 + f])
    }
    fn apply(self, p: Vec2) -> Vec2 {
        let [a, b, c, d, e, f] = self.0;
        Vec2::new(a * p.x + c * p.y + e, b * p.x + d * p.y + f)
    }
    fn vec(self, v: Vec2) -> Vec2 {
        let [a, b, c, d, ..] = self.0;
        Vec2::new(a * v.x + c * v.y, b * v.x + d * v.y)
    }
    /// Uniform scale (with any rotation or mirror) and its factor.
    fn uniform(self) -> Option<f64> {
        let (u, v) = (self.vec(Vec2::X), self.vec(Vec2::Y));
        ((u.len() - v.len()).abs() < 1e-9 * u.len().max(1e-12) && u.dot(v).abs() < 1e-9 * u.len2().max(1e-24)).then_some(u.len())
    }
    fn flips(self) -> bool {
        let [a, b, c, d, ..] = self.0;
        a * d - b * c < 0.0
    }
    /// Inverse affine map. `None` for a degenerate matrix (zero scale), which cannot be
    /// inverted and would produce infinities if forced.
    fn inverse(self) -> Option<Tf> {
        let [a, b, c, d, e, f] = self.0;
        let det = a * d - b * c;
        if det.abs() < 1e-12 {
            return None;
        }
        let inv = 1.0 / det;
        Some(Tf([d * inv, -b * inv, -c * inv, a * inv, (c * f - d * e) * inv, (b * e - a * f) * inv]))
    }
    fn is_finite(self) -> bool {
        self.0.iter().all(|x| x.is_finite())
    }
}

/// Every number of `g` is finite: a transform such as `scale(1e308)` can push a finite size in
/// the file past the range of `f64`, and an infinite curve must never reach a sketch.
fn geom_is_finite(g: &Geom2) -> bool {
    match g {
        Geom2::Point(p) => p.is_finite(),
        Geom2::Line(a, b) => a.is_finite() && b.is_finite(),
        Geom2::Circle(c, r) => c.is_finite() && r.is_finite(),
        Geom2::Arc { c, a, b } => c.is_finite() && a.is_finite() && b.is_finite(),
        Geom2::Ellipse { c, major, minor } => c.is_finite() && major.is_finite() && minor.is_finite(),
        Geom2::Spline { pts, .. } => pts.iter().all(|p| p.is_finite()),
        Geom2::Conic { a, apex, b, rho } => a.is_finite() && apex.is_finite() && b.is_finite() && rho.is_finite(),
        Geom2::Text { at, height, angle, .. } => at.is_finite() && height.is_finite() && angle.is_finite(),
    }
}

const OUT_OF_RANGE: &str = "SVG geometry is out of range: a transform or size scales a shape past the largest number SolidCraft can store";

/// Apply a second affine map to a geometry element that is already in root coordinates.
/// Used to replay a `<defs>` definition at each `<use>` site: the definition was captured
/// with its own transform applied, so the use's transform is composed as a delta.
fn reapply(g: Geom2, tf: Tf) -> Geom2 {
    match g {
        Geom2::Point(p) => Geom2::Point(tf.apply(p)),
        Geom2::Line(a, b) => Geom2::Line(tf.apply(a), tf.apply(b)),
        Geom2::Circle(c, r) => {
            let s = tf.uniform().unwrap_or(1.0).abs();
            Geom2::Circle(tf.apply(c), r * s)
        }
        Geom2::Arc { c, a, b } => Geom2::Arc { c: tf.apply(c), a: tf.apply(a), b: tf.apply(b) },
        Geom2::Ellipse { c, major, minor } => {
            let s = tf.uniform().unwrap_or(1.0).abs();
            Geom2::Ellipse { c: tf.apply(c), major: tf.apply(major), minor: minor * s }
        }
        Geom2::Spline { pts, control, degree } => Geom2::Spline { pts: pts.iter().map(|p| tf.apply(*p)).collect(), control, degree },
        Geom2::Conic { a, apex, b, rho } => Geom2::Conic { a: tf.apply(a), apex: tf.apply(apex), b: tf.apply(b), rho },
        Geom2::Text { text, at, height, angle } => {
            let s = tf.uniform().unwrap_or(1.0).abs();
            let dir = tf.vec(Vec2::new(angle.cos(), angle.sin()));
            Geom2::Text { text, at: tf.apply(at), height: height * s, angle: dir.angle() }
        }
    }
}

fn numbers(s: &str) -> Vec<f64> {
    // Numbers separated by commas/space, also "1-2" and "1.5.5" forms.
    let mut out = Vec::new();
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() && out.len() < 1_000_000 {
        while i < b.len() && !(b[i] == b'-' || b[i] == b'+' || b[i] == b'.' || b[i].is_ascii_digit()) {
            i += 1;
        }
        let st = i;
        let mut seen_dot = false;
        let mut seen_e = false;
        if i < b.len() && (b[i] == b'-' || b[i] == b'+') {
            i += 1;
        }
        while i < b.len() {
            let ch = b[i];
            if ch.is_ascii_digit() {
                i += 1;
            } else if ch == b'.' && !seen_dot && !seen_e {
                seen_dot = true;
                i += 1;
            } else if (ch == b'e' || ch == b'E') && !seen_e {
                seen_e = true;
                i += 1;
                if i < b.len() && (b[i] == b'-' || b[i] == b'+') {
                    i += 1;
                }
            } else {
                break;
            }
        }
        if let Some(x) = s.get(st..i).and_then(|t| t.parse::<f64>().ok()).filter(|x| x.is_finite()) {
            out.push(x);
        } else if i == st {
            i += 1;
        }
    }
    out
}

fn parse_transform(s: &str) -> Tf {
    let mut t = Tf::ID;
    for part in s.split(')') {
        let Some((name, args)) = part.split_once('(') else { continue };
        let n = numbers(args);
        let g = |i: usize, d: f64| n.get(i).copied().unwrap_or(d);
        let m = match name.trim().trim_start_matches(',').trim() {
            "matrix" if n.len() >= 6 => Tf([n[0], n[1], n[2], n[3], n[4], n[5]]),
            "translate" => Tf([1.0, 0.0, 0.0, 1.0, g(0, 0.0), g(1, 0.0)]),
            "scale" => Tf([g(0, 1.0), 0.0, 0.0, g(1, g(0, 1.0)), 0.0, 0.0]),
            "rotate" => {
                let (s, c) = g(0, 0.0).to_radians().sin_cos();
                let r = Tf([c, s, -s, c, 0.0, 0.0]);
                let (cx, cy) = (g(1, 0.0), g(2, 0.0));
                Tf([1.0, 0.0, 0.0, 1.0, cx, cy]).mul(r).mul(Tf([1.0, 0.0, 0.0, 1.0, -cx, -cy]))
            }
            "skewX" => Tf([1.0, 0.0, g(0, 0.0).to_radians().tan(), 1.0, 0.0, 0.0]),
            "skewY" => Tf([1.0, g(0, 0.0).to_radians().tan(), 0.0, 1.0, 0.0, 0.0]),
            _ => Tf::ID,
        };
        t = t.mul(m);
    }
    t
}

fn attrs(e: &BytesStart) -> Vec<(String, String)> {
    e.attributes()
        .flatten()
        .map(|a| {
            let k = String::from_utf8_lossy(a.key.local_name().as_ref()).to_string();
            let v = a
                .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                .map(|v| v.to_string())
                .unwrap_or_else(|_| String::from_utf8_lossy(&a.value).to_string());
            (k, v)
        })
        .collect()
}

fn attr<'a>(a: &'a [(String, String)], k: &str) -> Option<&'a str> {
    a.iter().find(|(n, _)| n == k).map(|(_, v)| v.as_str())
}

fn num_attr(a: &[(String, String)], k: &str) -> f64 {
    attr(a, k).map(numbers).and_then(|v| v.first().copied()).unwrap_or(0.0)
}

/// Length with a unit, in millimetres (None without a unit or for %).
fn length_mm(s: &str) -> Option<f64> {
    let v = numbers(s).first().copied()?;
    let u = s.trim().trim_start_matches(|c: char| c.is_ascii_digit() || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E');
    Some(match u.trim() {
        "mm" => v,
        "cm" => v * 10.0,
        "in" => v * 25.4,
        "pt" => v * 25.4 / 72.0,
        "pc" => v * 25.4 / 6.0,
        "px" | "" => v * PX_MM,
        _ => return None,
    })
}

struct Out {
    g: Vec<Geom2>,
    /// Parallel to `g`: the stroke/fill colour in effect for each entity, `None` when the
    /// document declares none. Kept alongside the geometry rather than inside [`Geom2`] so
    /// callers that only want geometry are unaffected.
    colors: Vec<Option<String>>,
    /// Colour in effect for the element being processed. The event loop sets this before
    /// dispatching an element and restores the inherited value when the element closes, so
    /// every `push` during that element records the same colour.
    current: Option<String>,
}

impl Out {
    fn push(&mut self, g: Geom2) -> Result<()> {
        if self.g.len() >= MAX_ENTITIES {
            return Err(IoError::Invalid("too many SVG elements".into()));
        }
        if !geom_is_finite(&g) {
            return Err(IoError::Invalid(OUT_OF_RANGE.into()));
        }
        self.g.push(g);
        self.colors.push(self.current.clone());
        Ok(())
    }

    /// Drop geometry past `mark`, keeping the colour vector in lockstep.
    ///
    /// Called when a definition container closes: its contents were captured for `<use>`
    /// replay and must not also render in place. Truncating only `g` would desynchronise the
    /// two vectors and attribute every later entity to the wrong colour.
    fn truncate(&mut self, mark: usize) {
        self.g.truncate(mark);
        self.colors.truncate(mark);
    }
    fn line(&mut self, t: Tf, a: Vec2, b: Vec2) -> Result<()> {
        let (a, b) = (t.apply(a), t.apply(b));
        if !(a.is_finite() && b.is_finite()) {
            // Checked before the length: the distance between infinite ends is NaN, which would
            // drop the line silently.
            return Err(IoError::Invalid(OUT_OF_RANGE.into()));
        }
        if a.dist(b) > 1e-12 { self.push(Geom2::Line(a, b)) } else { Ok(()) }
    }
    fn cubic(&mut self, t: Tf, p: [Vec2; 4]) -> Result<()> {
        self.push(Geom2::Spline { pts: p.iter().map(|q| t.apply(*q)).collect(), control: true, degree: 3 })
    }
    fn quad(&mut self, t: Tf, a: Vec2, c: Vec2, b: Vec2) -> Result<()> {
        let (a, c, b) = (t.apply(a), t.apply(c), t.apply(b));
        if (b - a).cross(c - a).abs() < 1e-12 { self.push(Geom2::Line(a, b)) } else { self.push(Geom2::Conic { a, apex: c, b, rho: 0.5 }) }
    }
    /// Ellipse `c + rx·cos t·u + ry·sin t·v` (u at angle `phi`) from `t0` sweeping `dt`.
    fn elliptic(&mut self, t: Tf, c: Vec2, rx: f64, ry: f64, phi: f64, t0: f64, dt: f64) -> Result<()> {
        let u = Vec2::from_angle(phi);
        let v = u.perp();
        let at = |a: f64| c + u * (rx * a.cos()) + v * (ry * a.sin());
        let circle = t.uniform().filter(|_| (rx - ry).abs() < 1e-9 * rx.max(ry));
        if let Some(k) = circle {
            let (p0, p1) = (t.apply(at(t0)), t.apply(at(t0 + dt)));
            let cc = t.apply(c);
            let ccw = (dt > 0.0) != t.flips();
            if dt.abs() >= std::f64::consts::TAU - 1e-9 {
                return self.push(Geom2::Circle(cc, rx * k));
            }
            return self.push(if ccw { Geom2::Arc { c: cc, a: p0, b: p1 } } else { Geom2::Arc { c: cc, a: p1, b: p0 } });
        }
        if dt.abs() >= std::f64::consts::TAU - 1e-9 {
            // Full ellipse: centre, major end, minor radius after the transform (affine images
            // of ellipses are ellipses; sample to find the axes).
            let pts: Vec<Vec2> = (0..720).map(|i| t.apply(at(i as f64 / 720.0 * std::f64::consts::TAU))).collect();
            let cc = t.apply(c);
            let (mut far, mut near) = (cc, f64::INFINITY);
            for q in &pts {
                if q.dist(cc) > far.dist(cc) {
                    far = *q;
                }
                near = near.min(q.dist(cc));
            }
            return self.push(Geom2::Ellipse { c: cc, major: far, minor: near });
        }
        // Elliptic arc: cubic Béziers of at most 90 degrees each.
        let n = ((dt.abs() / std::f64::consts::FRAC_PI_2).ceil() as usize).max(1);
        let h = dt / n as f64;
        let k = 4.0 / 3.0 * (h / 4.0).tan();
        let d = |a: f64| u * (-rx * a.sin()) + v * (ry * a.cos());
        for i in 0..n {
            let (a0, a1) = (t0 + h * i as f64, t0 + h * (i + 1) as f64);
            self.cubic(t, [at(a0), at(a0) + d(a0) * k, at(a1) - d(a1) * k, at(a1)])?;
        }
        Ok(())
    }
}

/// SVG endpoint arc → centre form (SVG 1.1 implementation notes F.6.5).
#[allow(clippy::too_many_arguments)]
fn arc_to(out: &mut Out, t: Tf, p0: Vec2, mut rx: f64, mut ry: f64, phi_deg: f64, large: bool, sweep: bool, p1: Vec2) -> Result<()> {
    if p0.dist(p1) < 1e-12 {
        return Ok(());
    }
    rx = rx.abs();
    ry = ry.abs();
    if rx < 1e-12 || ry < 1e-12 {
        return out.line(t, p0, p1);
    }
    let phi = phi_deg.to_radians();
    let (s, c) = phi.sin_cos();
    let d = (p0 - p1) * 0.5;
    let x1 = Vec2::new(c * d.x + s * d.y, -s * d.x + c * d.y);
    let lam = (x1.x / rx).powi(2) + (x1.y / ry).powi(2);
    if lam > 1.0 {
        rx *= lam.sqrt();
        ry *= lam.sqrt();
    }
    let num = (rx * rx * ry * ry - rx * rx * x1.y * x1.y - ry * ry * x1.x * x1.x).max(0.0);
    let den = rx * rx * x1.y * x1.y + ry * ry * x1.x * x1.x;
    let mut co = if den > 0.0 { (num / den).sqrt() } else { 0.0 };
    if large == sweep {
        co = -co;
    }
    let cx1 = Vec2::new(co * rx * x1.y / ry, -co * ry * x1.x / rx);
    let mid = (p0 + p1) * 0.5;
    let cc = Vec2::new(c * cx1.x - s * cx1.y, s * cx1.x + c * cx1.y) + mid;
    let ang = |u: Vec2, v: Vec2| u.cross(v).atan2(u.dot(v));
    let u = Vec2::new((x1.x - cx1.x) / rx, (x1.y - cx1.y) / ry);
    let v = Vec2::new((-x1.x - cx1.x) / rx, (-x1.y - cx1.y) / ry);
    let t0 = ang(Vec2::X, u);
    let mut dt = ang(u, v);
    if !sweep && dt > 0.0 {
        dt -= std::f64::consts::TAU;
    } else if sweep && dt < 0.0 {
        dt += std::f64::consts::TAU;
    }
    out.elliptic(t, cc, rx, ry, phi, t0, dt)
}

fn path(out: &mut Out, t: Tf, d: &str) -> Result<()> {
    // Tokenise into commands and their numbers.
    let mut cmds: Vec<(char, Vec<f64>)> = Vec::new();
    let mut cur: Option<(char, String)> = None;
    for ch in d.chars() {
        if ch.is_ascii_alphabetic() && ch != 'e' && ch != 'E' {
            if let Some((c, s)) = cur.take() {
                cmds.push((c, numbers(&s)));
            }
            cur = Some((ch, String::new()));
        } else if let Some((_, s)) = cur.as_mut() {
            s.push(ch);
        }
    }
    if let Some((c, s)) = cur.take() {
        cmds.push((c, numbers(&s)));
    }
    let (mut p, mut start) = (Vec2::ZERO, Vec2::ZERO);
    let mut last_c: Option<Vec2> = None; // last cubic control (for S)
    let mut last_q: Option<Vec2> = None; // last quadratic control (for T)
    for (c, n) in cmds {
        let rel = c.is_ascii_lowercase();
        let base = |p: Vec2| if rel { p } else { Vec2::ZERO };
        let arity = match c.to_ascii_uppercase() {
            'M' | 'L' | 'T' => 2,
            'H' | 'V' => 1,
            'C' => 6,
            'S' | 'Q' => 4,
            'A' => 7,
            _ => 0,
        };
        if c.eq_ignore_ascii_case(&'Z') {
            out.line(t, p, start)?;
            p = start;
            last_c = None;
            last_q = None;
            continue;
        }
        if arity == 0 {
            continue;
        }
        for (k, a) in n.chunks(arity).enumerate() {
            if a.len() < arity {
                break;
            }
            let o = base(p);
            let v = |i: usize| Vec2::new(a[i], a[i + 1]) + o;
            match c.to_ascii_uppercase() {
                'M' => {
                    let q = v(0);
                    if k == 0 {
                        p = q;
                        start = q;
                    } else {
                        out.line(t, p, q)?;
                        p = q;
                    }
                    last_c = None;
                    last_q = None;
                }
                'L' => {
                    let q = v(0);
                    out.line(t, p, q)?;
                    p = q;
                    last_c = None;
                    last_q = None;
                }
                'H' => {
                    let q = Vec2::new(a[0] + if rel { p.x } else { 0.0 }, p.y);
                    out.line(t, p, q)?;
                    p = q;
                    last_c = None;
                    last_q = None;
                }
                'V' => {
                    let q = Vec2::new(p.x, a[0] + if rel { p.y } else { 0.0 });
                    out.line(t, p, q)?;
                    p = q;
                    last_c = None;
                    last_q = None;
                }
                'C' => {
                    let (c1, c2, q) = (v(0), v(2), v(4));
                    out.cubic(t, [p, c1, c2, q])?;
                    last_c = Some(c2);
                    last_q = None;
                    p = q;
                }
                'S' => {
                    let c1 = last_c.map(|lc| p * 2.0 - lc).unwrap_or(p);
                    let (c2, q) = (v(0), v(2));
                    out.cubic(t, [p, c1, c2, q])?;
                    last_c = Some(c2);
                    last_q = None;
                    p = q;
                }
                'Q' => {
                    let (c1, q) = (v(0), v(2));
                    out.quad(t, p, c1, q)?;
                    last_q = Some(c1);
                    last_c = None;
                    p = q;
                }
                'T' => {
                    let c1 = last_q.map(|lq| p * 2.0 - lq).unwrap_or(p);
                    let q = v(0);
                    out.quad(t, p, c1, q)?;
                    last_q = Some(c1);
                    last_c = None;
                    p = q;
                }
                'A' => {
                    let q = Vec2::new(a[5], a[6]) + o;
                    arc_to(out, t, p, a[0], a[1], a[2], a[3] != 0.0, a[4] != 0.0, q)?;
                    last_c = None;
                    last_q = None;
                    p = q;
                }
                _ => {}
            }
        }
    }
    Ok(())
}

fn points_list(s: &str) -> Vec<Vec2> {
    numbers(s).chunks(2).filter(|c| c.len() == 2).map(|c| Vec2::new(c[0], c[1])).collect()
}

/// Read the geometry of an SVG document (millimetres, y up). `scale` multiplies the result.
fn read_svg_inner(text: &str) -> Result<(Vec<Geom2>, Vec<Option<String>>)> {
    if text.len() > MAX_DRAWING_BYTES {
        return Err(IoError::Invalid("the SVG file is too large".into()));
    }
    let mut rd = Reader::from_reader(text.as_bytes());
    rd.config_mut().trim_text(true);
    let mut buf = Vec::new();
    let mut stack: Vec<Tf> = Vec::new();
    // Colour inheritance stack, parallel to `stack`: a Start pushes the colour its children
    // inherit and its End pops it. SVG inherits stroke/fill down the element tree, so this
    // mirrors how `stack` carries the transform.
    let mut style_stack: Vec<Option<String>> = Vec::new();
    let mut out = Out { g: Vec::new(), colors: Vec::new(), current: None };
    let mut root: Option<Tf> = None;
    let mut pending_text: Option<(Tf, Vec2, f64)> = None;
    // `<defs>`/`<symbol>` hold geometry that only renders where a `<use>` points at it.
    // Emitting it directly cuts the definition at its own coordinates — silently wrong.
    // Capture it here and replay it at each use site. The colour vector is carried along
    // with the geometry so a definition drawn in red stays red when it is replayed.
    let mut defs: HashMap<String, (Tf, Vec<Geom2>, Vec<Option<String>>)> = HashMap::new();
    let mut defs_depth: usize = 0;
    // Marks in `out.g` where each `<defs>`/`<symbol>` started, so uncaptured geometry can
    // be dropped when the container closes. Parallel to those Start events.
    let mut defs_marks: Vec<usize> = Vec::new();
    // Parallel to `stack`: one entry per open Start element, holding its name and the
    // capture it owns (id, mark in `out.g`, transform) when inside a definition.
    let mut marks: Vec<(String, Option<(String, usize, Tf)>)> = Vec::new();
    // `<use>` sites whose target was not yet defined; resolved after the document is read.
    let mut pending_uses: Vec<(String, Tf)> = Vec::new();
    loop {
        let ev = rd.read_event_into(&mut buf).map_err(|e| IoError::Invalid(format!("SVG: {e}")))?;
        let (e, empty) = match &ev {
            Event::Start(e) => (Some(e.clone()), false),
            Event::Empty(e) => (Some(e.clone()), true),
            Event::End(end) => {
                let name = String::from_utf8_lossy(end.local_name().as_ref()).to_string();
                // Capture first. Inside a definition container the geometry must be dropped
                // from the output stream: a definition is not cut where it is written. An
                // identified element outside a container both renders in place and serves as
                // a `<use>` target, so its geometry stays.
                if let Some((_, Some((id, mark, tf)))) = marks.pop() {
                    defs.insert(id, (tf, out.g[mark..].to_vec(), out.colors[mark..].to_vec()));
                    if defs_depth > 0 {
                        out.truncate(mark);
                    }
                }
                if name == "defs" || name == "symbol" {
                    defs_depth = defs_depth.saturating_sub(1);
                    if let Some(mark) = defs_marks.pop() {
                        // Anything left in the buffer was never identified; dropping it beats
                        // cutting a definition in the wrong place.
                        out.truncate(mark);
                    }
                }
                stack.pop();
                style_stack.pop();
                buf.clear();
                continue;
            }
            Event::Text(tx) => {
                if let Some((tf, at, h)) = pending_text.take() {
                    let s = String::from_utf8_lossy(tx.as_ref()).trim().to_string();
                    if !s.is_empty() {
                        let k = tf.uniform().unwrap_or(1.0);
                        let dir = tf.vec(Vec2::X);
                        out.push(Geom2::Text { text: s, at: tf.apply(at), height: h * 0.7 * k, angle: dir.angle() })?;
                    }
                }
                buf.clear();
                continue;
            }
            Event::Eof => break,
            _ => (None, false),
        };
        let Some(e) = e else {
            buf.clear();
            continue;
        };
        let a = attrs(&e);
        let parent = stack.last().copied().or(root).unwrap_or(Tf::ID);
        let name = String::from_utf8_lossy(e.local_name().as_ref()).to_string();
        let mut tf = match attr(&a, "transform") {
            Some(s) => parent.mul(parse_transform(s)),
            None => parent,
        };
        if name == "svg" && root.is_none() {
            // Root: user units to millimetres, y flipped.
            let vb = attr(&a, "viewBox").map(numbers).filter(|v| v.len() == 4);
            let (sx, sy, ox, oy) = match (vb, attr(&a, "width").and_then(length_mm), attr(&a, "height").and_then(length_mm)) {
                (Some(v), Some(w), Some(h)) if v[2] > 0.0 && v[3] > 0.0 => (w / v[2], h / v[3], v[0], v[1]),
                (Some(v), _, _) => (PX_MM, PX_MM, v[0], v[1]),
                _ => (PX_MM, PX_MM, 0.0, 0.0),
            };
            let r = Tf([sx, 0.0, 0.0, -sy, -ox * sx, oy * sy]);
            root = Some(r);
            tf = r;
        }
        if !tf.is_finite() {
            return Err(IoError::Invalid(OUT_OF_RANGE.into()));
        }
        // Colour inheritance: an element's own stroke/fill wins over what it inherits from its
        // ancestors. The stack mirrors `stack` (transforms) — a Start pushes and its End pops —
        // so the value recorded here is the one in force for everything this element emits.
        let inherited = style_stack.last().cloned().flatten();
        let effective = colour_of(&a).or(inherited);
        out.current = effective.clone();
        // Any element with an `id` is a potential `<use>` target — SVG does not require the
        // target to live inside `<defs>`. Capture every identified element so a later
        // `<use href="#id">` resolves, but only *hide* its geometry when it sits inside a
        // definition container. An identified element outside `<defs>` renders in place AND
        // is usable as a target, which is the spec's behaviour.
        let id = attr(&a, "id").filter(|s| !s.is_empty()).map(|s| s.to_string());
        let container = name == "defs" || name == "symbol";
        let mut capture: Option<(String, usize, Tf)> = None;
        if let Some(id) = id.clone() {
            capture = Some((id, out.g.len(), tf));
        }
        if container {
            defs_depth += 1;
            defs_marks.push(out.g.len());
        }
        match name.as_str() {
            "path" => path(&mut out, tf, attr(&a, "d").unwrap_or(""))?,
            "line" => out.line(tf, Vec2::new(num_attr(&a, "x1"), num_attr(&a, "y1")), Vec2::new(num_attr(&a, "x2"), num_attr(&a, "y2")))?,
            "rect" => {
                let (x, y, w, h) = (num_attr(&a, "x"), num_attr(&a, "y"), num_attr(&a, "width"), num_attr(&a, "height"));
                if w > 0.0 && h > 0.0 {
                    let c = [Vec2::new(x, y), Vec2::new(x + w, y), Vec2::new(x + w, y + h), Vec2::new(x, y + h)];
                    for i in 0..4 {
                        out.line(tf, c[i], c[(i + 1) % 4])?;
                    }
                }
            }
            "circle" => {
                let r = num_attr(&a, "r");
                if r > 0.0 {
                    out.elliptic(tf, Vec2::new(num_attr(&a, "cx"), num_attr(&a, "cy")), r, r, 0.0, 0.0, std::f64::consts::TAU)?;
                }
            }
            "ellipse" => {
                let (rx, ry) = (num_attr(&a, "rx"), num_attr(&a, "ry"));
                if rx > 0.0 && ry > 0.0 {
                    out.elliptic(tf, Vec2::new(num_attr(&a, "cx"), num_attr(&a, "cy")), rx, ry, 0.0, 0.0, std::f64::consts::TAU)?;
                }
            }
            "polyline" | "polygon" => {
                let pts = points_list(attr(&a, "points").unwrap_or(""));
                for w in pts.windows(2) {
                    out.line(tf, w[0], w[1])?;
                }
                if name == "polygon"
                    && let (Some(f), Some(l)) = (pts.first(), pts.last())
                {
                    out.line(tf, *l, *f)?;
                }
            }
            "text" => {
                let size = attr(&a, "font-size").map(numbers).and_then(|v| v.first().copied()).unwrap_or(16.0);
                pending_text = Some((tf, Vec2::new(num_attr(&a, "x"), num_attr(&a, "y")), size));
            }
            "use" => {
                // `<use href="#id">` renders the referenced definition here. `x`/`y` shift the
                // instance and `transform` is already folded into `tf`, matching the spec order
                // (use transform, then its x/y translate, then the definition's own transform).
                let href = attr(&a, "href").or_else(|| attr(&a, "xlink:href"));
                let target = href.and_then(|h| {
                    let h = h.trim();
                    h.strip_prefix('#').filter(|s| !s.is_empty()).map(|s| s.to_string())
                });
                let shift = Vec2::new(num_attr(&a, "x"), num_attr(&a, "y"));
                let site = tf.mul(Tf([1.0, 0.0, 0.0, 1.0, shift.x, shift.y]));
                match target {
                    Some(id) => match defs.get(&id) {
                        Some((def_tf, geom, colors)) => {
                            // Replay the definition at this site: undo where it was written,
                            // then apply the use's placement. One affine delta is exact. Each
                            // entity keeps the colour it was captured with, so a red definition
                            // stays red wherever a `<use>` points at it.
                            let delta = def_tf.inverse().map(|inv| site.mul(inv));
                            for (g, c) in geom.iter().zip(colors.iter()) {
                                let saved = out.current.clone();
                                out.current = c.clone();
                                match delta {
                                    Some(d) => out.push(reapply(g.clone(), d))?,
                                    // Degenerate definition transform: place it unmoved rather
                                    // than emit infinities.
                                    None => out.push(g.clone())?,
                                }
                                out.current = saved;
                            }
                        }
                        None => {
                            // Forward reference or missing id. Keep the site so it can be
                            // resolved once the document is fully read.
                            pending_uses.push((id, site));
                        }
                    },
                    None => {}
                }
            }
            _ => {}
        }
        // An Empty element inside a definition has no End event, so capture it now.
        if empty {
            if let Some((id, mark, tf)) = capture {
                defs.insert(id, (tf, out.g[mark..].to_vec(), out.colors[mark..].to_vec()));
                // Hide the geometry only when this element is a definition. An identified
                // element outside `<defs>` renders in place and is merely *also* a target.
                if defs_depth > 0 {
                    out.truncate(mark);
                }
            }
            if container {
                // A self-closing `<defs/>` or `<symbol/>` never produces an End event, so
                // unwind its container bookkeeping here instead of leaving it dangling.
                defs_depth = defs_depth.saturating_sub(1);
                defs_marks.pop();
            }
        } else {
            stack.push(tf);
            style_stack.push(effective);
            marks.push((name, capture));
            if stack.len() > MAX_DEPTH {
                return Err(IoError::Invalid("SVG nested too deeply".into()));
            }
        }
        buf.clear();
    }
    // Resolve uses whose definition appeared later in the document (or was itself a use).
    for (id, site) in pending_uses {
        let Some((def_tf, geom, colors)) = defs.get(&id) else {
            // A dangling `#id` is a broken document. Failing loudly beats loading an empty
            // design the operator might then send to the cutter as a blank job.
            return Err(IoError::Invalid(format!("SVG: <use> refers to a missing definition '#{id}'")));
        };
        let delta = def_tf.inverse().map(|inv| site.mul(inv));
        for (g, c) in geom.iter().zip(colors.iter()) {
            let saved = out.current.clone();
            out.current = c.clone();
            match delta {
                Some(d) => out.push(reapply(g.clone(), d))?,
                None => out.push(g.clone())?,
            }
            out.current = saved;
        }
    }
    Ok((out.g, out.colors))
}

/// Pick the colour an element declares for itself, if any.
///
/// A cutter follows the stroke, so `stroke` wins over `fill`: a shape drawn with a red outline
/// on a blue fill is cut in red vinyl. `style="stroke:#f00;fill:#00f"` carries the same two
/// properties inline, so it is consulted last. `none` means "draw nothing" and is not a colour.
fn colour_of(a: &[(String, String)]) -> Option<String> {
    fn usable(v: &str) -> Option<String> {
        let v = v.trim();
        if v.is_empty() || v.eq_ignore_ascii_case("none") || v.eq_ignore_ascii_case("transparent") { None } else { Some(v.to_string()) }
    }
    if let Some(s) = attr(a, "stroke").and_then(usable) {
        return Some(s);
    }
    if let Some(s) = attr(a, "fill").and_then(usable) {
        return Some(s);
    }
    if let Some(s) = attr(a, "style") {
        for part in s.split(';') {
            let mut kv = part.splitn(2, ':');
            let key = kv.next().unwrap_or("").trim();
            let value = kv.next().unwrap_or("").trim();
            if key.eq_ignore_ascii_case("stroke") || key.eq_ignore_ascii_case("fill") {
                if let Some(v) = usable(value) {
                    return Some(v);
                }
            }
        }
    }
    None
}

/// Read the geometry of an SVG document (millimetres, y up).
pub fn read_svg(text: &str) -> Result<Vec<Geom2>> {
    read_svg_inner(text).map(|(geometry, _)| geometry)
}

/// Read an SVG document together with the colour in effect for each entity.
///
/// The colour vector is parallel to the geometry: `colors[i]` is the stroke/fill declared for
/// `geometry[i]`, or `None` when the document leaves it unset. Callers that cut by colour group
/// the entities by this value; callers that only want geometry use [`read_svg`].
///
/// Colour inherits down the element tree the way SVG specifies: an element's own `stroke`,
/// `fill`, or inline `style` wins, otherwise it takes its parent's. `stroke` beats `fill`
/// because that is the line a cutter follows. Definitions referenced by `<use>` keep the colour
/// they were drawn with, so a red symbol stays red at every use site.
pub fn read_svg_colored(text: &str) -> Result<(Vec<Geom2>, Vec<Option<String>>)> {
    read_svg_inner(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The parser recurses over groups and reads numbers out of a document the user did not write,
    /// so none of these may panic: `catch_unwind` turns one into a failed test, the way
    /// `hostile_files_never_panic` does for STEP (solidcraft#469).
    #[test]
    fn hostile_svg_never_panics() {
        let good = r###"<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="50mm" viewBox="0 0 100 50">
            <g transform="translate(10 5)"><rect x="1" y="2" width="3" height="4"/><path d="M0 0 L5 5 Z"/></g>
        </svg>"###;
        let mut cases: Vec<String> = vec![
            String::new(),
            "<svg".into(),
            "<svg><".into(),
            "<svg><path d=".into(),
            "<svg><path d='M 1 1 L'/></svg>".into(),
            "<svg><path d='M nan nan L inf inf'/></svg>".into(),
            "<svg><path d='M 1e400 1e400 L 1 1'/></svg>".into(),
            "<svg><rect width='-1' height='-1'/></svg>".into(),
            "<svg><use href='#a'/><g id='a'><use href='#a'/></g></svg>".into(),
            format!("<svg>{}{}</svg>", "<g>".repeat(300), "</g>".repeat(300)),
            format!("<svg>{}</svg>", "<g>".repeat(300)),
            "<svg><![CDATA[<g>]]></svg>".into(),
            "<?xml version='1.0'?><!DOCTYPE svg [<!ENTITY x SYSTEM 'file:///etc/passwd'>]><svg>&x;</svg>".into(),
            format!("<svg><path d='{}'/></svg>", "L 1 1 ".repeat(20_000)),
        ];
        for cut in (0..good.len()).step_by(13) {
            cases.push(good.get(..cut).unwrap_or("").to_string());
        }
        for (from, to) in [("10mm", "-10mm"), ("0 0 100 50", "0 0 0 0"), ("<g ", "<x "), ("rect", "rectx")] {
            cases.push(good.replacen(from, to, 3));
        }
        for c in &cases {
            let r = std::panic::catch_unwind(|| read_svg(c));
            assert!(r.is_ok(), "panicked on {:?}", c.get(..c.len().min(200)));
        }
    }

    #[test]
    fn shapes_paths_and_units() {
        let svg = r###"<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="50mm" viewBox="0 0 100 50">
            <rect x="10" y="10" width="20" height="10"/>
            <circle cx="50" cy="25" r="5"/>
            <g transform="translate(70 0)"><path d="M0 0 C 10 0 10 10 20 10 Q 25 20 30 10 A 5 5 0 0 1 40 10 Z"/></g>
            <ellipse cx="80" cy="40" rx="6" ry="3"/>
        </svg>"###;
        let g = read_svg(svg).unwrap();
        // rect 4 lines, circle, cubic, quad, arc, closing line, ellipse
        assert_eq!(g.len(), 4 + 1 + 4 + 1, "{g:?}");
        // y flipped: the circle centre is at (50, -25) in mm.
        assert!(g.iter().any(|x| matches!(x, Geom2::Circle(c, r) if c.dist(Vec2::new(50.0, -25.0)) < 1e-9 && (r - 5.0).abs() < 1e-9)));
        assert!(g.iter().any(|x| matches!(x, Geom2::Spline { control: true, .. })));
        assert!(g.iter().any(|x| matches!(x, Geom2::Conic { .. })));
        assert!(g.iter().any(|x| matches!(x, Geom2::Arc { .. })));
        assert!(g.iter().any(|x| matches!(x, Geom2::Ellipse { minor, .. } if (minor - 3.0).abs() < 1e-3)));
        assert!(read_svg("<svg><path d='M 1e400 0 L nan 3'/></svg>").is_ok());
        assert!(read_svg("<svg").is_err() || read_svg("<svg").is_ok());
        // Plain pixels.
        let g = read_svg(r###"<svg><line x1="0" y1="0" x2="96" y2="0"/></svg>"###).unwrap();
        assert!(matches!(g[0], Geom2::Line(a, b) if (a.dist(b) - 25.4).abs() < 1e-9));
    }

    /// A transform that scales a shape past the range of `f64` is refused with a clear error
    /// instead of handing the sketch an infinite curve (solidcraft#5020).
    #[test]
    fn geometry_scaled_to_infinity_is_refused() {
        let doc =
            |body: &str| format!(r###"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="100mm" height="100mm">{body}</svg>"###);
        for body in [
            r###"<g transform="scale(1e308)"><circle r="5"/></g>"###,
            r###"<g transform="scale(1e308)"><g transform="scale(1e308)"></g></g>"###,
            r###"<g transform="scale(1e308)"><rect width="5" height="5"/></g>"###,
            r###"<g transform="scale(1e308)"><path d="M0 0 C 1 1 2 2 5 5"/></g>"###,
            r###"<g transform="matrix(1e308 0 0 1e308 1e308 1e308)"><line x1="1" y1="1" x2="5" y2="5"/></g>"###,
            r###"<g transform="scale(1e308)"><text x="0" y="0" font-size="16">A</text></g>"###,
        ] {
            match read_svg(&doc(body)) {
                Err(IoError::Invalid(m)) => assert!(m.contains("out of range"), "{body}: {m}"),
                other => panic!("{body}: expected an out-of-range error, got {other:?}"),
            }
        }
        // Large but finite drawings still read.
        let g = read_svg(&doc(r###"<g transform="scale(1e6)"><circle r="5"/></g>"###)).unwrap();
        assert!(matches!(g[0], Geom2::Circle(_, r) if (r - 5e6).abs() < 1e-3));
    }

    /// Line endpoints as sorted, orientation-normalised integer millimetre pairs. Integer
    /// comparison keeps the assertions readable; every coordinate here is exact.
    fn line_pairs(g: &[Geom2]) -> Vec<((i64, i64), (i64, i64))> {
        let mut v: Vec<((i64, i64), (i64, i64))> = g
            .iter()
            .filter_map(|x| match x {
                Geom2::Line(a, b) => {
                    let (p, q) = if (a.x, a.y) <= (b.x, b.y) { (*a, *b) } else { (*b, *a) };
                    Some(((p.x.round() as i64, p.y.round() as i64), (q.x.round() as i64, q.y.round() as i64)))
                }
                _ => None,
            })
            .collect();
        v.sort();
        v
    }

    fn doc_50(body: &str) -> String {
        format!(r###"<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="50mm" viewBox="0 0 100 50">{body}</svg>"###)
    }

    /// A `<defs>` element holds geometry that renders only where a `<use>` points at it.
    /// Emitting it where it is written cut a stray square at the definition's own coordinates
    /// and dropped both intended copies (silent geometry corruption).
    #[test]
    fn defs_geometry_is_not_cut_where_it_is_written() {
        let g = read_svg(&doc_50(r###"<defs><rect id="a" x="0" y="0" width="10" height="10"/></defs>"###)).unwrap();
        assert!(g.is_empty(), "a definition must not be cut in place: {g:?}");
    }

    /// `<use>` replays the definition at the use site. With no offset the replay must land
    /// exactly where the same shape written inline would.
    #[test]
    fn use_replays_definition_in_place() {
        let g = read_svg(&doc_50(r###"<defs><rect id="a" x="0" y="0" width="10" height="10"/></defs><use href="#a"/>"###)).unwrap();
        // y is flipped: SVG y 0..10 becomes mm y 0..-10.
        assert_eq!(line_pairs(&g), vec![((0, -10), (0, 0)), ((0, -10), (10, -10)), ((0, 0), (10, 0)), ((10, -10), (10, 0))], "{g:?}");
    }

    /// The use's `x`/`y` shift the instance. In a y-down document an SVG y offset moves the
    /// shape down, which is -y in millimetres after the import flip.
    #[test]
    fn use_x_y_offsets_place_the_instance() {
        let g = read_svg(&doc_50(r###"<defs><rect id="a" x="0" y="0" width="10" height="10"/></defs><use href="#a" x="5" y="5"/>"###)).unwrap();
        assert_eq!(line_pairs(&g), vec![((5, -15), (5, -5)), ((5, -15), (15, -15)), ((5, -5), (15, -5)), ((15, -15), (15, -5))], "{g:?}");
    }

    /// Two uses of one definition produce two instances, and the y offset separates them.
    #[test]
    fn two_uses_of_one_definition_produce_two_instances() {
        let g = read_svg(&doc_50(r###"<defs><rect id="a" x="0" y="0" width="10" height="10"/></defs><use href="#a"/><use href="#a" y="20"/>"###))
            .unwrap();
        assert_eq!(g.len(), 8, "expected two four-line rectangles: {g:?}");
        let ys: Vec<f64> = g
            .iter()
            .filter_map(|x| match x {
                Geom2::Line(a, _) => Some(a.y),
                _ => None,
            })
            .collect();
        let (lo, hi) = (ys.iter().cloned().fold(f64::INFINITY, f64::min), ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
        // 0..-10 and -20..-30 -> a 30 mm span, not overlapping.
        assert!((hi - lo - 30.0).abs() < 1e-9, "instances not offset: y {lo}..{hi}");
    }

    /// A `<g id>` inside `<defs>` is captured as a whole, not just its children.
    #[test]
    fn grouped_definition_is_captured() {
        let g = read_svg(&doc_50(r###"<defs><g id="grp"><rect x="0" y="0" width="10" height="10"/></g></defs><use href="#grp"/>"###)).unwrap();
        assert_eq!(g.len(), 4, "grouped definition not replayed: {g:?}");
    }

    /// A `<symbol id>` is both a container and a usable definition.
    #[test]
    fn symbol_definition_is_captured() {
        let g =
            read_svg(&doc_50(r###"<defs><symbol id="sym"><rect x="0" y="0" width="10" height="10"/></symbol></defs><use href="#sym"/>"###)).unwrap();
        assert_eq!(g.len(), 4, "symbol definition not replayed: {g:?}");
    }

    /// A use may appear before its definition; it resolves after the document is read.
    #[test]
    fn forward_reference_resolves() {
        let g = read_svg(&doc_50(r###"<use href="#late"/><defs><rect id="late" x="0" y="0" width="10" height="10"/></defs>"###)).unwrap();
        assert_eq!(g.len(), 4, "forward-referenced definition not replayed: {g:?}");
    }

    /// A self-closing `<defs/>` must not leave the definition scope open, or the next shape
    /// would be swallowed as a definition and never cut.
    #[test]
    fn self_closing_defs_does_not_swallow_following_shapes() {
        let g = read_svg(&doc_50(r###"<defs/><rect x="0" y="0" width="10" height="10"/>"###)).unwrap();
        assert_eq!(g.len(), 4, "shape after <defs/> was swallowed: {g:?}");
    }

    /// A dangling reference is a broken document. Loading it as an empty design would let the
    /// operator send a blank job to the cutter without noticing.
    #[test]
    fn dangling_use_is_an_error() {
        match read_svg(&doc_50(r###"<use href="#missing"/>"###)) {
            Err(IoError::Invalid(m)) => assert!(m.contains("missing definition"), "{m}"),
            other => panic!("expected a missing-definition error, got {other:?}"),
        }
    }

    /// `xlink:href` is the pre-SVG2 spelling and still appears in Inkscape output.
    #[test]
    fn xlink_href_is_accepted() {
        let g = read_svg(&doc_50(r###"<defs><rect id="a" x="0" y="0" width="10" height="10"/></defs><use xlink:href="#a"/>"###)).unwrap();
        assert_eq!(g.len(), 4, "xlink:href use not replayed: {g:?}");
    }

    /// Colour is reported alongside geometry so a caller can split a multi-colour design into
    /// per-colour cutting jobs. The two vectors stay parallel: `colors[i]` belongs to
    /// `geometry[i]`.
    #[test]
    fn colour_is_recorded_per_entity() {
        let doc = doc_50(
            r###"<g stroke="#ff0000"><rect x="0" y="0" width="10" height="10"/></g>
                 <circle cx="30" cy="10" r="4" fill="#0000ff"/>"###,
        );
        let (geometry, colors) = read_svg_colored(&doc).unwrap();
        assert_eq!(geometry.len(), colors.len(), "geometry and colour vectors must stay parallel");
        // The rect's four lines inherit the group's red stroke.
        for (g, c) in geometry.iter().zip(colors.iter()).take(4) {
            assert!(matches!(g, Geom2::Line(..)), "{g:?}");
            assert_eq!(c.as_deref(), Some("#ff0000"), "rect line lost its inherited stroke");
        }
        // The circle carries its own fill, and inherits nothing from the group.
        let i = geometry.iter().position(|g| matches!(g, Geom2::Circle(..))).expect("circle missing");
        assert_eq!(colors[i].as_deref(), Some("#0000ff"), "circle lost its fill colour");
    }

    /// A cutter follows the stroke, so a shape with both a stroke and a fill is cut in the
    /// stroke's colour. `fill` only supplies a colour when no stroke is declared.
    #[test]
    fn stroke_wins_over_fill() {
        let doc = doc_50(r###"<path d="M0 0 L10 0 L10 10" stroke="red" fill="blue"/>"###);
        let (geometry, colors) = read_svg_colored(&doc).unwrap();
        assert!(!geometry.is_empty(), "path produced no geometry");
        for c in &colors {
            assert_eq!(c.as_deref(), Some("red"), "stroke should win over fill");
        }
    }

    /// Properties written into an inline `style` attribute count the same as presentation
    /// attributes. Illustrator and Inkscape both emit style strings.
    #[test]
    fn inline_style_is_read() {
        let doc = doc_50(r###"<rect x="0" y="0" width="10" height="10" style="fill:none;stroke:#00ff00"/>"###);
        let (geometry, colors) = read_svg_colored(&doc).unwrap();
        assert_eq!(geometry.len(), 4);
        for c in &colors {
            assert_eq!(c.as_deref(), Some("#00ff00"), "inline style stroke not read");
        }
    }

    /// `stroke="none"` means "draw no line"; reporting it as a colour would put an empty vinyl
    /// run in front of the cutter.
    #[test]
    fn none_is_not_a_colour() {
        let doc = doc_50(r###"<rect x="0" y="0" width="10" height="10" stroke="none"/>"###);
        let (_, colors) = read_svg_colored(&doc).unwrap();
        assert!(colors.iter().all(|c| c.is_none()), "none leaked as a colour: {colors:?}");
    }

    /// A definition drawn in one colour stays that colour wherever a `<use>` replays it. Losing
    /// this would attribute a red symbol to whatever colour the use site inherits.
    #[test]
    fn use_keeps_definition_colour() {
        let doc = doc_50(r###"<defs><rect id="a" x="0" y="0" width="10" height="10" stroke="#123456"/></defs><use href="#a"/>"###);
        let (geometry, colors) = read_svg_colored(&doc).unwrap();
        assert_eq!(geometry.len(), 4, "definition not replayed");
        for c in &colors {
            assert_eq!(c.as_deref(), Some("#123456"), "definition colour lost at use site");
        }
    }

    /// A group's colour must reach geometry nested two levels down, and must not leak to a
    /// sibling group that declares its own.
    #[test]
    fn colour_inherits_through_nested_groups() {
        let doc = doc_50(
            r###"<g stroke="#111111"><g><rect x="0" y="0" width="4" height="4"/></g></g>
                 <g stroke="#222222"><rect x="10" y="0" width="4" height="4"/></g>"###,
        );
        let (geometry, colors) = read_svg_colored(&doc).unwrap();
        assert_eq!(geometry.len(), 8, "two rects expected");
        for (i, c) in colors.iter().enumerate() {
            let expected = if i < 4 { "#111111" } else { "#222222" };
            assert_eq!(c.as_deref(), Some(expected), "entity {i} has the wrong colour");
        }
    }

    /// `read_svg` must keep returning geometry alone so existing callers are unaffected by the
    /// added colour tracking.
    #[test]
    fn geometry_only_reader_is_unchanged() {
        let doc = doc_50(r###"<rect x="0" y="0" width="10" height="10" stroke="#ff0000"/>"###);
        let plain = read_svg(&doc).unwrap();
        let (colored, colors) = read_svg_colored(&doc).unwrap();
        assert_eq!(plain.len(), colored.len());
        assert!(colors.iter().any(|c| c.is_some()), "colour not recorded");
    }
}
