//! Polygon offset (outline) for vinyl cutting.
//!
//! An outline tool grows or shrinks a closed contour by a distance. A sign shop
//! uses it two ways: to make a cut border N millimetres outside a design (the classic
//! "sticker outline"), and to make an inline contour inside it. It is also the foundation of
//! contour cutting around printed graphics, where the cut line has to run around the printed
//! image rather than on its own edge.
//!
//! # What this does, and what it does not
//!
//! It offsets ONE closed contour. The outline of text, or of any graphic built from several
//! contours, needs the union of the individual offsets — boolean geometry that is not
//! implemented here. A multi-contour design is offset contour by contour and the results cut
//! as separate paths, which is what a vinyl cutter does anyway: it follows one closed path at
//! a time.
//!
//! # Method
//!
//! Every edge is shifted `distance` along its outward normal. At each vertex the two shifted
//! edges either diverge or overlap, and the sign of the corner's turn against the sign of the
//! distance decides which:
//!
//! - **Diverge** (a convex corner grown outward, or a concave corner shrunk inward): an arc of
//!   radius `|distance|` centred on the vertex fills the gap. Round corners are what
//!   a typical sign-shop outline produces, and they are what a drag knife follows most
//!   smoothly — a sharp spike catches the blade.
//! - **Overlap** (the opposite cases): the two shifted edges cross, so they are trimmed to
//!   their intersection, exactly as a miter join would.
//!
//! The arcs are flattened to short straight segments, because this layer only carries
//! `Segment::Line`; curves are flattened before they reach the CAM crate by convention.
//!
//! The polygon's winding is normalised to counter-clockwise for the maths and restored on
//! output, so a caller that hands in a clockwise contour gets a clockwise contour back. The
//! winding matters when the toolpath is cut (a drag knife enters from a particular side), so
//! silently flipping it would be a trap.

use crate::geom::{Point, Segment, Toolpath};

/// Arc approximation step in radians. A corner is rounded in steps no larger than this, so a
/// 90 degree corner becomes nine short segments — smooth enough for a knife, coarse enough
/// not to bloat the command stream.
const ARC_STEP: f64 = std::f64::consts::PI / 18.0;

/// Two points closer than this are treated as one, in millimetres. A cutter cannot resolve
/// anything finer, and a zero-length segment would confuse the post-processor.
const EPS: f64 = 1e-9;

/// Offset a closed contour by `distance` millimetres.
///
/// Positive grows the contour outward (away from its interior), negative shrinks it inward.
/// The result is a single closed `Toolpath`, or empty when the contour collapses — an inward
/// offset larger than the contour can absorb — or the input is degenerate (fewer than three
/// distinct vertices, zero area, or not closed).
///
/// Corners are rounded with arcs of radius `|distance|`. The input's winding is preserved.
///
/// Curves are flattened at [`DEFAULT_FLATTEN_TOL_MM`](crate::geom::DEFAULT_FLATTEN_TOL_MM)
/// before offsetting. A caller that knows the process's own tolerance — a torch is happy with
/// coarser chords than a knife — should use [`offset_closed_with_tolerance`].
pub fn offset_closed(toolpath: &Toolpath, distance: f64) -> Vec<Toolpath> {
    offset_closed_with_tolerance(toolpath, distance, crate::geom::DEFAULT_FLATTEN_TOL_MM)
}

/// [`offset_closed`] with the chord tolerance the caller's process needs.
pub fn offset_closed_with_tolerance(toolpath: &Toolpath, distance: f64, flatten_tol_mm: f64) -> Vec<Toolpath> {
    // A zero offset is the identity. Returning the input unchanged is both correct and the
    // least surprising thing to do.
    if distance.abs() < EPS {
        return if toolpath.closed && toolpath.segments.len() >= 3 { vec![toolpath.clone()] } else { Vec::new() };
    }
    if !toolpath.closed {
        return Vec::new();
    }

    // Offset reasons about a polygon, so a curve has to become chords first; taking a
    // segment's start point alone would drop the whole curve.
    let flat = toolpath.flattened(flatten_tol_mm.max(0.0));
    let mut pts = contour_points(&flat);
    dedupe(&mut pts);
    if pts.len() < 3 {
        return Vec::new();
    }

    // Normalise to counter-clockwise so the outward normal has one known sign. Remember the
    // original winding so the result can be handed back in the same direction.
    let mut area = signed_area(&pts);
    if area.abs() < EPS {
        return Vec::new();
    }
    let was_cw = area < 0.0;
    if was_cw {
        pts.reverse();
        // Now counter-clockwise, so the area that the output is compared against is positive.
        area = -area;
    }

    let n = pts.len();
    let mut out_pts: Vec<Point> = Vec::new();
    // Where each vertex's join geometry starts and ends in `out_pts`, so the offset edges
    // between consecutive joins can be checked for reversal once the ring is built.
    let mut corner_ranges: Vec<(usize, usize)> = Vec::with_capacity(n);
    let mut spike = vec![false; n];
    for i in 0..n {
        let prev = pts[(i + n - 1) % n];
        let curr = pts[i];
        let next = pts[(i + 1) % n];

        // `dedupe` removed zero-length edges, so these should always succeed. If one does not,
        // skip the vertex and the ring's corner count will not match `n`, which disables the
        // reversal check rather than letting it index the wrong corner.
        let start = out_pts.len();
        let Some(d_in) = normalize(sub(curr, prev)) else {
            continue;
        };
        let Some(d_out) = normalize(sub(next, curr)) else {
            continue;
        };

        // Outward normals for a counter-clockwise polygon: rotate the edge direction -90°.
        let n_in = Point::new(d_in.y, -d_in.x);
        let n_out = Point::new(d_out.y, -d_out.x);

        // Where this vertex's two shifted edges start and end.
        let a = add(curr, scale(n_in, distance));
        let b = add(curr, scale(n_out, distance));

        // `turn_cross` rather than `cross`: the name would shadow the helper below.
        let turn_cross = cross(d_in, d_out);
        let dot = d_in.x * d_out.x + d_in.y * d_out.y;

        // A straight vertex needs no join: both shifted edges share one point.
        if turn_cross.abs() < 1e-12 {
            if dot > 0.0 {
                // Collinear, same direction — one point.
                out_pts.push(a);
            } else {
                // A 180° reversal doubles the contour back on itself. There is no sensible
                // offset for it, but dropping the vertex would lose geometry, so keep both
                // ends and let the cut follow the spike. Degenerate input, but honest output.
                spike[i] = true;
                out_pts.push(a);
                out_pts.push(b);
            }
        } else {
            // Diverge when the turn and the distance have the same sign; overlap when they
            // differ. A left turn is a convex corner, so growing (positive) leaves a gap there;
            // shrinking (negative) leaves a gap at a right turn instead. One test covers both.
            let diverges = (turn_cross > 0.0) == (distance > 0.0);

            if diverges {
                // Round the corner: sweep from the incoming normal to the outgoing one, staying
                // at radius |distance| around the vertex. The sweep is the signed turn angle.
                let sweep = turn_cross.atan2(dot);
                push_arc(&mut out_pts, curr, scale(n_in, distance), sweep);
            } else {
                // Miter: the two shifted edges cross. Find that point. The lines are:
                //   a + d_in * t   and   b + d_out * s
                // Solving gives t = cross(b - a, d_out) / cross(d_in, d_out).
                let t = cross(sub(b, a), d_out) / turn_cross;
                let edge_len = length(sub(next, curr)).max(length(sub(curr, prev)));
                // A huge t means the edges are nearly parallel and the intersection shoots off
                // to infinity. Falling back to the vertex's own shifted point keeps the outline
                // sane instead of adding a spike the knife would chase.
                if t.is_finite() && t.abs() <= edge_len.max(1.0) * 1e3 {
                    out_pts.push(add(a, scale(d_in, t)));
                } else {
                    out_pts.push(a);
                }
            }
        }
        corner_ranges.push((start, out_pts.len().saturating_sub(1)));
    }

    dedupe(&mut out_pts);
    if out_pts.len() < 3 {
        return Vec::new();
    }

    // An inward offset that overshoots inverts the contour. Detect it by the signed area
    // flipping sign or shrinking to nothing; both mean there is no contour left to cut.
    let new_area = signed_area(&out_pts);
    if new_area.abs() < EPS || (new_area < 0.0) != (area < 0.0) {
        return Vec::new();
    }

    // For an inward offset the area test alone is not enough. A square shrunk past its own
    // half-width collapses to a smaller square of the SAME winding — area 100 mm² becomes
    // 4 mm², still positive — yet the contour has turned inside out and its cut order is
    // backwards. The tell is an offset edge running opposite to the edge it came from, so
    // each vertex's join geometry is compared against the original edge between it and the
    // next vertex. A reversal means the shifted edges crossed each other: no contour left.
    if distance < 0.0 && corner_ranges.len() == n {
        for i in 0..n {
            let j = (i + 1) % n;
            if spike[i] || spike[j] {
                continue;
            }
            let from = out_pts[corner_ranges[i].1];
            let to = out_pts[corner_ranges[j].0];
            let edge = sub(to, from);
            let Some(orig) = normalize(sub(pts[j], pts[i])) else {
                continue;
            };
            if edge.x * orig.x + edge.y * orig.y < 0.0 {
                return Vec::new();
            }
        }
    }

    if was_cw {
        out_pts.reverse();
    }

    vec![closed_toolpath(out_pts)]
}

/// The distinct vertices of a closed toolpath, in order.
///
/// A toolpath's vertices are the starts of its segments. For a closed path the last segment's
/// end should return to the first start; the duplicate is dropped by [`dedupe`].
fn contour_points(toolpath: &Toolpath) -> Vec<Point> {
    toolpath.segments.iter().map(|s| s.start()).collect()
}

/// Remove consecutive duplicate points, including a duplicate between the last and first.
fn dedupe(pts: &mut Vec<Point>) {
    pts.dedup_by(|a, b| near(*a, *b));
    while pts.len() > 1 && pts.first().zip(pts.last()).is_some_and(|(f, l)| near(*f, *l)) {
        pts.pop();
    }
}

/// Append points along the circular arc from `start` (a vector from `centre`) sweeping by
/// `sweep` radians around `centre`, staying at the start vector's length.
///
/// The first point (the arc's start) is included; the caller has not emitted it yet. A zero
/// sweep emits the single point.
fn push_arc(out: &mut Vec<Point>, centre: Point, start: Point, sweep: f64) {
    let radius = length(start);
    if radius < EPS {
        out.push(centre);
        return;
    }
    let steps = ((sweep.abs() / ARC_STEP).ceil() as usize).max(1);
    let step = sweep / steps as f64;
    for k in 0..=steps {
        let angle = step * k as f64;
        let v = rotate(start, angle);
        out.push(add(centre, v));
    }
}

/// Build a closed toolpath from an ordered vertex ring.
fn closed_toolpath(pts: Vec<Point>) -> Toolpath {
    let n = pts.len();
    let mut segments = Vec::with_capacity(n);
    for i in 0..n {
        let from = pts[i];
        let to = pts[(i + 1) % n];
        if !near(from, to) {
            segments.push(Segment::Line { a: from, b: to });
        }
    }
    Toolpath { segments, closed: true }
}

/// The signed area of a vertex ring. Positive means counter-clockwise.
fn signed_area(pts: &[Point]) -> f64 {
    let n = pts.len();
    let mut a = 0.0;
    for i in 0..n {
        let p = pts[i];
        let q = pts[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    a * 0.5
}

// --- small vector helpers -------------------------------------------------------------

fn sub(a: Point, b: Point) -> Point {
    Point::new(a.x - b.x, a.y - b.y)
}

fn add(a: Point, b: Point) -> Point {
    Point::new(a.x + b.x, a.y + b.y)
}

fn scale(v: Point, k: f64) -> Point {
    Point::new(v.x * k, v.y * k)
}

fn length(v: Point) -> f64 {
    v.x.hypot(v.y)
}

fn normalize(v: Point) -> Option<Point> {
    let len = length(v);
    if len < EPS { None } else { Some(Point::new(v.x / len, v.y / len)) }
}

fn cross(a: Point, b: Point) -> f64 {
    a.x * b.y - a.y * b.x
}

/// Rotate `v` by `angle` radians counter-clockwise.
fn rotate(v: Point, angle: f64) -> Point {
    let (s, c) = angle.sin_cos();
    Point::new(v.x * c - v.y * s, v.x * s + v.y * c)
}

fn near(a: Point, b: Point) -> bool {
    (a.x - b.x).abs() < EPS && (a.y - b.y).abs() < EPS
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Axis-aligned bounds of a toolpath, as `(min, max)`.
    fn bounds(tp: &Toolpath) -> (Point, Point) {
        let mut min = Point::new(f64::INFINITY, f64::INFINITY);
        let mut max = Point::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for seg in &tp.segments {
            for p in [seg.start(), seg.end()] {
                min.x = min.x.min(p.x);
                min.y = min.y.min(p.y);
                max.x = max.x.max(p.x);
                max.y = max.y.max(p.y);
            }
        }
        (min, max)
    }

    /// A 10 x 10 square with its bottom-left corner at the origin, counter-clockwise.
    fn square() -> Toolpath {
        Toolpath::rectangle(0.0, 0.0, 10.0, 10.0)
    }

    #[test]
    fn zero_offset_is_the_identity() {
        let r = offset_closed(&square(), 0.0);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0], square());
    }

    #[test]
    fn open_toolpath_is_refused() {
        let mut tp = square();
        tp.closed = false;
        assert!(offset_closed(&tp, 1.0).is_empty());
    }

    #[test]
    fn outward_offset_grows_the_bounding_box() {
        let r = offset_closed(&square(), 1.0);
        assert_eq!(r.len(), 1);
        let (min, max) = bounds(&r[0]);
        // The straight edges shift out by 1 mm; the rounded corners stay within that box.
        assert!((min.x + 1.0).abs() < 1e-6, "min.x = {}", min.x);
        assert!((min.y + 1.0).abs() < 1e-6, "min.y = {}", min.y);
        assert!((max.x - 11.0).abs() < 1e-6, "max.x = {}", max.x);
        assert!((max.y - 11.0).abs() < 1e-6, "max.y = {}", max.y);
    }

    #[test]
    fn inward_offset_shrinks_the_bounding_box() {
        let r = offset_closed(&square(), -1.0);
        assert_eq!(r.len(), 1);
        let (min, max) = bounds(&r[0]);
        assert!((min.x - 1.0).abs() < 1e-6, "min.x = {}", min.x);
        assert!((min.y - 1.0).abs() < 1e-6, "min.y = {}", min.y);
        assert!((max.x - 9.0).abs() < 1e-6, "max.x = {}", max.x);
        assert!((max.y - 9.0).abs() < 1e-6, "max.y = {}", max.y);
    }

    #[test]
    fn inward_offset_past_collapse_returns_empty() {
        // Half the 10 mm width is 5 mm, so 6 mm inward has no contour left.
        assert!(offset_closed(&square(), -6.0).is_empty());
    }

    #[test]
    fn outward_offset_preserves_area_sign() {
        let r = offset_closed(&square(), 2.0);
        assert_eq!(r.len(), 1);
        // Grown, so the area is greater than the original 100 mm².
        let a = signed_area(&r[0].segments.iter().map(|s| s.start()).collect::<Vec<_>>());
        assert!(a > 100.0, "area = {a}");
    }

    #[test]
    fn clockwise_input_returns_clockwise_output() {
        // A clockwise square: reverse the counter-clockwise one.
        let mut tp = square();
        tp.segments.reverse();
        for s in &mut tp.segments {
            // `reversed()` covers every segment kind: an arc flips its sweep rather than
            // being skipped by a pattern that only matched lines.
            *s = s.reversed();
        }
        let before = signed_area(&tp.segments.iter().map(|s| s.start()).collect::<Vec<_>>());
        assert!(before < 0.0);
        let r = offset_closed(&tp, 1.0);
        assert_eq!(r.len(), 1);
        let after = signed_area(&r[0].segments.iter().map(|s| s.start()).collect::<Vec<_>>());
        assert!(after < 0.0, "winding flipped: {after}");
    }

    #[test]
    fn offset_keeps_the_contour_closed() {
        let r = offset_closed(&square(), 0.5);
        assert_eq!(r.len(), 1);
        let tp = &r[0];
        assert!(tp.closed);
        // Every segment's end must be the next segment's start.
        let n = tp.segments.len();
        for i in 0..n {
            assert!(near(tp.segments[i].end(), tp.segments[(i + 1) % n].start()), "gap between segment {i} and {}", (i + 1) % n);
        }
    }

    #[test]
    fn degenerate_inputs_are_refused() {
        // Two points.
        let tp = Toolpath {
            segments: vec![
                Segment::Line { a: Point::new(0.0, 0.0), b: Point::new(1.0, 0.0) },
                Segment::Line { a: Point::new(1.0, 0.0), b: Point::new(0.0, 0.0) },
            ],
            closed: true,
        };
        assert!(offset_closed(&tp, 1.0).is_empty());
        // Zero area: a line traced out and back.
        let tp = Toolpath {
            segments: vec![
                Segment::Line { a: Point::new(0.0, 0.0), b: Point::new(10.0, 0.0) },
                Segment::Line { a: Point::new(10.0, 0.0), b: Point::new(0.0, 0.0) },
                Segment::Line { a: Point::new(0.0, 0.0), b: Point::new(5.0, 0.0) },
            ],
            closed: true,
        };
        assert!(offset_closed(&tp, 1.0).is_empty());
    }

    #[test]
    fn offset_polygon_is_larger_by_the_distance() {
        // The area of a square grown by d on all sides is (10 + 2d)², ignoring the corner
        // rounding. With round corners the area is that minus the four corner cut-offs:
        // each corner loses a d x d square and gains a quarter circle of radius d.
        let d = 2.0;
        let r = offset_closed(&square(), d);
        let pts: Vec<Point> = r[0].segments.iter().map(|s| s.start()).collect();
        let area = signed_area(&pts);
        let straight = (10.0 + 2.0 * d) * (10.0 + 2.0 * d);
        let corner_loss = 4.0 * (d * d - std::f64::consts::PI * d * d / 4.0);
        let expected = straight - corner_loss;
        assert!((area - expected).abs() < 0.5, "area {area} vs expected {expected}");
    }
}
