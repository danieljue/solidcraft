//! 2D polygon boolean operations for vinyl cutting.
//!
//! The sign-shop design tools — Weld, Trim, Punch, Intersect, Exclude — all reduce to a boolean
//! operation on filled regions. A sign shop reaches for them constantly: welding overlapping
//! lettering into one cuttable outline, punching a hole in a shape, trimming artwork to a panel.
//! Without them a cutter follows every contour literally and cuts the overlap twice, which scores
//! the vinyl and leaves a visible seam under the transfer tape.
//!
//! # Model
//!
//! A region is described by its contours. Which contours are solid and which are holes is decided
//! by the **fill mode**, not by list order — the geometry engine flattens shape boundaries and
//! treats every contour as a candidate boundary, exactly as a vector editor's fill does. That is
//! why [`FillMode`] is a parameter rather than something inferred:
//!
//! - [`FillMode::NonZero`] (the default) fills where the winding number is non-zero. An outer
//!   contour and a hole wound the opposite way produce a ring; two contours wound the same way
//!   merge into their union. This matches the TrueType/OpenType convention that text outlines
//!   already follow, so welded text needs no extra work.
//! - [`FillMode::EvenOdd`] fills where the winding number is odd. Nested contours alternate
//!   solid and hole regardless of direction. Use this for imported artwork whose winding is not
//!   consistent — SVG and DXF rarely normalise it — accepting that overlapping same-direction
//!   contours will cancel in the overlap rather than merge.
//!
//! # What this does, and what it does not
//!
//! Every input contour must be **closed**: an open polyline has no interior, so there is nothing
//! to fill. Open paths are skipped and counted in the return value rather than silently treated as
//! zero-area regions. Curves must already be flattened to line segments — the CAM layer carries
//! only `Segment::Line`, and the caller flattens before reaching here.
//!
//! The output is one closed `Toolpath` per resulting contour. A ring comes back as two toolpaths,
//! an outer and its hole, because that is how a cutter follows a ring: it cuts the outer outline,
//! lifts, moves, and cuts the hole. Grouping them into a single "shape" would lose that.

use crate::geom::{Point, Segment, Toolpath};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;

/// Two points closer than this are the same point, in millimetres. A cutter cannot resolve
/// anything finer, and i_overlay rejects contours that repeat a vertex.
const EPS: f64 = 1e-9;

/// Which boolean operation to perform. The names follow what a sign maker calls the result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOp {
    /// Merge everything into one filled outline. Sign shops call this **Weld**.
    Union,
    /// Subtract the clip from the subject. Sign shops call this **Trim** or **Punch**.
    Difference,
    /// Keep only the region both shapes share. Sign shops call this **Intersect**.
    Intersect,
    /// Keep only the regions unique to each shape. Sign shops call this **Exclude**.
    Xor,
}

impl BooleanOp {
    /// The engine's name for this operation. Kept private: [`BooleanOp`] is the public vocabulary
    /// and should not leak the dependency's enum into every caller's match arms.
    fn rule(self) -> OverlayRule {
        match self {
            BooleanOp::Union => OverlayRule::Union,
            BooleanOp::Difference => OverlayRule::Difference,
            BooleanOp::Intersect => OverlayRule::Intersect,
            BooleanOp::Xor => OverlayRule::Xor,
        }
    }
}

/// How overlapping and nested contours combine into filled regions. See the module docs for the
/// trade-off between the two; `NonZero` is the default because it matches how glyph outlines are
/// wound and how vector editors fill by default.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FillMode {
    /// Non-zero winding. Nested contours must wind oppositely to make a hole. Overlapping contours
    /// of the same winding merge. The default.
    #[default]
    NonZero,
    /// Even-odd winding. Nested contours alternate solid and hole regardless of winding.
    EvenOdd,
}

impl FillMode {
    fn rule(self) -> FillRule {
        match self {
            FillMode::NonZero => FillRule::NonZero,
            FillMode::EvenOdd => FillRule::EvenOdd,
        }
    }
}

/// A single closed contour as a plain point list, or `None` when the toolpath is not closed or has
/// too few distinct points to enclose an area.
///
/// The closing vertex is dropped when it duplicates the first: i_overlay closes contours itself,
/// and a repeated vertex would create a zero-length edge in its sweep.
fn contour_points(toolpath: &Toolpath) -> Option<Vec<[f64; 2]>> {
    if !toolpath.closed {
        return None;
    }
    // A closed toolpath's segments already form a cycle — the last end meets the first start — so
    // the start points are the contour. A toolpath whose final end does not meet its first start
    // is malformed; closing it anyway is kinder than rejecting the whole design.
    let mut pts: Vec<[f64; 2]> = toolpath
        .segments
        .iter()
        .map(|s| {
            let p = s.start();
            [p.x, p.y]
        })
        .collect();
    if let Some(last) = toolpath.segments.last() {
        let e = last.end();
        if pts.is_empty() || (e.x - pts[0][0]).abs() > EPS || (e.y - pts[0][1]).abs() > EPS {
            pts.push([e.x, e.y]);
        }
    }
    // Drop consecutive duplicates. A zero-length edge has no direction, and the sweep treats it as
    // a degenerate event that can produce a spurious vertex.
    pts.dedup_by(|a, b| (a[0] - b[0]).abs() < EPS && (a[1] - b[1]).abs() < EPS);
    // A ring closed by an explicit duplicate has that duplicate at both ends after the checks
    // above; remove it so the count is the distinct vertex count.
    while pts.len() > 1 {
        let first = pts[0];
        let last = pts[pts.len() - 1];
        if (first[0] - last[0]).abs() < EPS && (first[1] - last[1]).abs() < EPS {
            pts.pop();
        } else {
            break;
        }
    }
    if pts.len() < 3 {
        return None;
    }
    Some(pts)
}

/// Convert the contours the engine returned into closed toolpaths.
///
/// The engine returns each contour without repeating its first vertex — it is implicitly closed.
/// Rebuilding the ring here means every caller gets a toolpath whose last segment lands on its
/// first point, which is what the post-processor and the cutter expect.
fn to_toolpaths(shapes: Vec<Vec<Vec<[f64; 2]>>>) -> Vec<Toolpath> {
    let mut out = Vec::new();
    for shape in shapes {
        for contour in shape {
            if contour.len() < 3 {
                continue;
            }
            let mut segments = Vec::with_capacity(contour.len());
            for i in 0..contour.len() {
                let a = contour[i];
                let b = contour[(i + 1) % contour.len()];
                segments.push(Segment::Line { a: Point::new(a[0], a[1]), b: Point::new(b[0], b[1]) });
            }
            out.push(Toolpath { segments, closed: true });
        }
    }
    out
}

/// Apply a boolean operation to two collections of closed contours.
///
/// `subject` is the geometry being operated on; `clip` is the second operand. An empty `clip` with
/// [`BooleanOp::Union`] cleans the subject against itself — resolving self-intersections and
/// merging overlapping contours — which is exactly what "weld this design" means. Use
/// [`weld`] for that case as it reads more clearly.
///
/// Open contours in either input are skipped: they enclose no area. The return value is one closed
/// `Toolpath` per resulting contour, in the order the engine emits them. Winding of the output
/// follows the engine's convention (outer counter-clockwise, holes clockwise), which a cutter
/// handles either way.
///
/// Returns an empty vector when the operation leaves nothing — a difference that removes the whole
/// subject, for example — rather than erroring. "Nothing survived" is a legitimate design outcome.
pub fn boolean_paths(subject: &[Toolpath], clip: &[Toolpath], op: BooleanOp, fill: FillMode) -> Vec<Toolpath> {
    // `contour_points` reads segment start points, so curves must be chords before it runs.
    let subject_flat: Vec<Toolpath> = subject.iter().map(|t| t.flattened(crate::geom::DEFAULT_FLATTEN_TOL_MM)).collect();
    let clip_flat: Vec<Toolpath> = clip.iter().map(|t| t.flattened(crate::geom::DEFAULT_FLATTEN_TOL_MM)).collect();
    let subj: Vec<Vec<[f64; 2]>> = subject_flat.iter().filter_map(contour_points).collect();
    if subj.is_empty() {
        return Vec::new();
    }
    let clip_contours: Vec<Vec<[f64; 2]>> = clip_flat.iter().filter_map(contour_points).collect();
    let shapes = subj.overlay(&clip_contours, op.rule(), fill.rule());
    to_toolpaths(shapes)
}

/// Weld a design against itself: merge overlapping contours and resolve self-intersections.
///
/// This is [`boolean_paths`] with an empty clip and [`BooleanOp::Union`], named for the tool a sign
/// maker actually reaches for. Weld is the single most common cleanup step before cutting, because
/// traced artwork and overlapping lettering otherwise cut their overlaps twice.
pub fn weld(paths: &[Toolpath], fill: FillMode) -> Vec<Toolpath> {
    boolean_paths(paths, &[], BooleanOp::Union, fill)
}

/// Net signed area of a closed contour in square millimetres, positive when counter-clockwise.
///
/// Exposed because callers need to check whether a boolean operation kept or removed a region, and
/// counting contours is not enough: a hole and an outer ring are both one contour. This is the
/// shoelace formula over the contour's vertices.
pub fn signed_area(toolpath: &Toolpath) -> f64 {
    let Some(pts) = contour_points(toolpath) else {
        return 0.0;
    };
    let n = pts.len();
    let mut sum = 0.0;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    sum * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Axis-aligned rectangle as a closed toolpath. `Toolpath::rectangle` winds counter-clockwise.
    fn rect(x: f64, y: f64, w: f64, h: f64) -> Toolpath {
        Toolpath::rectangle(x, y, w, h)
    }

    /// The same rectangle wound clockwise, for building holes under [`FillMode::NonZero`].
    fn rect_cw(x: f64, y: f64, w: f64, h: f64) -> Toolpath {
        let mut tp = Toolpath::rectangle(x, y, w, h);
        // Reverse the ring: swap each segment's ends and reverse the order. A clockwise ring is a
        // hole when it sits inside a counter-clockwise one.
        tp.segments.reverse();
        for s in &mut tp.segments {
            // `reversed()` covers every segment kind: an arc flips its sweep rather than
            // being skipped by a pattern that only matched lines.
            *s = s.reversed();
        }
        tp
    }

    /// Net filled area. Sums *signed* areas, so a hole (wound clockwise, negative) subtracts from
    /// its outer ring instead of adding to it. Summing absolute values would report a ring as
    /// outer-plus-hole and hide the very mistake these tests exist to catch.
    fn net_area(paths: &[Toolpath]) -> f64 {
        paths.iter().map(signed_area).sum()
    }

    #[test]
    fn overlapping_rectangles_union_into_one_outline() {
        // 4x4 at the origin and 4x4 shifted 2 right: they overlap in a 2x4 band.
        // Union area = 16 + 16 - 8 = 24, one contour.
        let a = rect(0.0, 0.0, 4.0, 4.0);
        let b = rect(2.0, 0.0, 4.0, 4.0);
        let out = boolean_paths(&[a, b], &[], BooleanOp::Union, FillMode::NonZero);
        assert_eq!(out.len(), 1, "overlapping rectangles should weld to one contour");
        assert!((net_area(&out) - 24.0).abs() < 0.01, "union area should be 24, got {}", net_area(&out));
    }

    #[test]
    fn disjoint_rectangles_union_keeps_both() {
        let a = rect(0.0, 0.0, 4.0, 4.0);
        let b = rect(10.0, 0.0, 4.0, 4.0);
        let out = boolean_paths(&[a, b], &[], BooleanOp::Union, FillMode::NonZero);
        assert_eq!(out.len(), 2, "disjoint rectangles should stay separate");
        assert!((net_area(&out) - 32.0).abs() < 0.01);
    }

    #[test]
    fn difference_punches_the_clip_out() {
        // 4x4 minus a 2x4 band starting at x=2 leaves a 2x4 strip: area 8.
        let subj = rect(0.0, 0.0, 4.0, 4.0);
        let clip = rect(2.0, 0.0, 2.0, 4.0);
        let out = boolean_paths(&[subj], &[clip], BooleanOp::Difference, FillMode::NonZero);
        assert_eq!(out.len(), 1);
        assert!((net_area(&out) - 8.0).abs() < 0.01, "difference area should be 8, got {}", net_area(&out));
    }

    #[test]
    fn intersect_keeps_only_the_overlap() {
        let subj = rect(0.0, 0.0, 4.0, 4.0);
        let clip = rect(2.0, 0.0, 2.0, 4.0);
        let out = boolean_paths(&[subj], &[clip], BooleanOp::Intersect, FillMode::NonZero);
        assert_eq!(out.len(), 1);
        assert!((net_area(&out) - 8.0).abs() < 0.01, "intersection area should be 8, got {}", net_area(&out));
    }

    #[test]
    fn xor_keeps_the_non_overlapping_parts() {
        // Two 4x4 rectangles overlapping in a 2x4 band. Xor is the symmetric difference, so it
        // needs one rectangle as the subject and the other as the clip: 16 + 16 - 2*8 = 16.
        // Passing both as subject with an empty clip would just return their union, which is what
        // the first version of this test did and why it reported 24.
        let a = rect(0.0, 0.0, 4.0, 4.0);
        let b = rect(2.0, 0.0, 4.0, 4.0);
        let out = boolean_paths(&[a], &[b], BooleanOp::Xor, FillMode::NonZero);
        assert!((net_area(&out) - 16.0).abs() < 0.01, "xor area should be 16, got {}", net_area(&out));
    }

    #[test]
    fn difference_that_removes_everything_returns_nothing() {
        // The clip covers the subject entirely, so nothing survives. Empty is the honest answer.
        let subj = rect(0.0, 0.0, 4.0, 4.0);
        let clip = rect(-1.0, -1.0, 10.0, 10.0);
        let out = boolean_paths(&[subj], &[clip], BooleanOp::Difference, FillMode::NonZero);
        assert!(out.is_empty(), "a clip covering the subject should leave nothing");
    }

    #[test]
    fn nonzero_hole_needs_opposite_winding() {
        // Outer 0..10 square counter-clockwise, inner 2..8 square clockwise. Non-zero winding
        // cancels inside the hole, leaving a ring of area 100 - 36 = 64 in two contours.
        let outer = rect(0.0, 0.0, 10.0, 10.0);
        let hole = rect_cw(2.0, 2.0, 6.0, 6.0);
        let out = weld(&[outer, hole], FillMode::NonZero);
        assert_eq!(out.len(), 2, "a ring should come back as outer plus hole");
        assert!((net_area(&out) - 64.0).abs() < 0.01, "ring area should be 64, got {}", net_area(&out));
    }

    #[test]
    fn even_odd_makes_a_hole_from_same_winding() {
        // Both squares counter-clockwise. Even-odd fills odd winding numbers, so the inner square
        // becomes a hole even though it winds the same way as the outer.
        let outer = rect(0.0, 0.0, 10.0, 10.0);
        let inner = rect(2.0, 2.0, 6.0, 6.0);
        let out = weld(&[outer, inner], FillMode::EvenOdd);
        assert_eq!(out.len(), 2);
        assert!((net_area(&out) - 64.0).abs() < 0.01, "even-odd ring area should be 64, got {}", net_area(&out));
    }

    #[test]
    fn open_contours_are_skipped() {
        // An open polyline has no interior. Welding it must not invent an area from nothing.
        let open = Toolpath {
            segments: vec![
                Segment::Line { a: Point::new(0.0, 0.0), b: Point::new(4.0, 0.0) },
                Segment::Line { a: Point::new(4.0, 0.0), b: Point::new(4.0, 4.0) },
            ],
            closed: false,
        };
        assert!(weld(&[open], FillMode::NonZero).is_empty());
    }

    #[test]
    fn empty_subject_returns_empty() {
        assert!(weld(&[], FillMode::NonZero).is_empty());
    }

    #[test]
    fn welding_a_single_rectangle_is_identity() {
        // A clean rectangle already has no overlaps or self-intersections, so welding should hand
        // back the same area and one contour. Anything else means the engine is reshaping geometry
        // that was already correct.
        let a = rect(0.0, 0.0, 4.0, 3.0);
        let out = weld(&[a], FillMode::NonZero);
        assert_eq!(out.len(), 1);
        assert!((net_area(&out) - 12.0).abs() < 0.01);
    }

    #[test]
    fn signed_area_is_positive_counter_clockwise() {
        // Toolpath::rectangle walks right, up, left, down — counter-clockwise, positive area.
        let a = rect(0.0, 0.0, 4.0, 3.0);
        assert!((signed_area(&a) - 12.0).abs() < 1e-9, "got {}", signed_area(&a));
        assert!(signed_area(&rect_cw(0.0, 0.0, 4.0, 3.0)) < 0.0);
    }
}
