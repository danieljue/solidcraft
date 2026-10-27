//! Post-processing filters. Geometry changes applied to toolpaths before they reach the
//! emitter. These are protocol-agnostic: they change the path, not the command words.
//!
//! Order matters. `apply` runs them as:
//!   1. `min_line`      drop blobs and zero-length segments
//!   2. `linemerge`     join open contours whose ends meet
//!   3. `blade_offset`  drag-knife corner compensation (changes geometry)
//!   4. `repeat_passes` cut each contour N times in place
//!   5. `linesort`      order contours to minimise pen-up travel
//!
//! Running `min_line` before `blade_offset` means the offset arcs are not pruned;
//! running `linesort` last means the order is not disturbed by later geometry edits.

use crate::geom::{Point, Segment, Toolpath};
use crate::profile::MachineProfile;

fn dist(a: Point, b: Point) -> f64 {
    ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt()
}

/// Unit vector from `a` toward `b`. `None` for a degenerate segment.
fn unit(a: Point, b: Point) -> Option<(f64, f64)> {
    let d = dist(a, b);
    if d < 1e-12 { None } else { Some(((b.x - a.x) / d, (b.y - a.y) / d)) }
}

fn path_points(tp: &Toolpath) -> Vec<Point> {
    let mut pts = Vec::with_capacity(tp.segments.len() + 1);
    if let Some(first) = tp.segments.first() {
        pts.push(first.start());
        for s in &tp.segments {
            pts.push(s.end());
        }
    }
    pts
}

fn reversed(tp: &Toolpath) -> Toolpath {
    let segments = tp.segments.iter().rev().map(|s| Segment::Line { a: s.end(), b: s.start() }).collect();
    Toolpath { segments, closed: tp.closed }
}

fn total_length(pts: &[Point]) -> f64 {
    pts.windows(2).map(|w| dist(w[0], w[1])).sum()
}

/// Drop vertices that create a segment shorter than `min_mm`. A zero-length or sub-mm
/// move draws nothing but still cycles the blade and leaves a blob where the knife
/// pauses. A contour whose whole length is under the threshold is dropped.
pub fn min_line(toolpaths: Vec<Toolpath>, min_mm: f64) -> Vec<Toolpath> {
    if min_mm <= 0.0 {
        return toolpaths;
    }
    toolpaths
        .into_iter()
        .filter_map(|tp| {
            let pts = path_points(&tp);
            if pts.len() < 2 {
                return None;
            }
            let mut kept: Vec<Point> = Vec::with_capacity(pts.len());
            kept.push(pts[0]);
            for p in &pts[1..pts.len() - 1] {
                let gap = kept.last().map_or(0.0, |k| dist(*k, *p));
                if gap >= min_mm {
                    kept.push(*p);
                }
            }
            // Always keep the endpoint: a closed contour must stay closed and an open one
            // must keep the point the user designed.
            let Some(&last) = pts.last() else { return None };
            let gap = kept.last().map_or(0.0, |k| dist(*k, last));
            if gap >= min_mm || tp.closed {
                kept.push(last);
            }
            if kept.len() < 2 || total_length(&kept) < min_mm {
                return None;
            }
            let closed = tp.closed && kept.first().zip(kept.last()).is_some_and(|(f, l)| dist(*f, *l) < 1e-6);
            Some(Toolpath { segments: kept.windows(2).map(|w| Segment::Line { a: w[0], b: w[1] }).collect(), closed })
        })
        .collect()
}

/// Join open contours whose end meets another's start within `tol_mm` into one
/// continuous path. Fewer lifts means fewer puncture points. A closed contour is left
/// alone: merging one would break its seam.
pub fn linemerge(toolpaths: Vec<Toolpath>, tol_mm: f64) -> Vec<Toolpath> {
    if tol_mm <= 0.0 {
        return toolpaths;
    }
    let mut pool: Vec<Option<Toolpath>> = toolpaths.into_iter().map(Some).collect();
    // A closed contour is finished: do not chain anything onto it.
    let mut done: Vec<bool> = pool.iter().map(|tp| tp.as_ref().map_or(true, |t| t.closed)).collect();
    let mut out: Vec<Toolpath> = Vec::new();
    for i in 0..pool.len() {
        if done[i] {
            continue;
        }
        let Some(mut chain) = pool[i].take() else { continue };
        done[i] = true;
        loop {
            let Some(tail) = chain.segments.last().map(|s| s.end()) else {
                break;
            };
            let mut found: Option<(usize, bool)> = None;
            for j in 0..pool.len() {
                if done[j] {
                    continue;
                }
                if let Some(cand) = &pool[j] {
                    if let Some(cs) = cand.segments.first() {
                        if dist(tail, cs.start()) <= tol_mm {
                            found = Some((j, false));
                            break;
                        }
                    }
                    if let Some(ce) = cand.segments.last() {
                        if dist(tail, ce.end()) <= tol_mm {
                            found = Some((j, true));
                            break;
                        }
                    }
                }
            }
            match found {
                Some((j, reverse)) => {
                    let Some(mut cand) = pool[j].take() else { break };
                    done[j] = true;
                    if reverse {
                        cand = reversed(&cand);
                    }
                    // Bridge the gap with a short segment so the chain stays connected.
                    // Bridge the gap between whatever the chain ended on and whatever the
                    // candidate starts with; `end()`/`start()` keep this independent of the
                    // segment kind.
                    let bridge = match (chain.segments.last(), cand.segments.first()) {
                        (Some(last_seg), Some(first)) => {
                            let end = last_seg.end();
                            let begin = first.start();
                            if dist(end, begin) > 1e-9 { Some((end, begin)) } else { None }
                        }
                        _ => None,
                    };
                    if let Some((end_pt, start_pt)) = bridge {
                        chain.segments.push(Segment::Line { a: end_pt, b: start_pt });
                    }
                    chain.segments.extend(cand.segments);
                }
                None => break,
            }
        }
        let closes = chain.segments.first().zip(chain.segments.last()).map_or(false, |(f, l)| dist(f.start(), l.end()) <= tol_mm);
        chain.closed = closes;
        out.push(chain);
    }
    for tp in pool.into_iter().flatten() {
        out.push(tp);
    }
    out
}

/// Cut each contour `passes` times in place: contour A twice, then B twice. Re-cutting a
/// contour while it is still under the blade tracks the same groove, which is what thick
/// material needs. Interleaving (A B A B) would drag the blade across vinyl instead.
pub fn repeat_passes(toolpaths: Vec<Toolpath>, passes: u32) -> Vec<Toolpath> {
    let n = passes.max(1);
    if n == 1 {
        return toolpaths;
    }
    let mut out = Vec::with_capacity(toolpaths.len() * n as usize);
    for tp in toolpaths {
        for _ in 0..n {
            out.push(tp.clone());
        }
    }
    out
}

/// Greedy nearest-neighbour ordering. Start from the current cursor and repeatedly take
/// the contour whose start — or end, reversing when closer — is nearest. Cuts pen-up
/// travel without changing geometry. Jobs with thousands of contours keep their order;
/// the O(n²) scan is not worth it there.
pub fn linesort(toolpaths: Vec<Toolpath>) -> Vec<Toolpath> {
    if toolpaths.len() < 2 || toolpaths.len() > 4000 {
        return toolpaths;
    }
    let mut pool: Vec<Option<Toolpath>> = toolpaths.into_iter().map(Some).collect();
    let total = pool.len();
    let mut out: Vec<Toolpath> = Vec::with_capacity(total);
    let mut cursor = Point::new(0.0, 0.0);
    while out.len() < total {
        let mut best: Option<(usize, bool, f64)> = None;
        for j in 0..pool.len() {
            let Some(cand) = &pool[j] else { continue };
            if let Some(f) = cand.segments.first() {
                let d = dist(cursor, f.start());
                if best.map_or(true, |(_, _, bd)| d < bd) {
                    best = Some((j, false, d));
                }
            }
            if let Some(l) = cand.segments.last() {
                let d = dist(cursor, l.end());
                if best.map_or(true, |(_, _, bd)| d < bd) {
                    best = Some((j, true, d));
                }
            }
        }
        let Some((j, rev, _)) = best else { break };
        let Some(mut cand) = pool[j].take() else { break };
        if rev {
            cand = reversed(&cand);
        }
        if let Some(l) = cand.segments.last() {
            cursor = l.end();
        }
        out.push(cand);
    }
    out
}

/// Drag-knife corner compensation. A drag knife's tip trails the carriage pivot by
/// `offset_mm` along the direction of travel. Commanding the pivot path — the intended
/// tip path shifted forward by the offset — makes the tip follow the design. At each
/// corner the tip must reach the vertex, so the pivot arcs around it while the holder
/// swivels. Straight-segment geometry only: curves are flattened before reaching CAM.
///
/// The offset arcs are flattened into short line segments. The seam of a closed contour
/// is approximate; the profile's `overcut_mm` hides it.
pub fn blade_offset(toolpaths: Vec<Toolpath>, offset_mm: f64, cutoff_deg: f64) -> Vec<Toolpath> {
    if offset_mm <= 0.0 {
        return toolpaths;
    }
    let cutoff = cutoff_deg.to_radians();
    toolpaths
        .into_iter()
        .map(|tp| {
            let pts = path_points(&tp);
            if pts.len() < 2 {
                return tp;
            }
            let closed = tp.closed && pts.first().zip(pts.last()).is_some_and(|(f, l)| dist(*f, *l) < 1e-6);
            // Walk the vertices once. A closed contour repeats its first point at the end;
            // drop the duplicate and close the pivot loop explicitly.
            let work: Vec<Point> = if closed { pts[..pts.len() - 1].to_vec() } else { pts.clone() };
            let n = work.len();
            if n < 2 {
                return tp;
            }
            let dir = |a: Point, b: Point| unit(a, b).unwrap_or((0.0, 1.0));

            let mut pivot: Vec<Point> = Vec::with_capacity(n + 16);
            // First pivot: lead the tip along the first edge.
            let (ux, uy) = dir(work[0], work[1]);
            pivot.push(Point::new(work[0].x + offset_mm * ux, work[0].y + offset_mm * uy));

            let end = if closed { n } else { n - 1 };
            for i in 1..end {
                let prev = work[i - 1];
                let cur = work[i];
                let next = if i + 1 < n { work[i + 1] } else { work[0] };
                let (inx, iny) = dir(prev, cur);
                let (outx, outy) = dir(cur, next);
                // Pivot reaches past the corner along the incoming edge: the tip sits on
                // the vertex at this moment.
                pivot.push(Point::new(cur.x + offset_mm * inx, cur.y + offset_mm * iny));
                let a0 = iny.atan2(inx);
                let a1 = outy.atan2(outx);
                let mut sweep = a1 - a0;
                while sweep > std::f64::consts::PI {
                    sweep -= std::f64::consts::TAU;
                }
                while sweep < -std::f64::consts::PI {
                    sweep += std::f64::consts::TAU;
                }
                if sweep.abs() > cutoff {
                    // ~15° per step keeps the arc smooth without bloating the stream.
                    let steps = ((sweep.abs() / 0.26).ceil() as usize).max(2);
                    for k in 1..steps {
                        let a = a0 + sweep * (k as f64 / steps as f64);
                        pivot.push(Point::new(cur.x + offset_mm * a.cos(), cur.y + offset_mm * a.sin()));
                    }
                }
                pivot.push(Point::new(cur.x + offset_mm * outx, cur.y + offset_mm * outy));
            }

            if closed {
                // Close the pivot loop on its own start so the tip returns to the vertex.
                pivot.push(pivot[0]);
            } else {
                let n1 = work[n - 1];
                let n2 = work[n - 2];
                let (ux, uy) = dir(n2, n1);
                pivot.push(Point::new(n1.x + offset_mm * ux, n1.y + offset_mm * uy));
            }

            Toolpath { segments: pivot.windows(2).map(|w| Segment::Line { a: w[0], b: w[1] }).collect(), closed }
        })
        .collect()
}

/// Run every filter the profile asks for, in the order documented at the top.
pub fn apply(profile: &MachineProfile, toolpaths: Vec<Toolpath>) -> Vec<Toolpath> {
    let mut t = toolpaths;
    if profile.min_line_mm > 0.0 {
        t = min_line(t, profile.min_line_mm);
    }
    if profile.linemerge_tol_mm > 0.0 {
        t = linemerge(t, profile.linemerge_tol_mm);
    }
    // The active tool preset wins over the profile's flat values: a 30-degree blade
    // needs a shorter trail than a 60-degree one, and a pen needs none at all.
    let blade_offset_mm = profile.effective_blade_offset_mm();
    if blade_offset_mm > 0.0 {
        t = blade_offset(t, blade_offset_mm, 5.0);
    }
    let passes = profile.effective_repeat_passes();
    if passes > 1 {
        t = repeat_passes(t, passes);
    }
    linesort(t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Toolpath;

    fn line_path(a: Point, b: Point) -> Toolpath {
        Toolpath { segments: vec![Segment::Line { a: a, b: b }], closed: false }
    }

    #[test]
    fn min_line_collapses_short_segments() {
        // A 0.1 mm jog between two long runs should be collapsed away.
        let pts = vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0), Point::new(10.05, 0.0), Point::new(20.0, 0.0)];
        let tp = Toolpath { segments: pts.windows(2).map(|w| Segment::Line { a: w[0], b: w[1] }).collect(), closed: false };
        let out = min_line(vec![tp], 0.5);
        assert_eq!(out.len(), 1);
        // The 0.05 mm jog is gone; the path is 0 -> 10 -> 20.
        assert_eq!(out[0].segments.len(), 2);
    }

    #[test]
    fn linemerge_joins_contiguous_open_paths() {
        let a = line_path(Point::new(0.0, 0.0), Point::new(10.0, 0.0));
        let b = line_path(Point::new(10.0, 0.0), Point::new(10.0, 10.0));
        let out = linemerge(vec![a, b], 0.1);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].segments.len(), 2);
    }

    #[test]
    fn linesort_takes_the_nearest_contour_first() {
        let far = line_path(Point::new(100.0, 0.0), Point::new(110.0, 0.0));
        let near = line_path(Point::new(1.0, 0.0), Point::new(2.0, 0.0));
        let out = linesort(vec![far, near]);
        assert_eq!(out[0].segments[0].start(), Point::new(1.0, 0.0));
    }

    #[test]
    fn repeat_passes_duplicates_in_place() {
        let a = line_path(Point::new(0.0, 0.0), Point::new(1.0, 0.0));
        let b = line_path(Point::new(5.0, 0.0), Point::new(6.0, 0.0));
        let out = repeat_passes(vec![a, b], 2);
        assert_eq!(out.len(), 4);
        // In place: A A B B, not A B A B.
        assert_eq!(out[0].segments[0].start(), Point::new(0.0, 0.0));
        assert_eq!(out[1].segments[0].start(), Point::new(0.0, 0.0));
        assert_eq!(out[2].segments[0].start(), Point::new(5.0, 0.0));
    }

    #[test]
    fn blade_offset_leads_the_tip_along_the_first_edge() {
        // A single 10 mm horizontal cut with a 0.5 mm offset: the pivot starts 0.5 mm to
        // the right of the design start, so the tip lands on the design start.
        let tp = line_path(Point::new(0.0, 0.0), Point::new(10.0, 0.0));
        let out = blade_offset(vec![tp], 0.5, 5.0);
        let first = out[0].segments[0].start();
        assert!((first.x - 0.5).abs() < 1e-9);
        assert!(first.y.abs() < 1e-9);
    }
}
