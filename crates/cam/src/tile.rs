//! Split a job into tiles so a design larger than the cutter's travel can still be cut.
//!
//! Tiling does exactly this: it slices the design into a grid of
//! rectangles, emits each rectangle as its own job, and the operator advances the roll
//! between tiles. An overlap keeps adjacent tiles joined so a cut spans the seam instead
//! of leaving a gap.
//!
//! The clipping is plain Liang-Barsky against an axis-aligned rectangle. A `Toolpath` is a
//! run of straight segments, so clipping each segment and rebuilding contiguous runs keeps
//! the geometry exact: no curve flattening, no tolerance, no new vertices except where a
//! segment crosses a tile edge.

use crate::geom::{Point, Segment, Toolpath};

/// How to slice the design.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TileOptions {
    pub tile_w: f64,
    pub tile_h: f64,
    /// Extra material cut on each side so neighbouring tiles join cleanly. The overlap is
    /// split evenly: half on the leading edge, half on the trailing edge.
    pub overlap_mm: f64,
    /// Cut a box around each tile so the tile's edge is visible on the vinyl. Useful when
    /// the operator has to line up the next tile by eye.
    pub box_around_tile: bool,
    pub box_margin_mm: f64,
}

impl Default for TileOptions {
    fn default() -> Self {
        Self {
            tile_w: 300.0,
            tile_h: 300.0,
            overlap_mm: 2.0,
            box_around_tile: false,
            box_margin_mm: 0.0,
        }
    }
}

/// One tile of the design.
#[derive(Debug, Clone, PartialEq)]
pub struct Tile {
    /// `(column, row)` in the tile grid, zero-based from the bottom-left of the design.
    pub index: (usize, usize),
    /// Bottom-left corner of this tile in the design's own coordinates. Add this to a
    /// clipped path to get the path's position in the design, or subtract it to get the
    /// position relative to the tile origin for emitting.
    pub origin: Point,
    /// Geometry clipped to this tile.
    pub toolpaths: Vec<Toolpath>,
}

/// Slice `toolpaths` (already in millimetres) into a grid of tiles covering `bbox`.
///
/// A design smaller than one tile comes back as a single tile whose geometry is the input
/// unchanged. Tiles that contain no geometry are dropped.
pub fn tile(toolpaths: &[Toolpath], bbox: (Point, Point), opts: &TileOptions) -> Vec<Tile> {
    let (min, max) = bbox;
    let w = (max.x - min.x).max(1e-6);
    let h = (max.y - min.y).max(1e-6);
    // A 300 mm tile with 2 mm overlap advances 298 mm, so the count is not w/tile_w.
    let advance_x = (opts.tile_w - opts.overlap_mm).max(1e-6);
    let advance_y = (opts.tile_h - opts.overlap_mm).max(1e-6);
    let cols = (((w - opts.overlap_mm) / advance_x).ceil() as usize).max(1);
    let rows = (((h - opts.overlap_mm) / advance_y).ceil() as usize).max(1);

    let mut tiles = Vec::with_capacity(cols * rows);
    for iy in 0..rows {
        for ix in 0..cols {
            let x0 = min.x + ix as f64 * advance_x - opts.overlap_mm * 0.5;
            let y0 = min.y + iy as f64 * advance_y - opts.overlap_mm * 0.5;
            let rect = (x0, y0, x0 + opts.tile_w, y0 + opts.tile_h);
            let mut clipped: Vec<Toolpath> = Vec::new();
            for tp in toolpaths {
                clipped.extend(clip_toolpath(tp, rect));
            }
            if clipped.is_empty() {
                continue;
            }
            if opts.box_around_tile {
                clipped.push(Toolpath::rectangle(
                    x0 - opts.box_margin_mm,
                    y0 - opts.box_margin_mm,
                    opts.tile_w + 2.0 * opts.box_margin_mm,
                    opts.tile_h + 2.0 * opts.box_margin_mm,
                ));
            }
            tiles.push(Tile {
                index: (ix, iy),
                origin: Point::new(x0, y0),
                toolpaths: clipped,
            });
        }
    }
    tiles
}

/// Clip a toolpath against `rect = (x0, y0, x1, y1)`, returning contiguous runs. A run ends
/// where the original path left the rectangle; it does not jump across the gap, because a
/// jump would make the blade lift and drag a straight cut across the tile.
fn clip_toolpath(tp: &Toolpath, rect: (f64, f64, f64, f64)) -> Vec<Toolpath> {
    let mut out = Vec::new();
    let mut run: Vec<Segment> = Vec::new();
    let mut split = false;
    for seg in &tp.segments {
        match clip_segment(seg, rect) {
            Some(s) => run.push(s),
            None => {
                split = true;
                flush(&mut run, &mut out);
            }
        }
    }
    flush(&mut run, &mut out);
    // A closed contour that was not cut by the tile edge is still the same closed contour.
    // Losing the flag would make the blade lift and re-enter at the start point, leaving a
    // nick in a shape that the design closed deliberately.
    if !split && tp.closed {
        if let Some(only) = out.first_mut() {
            only.closed = true;
        }
    }
    out
}

fn flush(run: &mut Vec<Segment>, out: &mut Vec<Toolpath>) {
    if !run.is_empty() {
        out.push(Toolpath {
            segments: std::mem::take(run),
            closed: false,
        });
    }
}

/// Liang-Barsky: keep the part of `seg` inside `rect`.
///
/// Returns `None` when the whole segment is outside, or when what remains is shorter than
/// 1 nm (a rounding artefact at a corner, not a real cut).
fn clip_segment(seg: &Segment, rect: (f64, f64, f64, f64)) -> Option<Segment> {
    let (x0, y0, x1, y1) = rect;
    let a = seg.start();
    let b = seg.end();
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let mut t0 = 0.0f64;
    let mut t1 = 1.0f64;
    // Four edges: left, right, bottom, top. For each, p is the direction and q the distance
    // from the start to the edge.
    for (p, q) in [
        (-dx, a.x - x0),
        (dx, x1 - a.x),
        (-dy, a.y - y0),
        (dy, y1 - a.y),
    ] {
        if p.abs() < 1e-12 {
            // Parallel to this edge: keep only if already inside it.
            if q < 0.0 {
                return None;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                if r > t1 {
                    return None;
                }
                if r > t0 {
                    t0 = r;
                }
            } else {
                if r < t0 {
                    return None;
                }
                if r < t1 {
                    t1 = r;
                }
            }
        }
    }
    if t0 > t1 {
        return None;
    }
    let from = Point::new(a.x + t0 * dx, a.y + t0 * dy);
    let to = Point::new(a.x + t1 * dx, a.y + t1 * dy);
    if (from.x - to.x).abs() < 1e-9 && (from.y - to.y).abs() < 1e-9 {
        return None;
    }
    Some(Segment::Line { a: from, b: to })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bbox() -> (Point, Point) {
        (Point::new(0.0, 0.0), Point::new(100.0, 100.0))
    }

    #[test]
    fn small_design_is_a_single_tile() {
        let tp = Toolpath::rectangle(10.0, 10.0, 20.0, 20.0);
        let opts = TileOptions {
            tile_w: 300.0,
            tile_h: 300.0,
            ..Default::default()
        };
        let tiles = tile(&[tp.clone()], bbox(), &opts);
        assert_eq!(tiles.len(), 1);
        assert_eq!(tiles[0].index, (0, 0));
        // Inside one tile, the geometry is unchanged.
        assert_eq!(tiles[0].toolpaths.len(), 1);
        assert!(tiles[0].toolpaths[0].closed);
    }

    #[test]
    fn wide_design_splits_into_columns_with_overlap() {
        // 100 mm wide, 40 mm tiles, 4 mm overlap -> advance 36 mm -> 3 columns.
        let tp = Toolpath {
            segments: vec![Segment::Line {
                a: Point::new(0.0, 50.0),
                b: Point::new(100.0, 50.0),
            }],
            closed: false,
        };
        let opts = TileOptions {
            tile_w: 40.0,
            tile_h: 40.0,
            overlap_mm: 4.0,
            box_around_tile: false,
            box_margin_mm: 0.0,
        };
        let tiles = tile(&[tp], bbox(), &opts);
        assert_eq!(tiles.len(), 3, "expected 3 columns, got {}", tiles.len());
        // A horizontal line only crosses the middle row, so the surviving tiles are
        // (0,1), (1,1), (2,1) — rows 0 and 2 hold no geometry and are dropped.
        let mid = tiles.iter().find(|t| t.index == (1, 1)).unwrap();
        // The middle tile starts at advance 36 minus half the 4 mm overlap = 34 mm.
        assert!((mid.origin.x - 34.0).abs() < 1e-9, "origin was {}", mid.origin.x);
        // Each tile's geometry stays inside its own rectangle.
        for t in &tiles {
            for path in &t.toolpaths {
                for s in &path.segments {
                    for p in [s.start(), s.end()] {
                        assert!(p.x >= t.origin.x - 1e-9 && p.x <= t.origin.x + 40.0 + 1e-9);
                    }
                }
            }
        }
    }

    #[test]
    fn box_around_tile_adds_a_closed_rectangle() {
        let tp = Toolpath::rectangle(10.0, 10.0, 20.0, 20.0);
        let opts = TileOptions {
            tile_w: 300.0,
            tile_h: 300.0,
            overlap_mm: 2.0,
            box_around_tile: true,
            box_margin_mm: 1.0,
        };
        let tiles = tile(&[tp], bbox(), &opts);
        assert_eq!(tiles.len(), 1);
        let boxes: Vec<&Toolpath> = tiles[0].toolpaths.iter().filter(|t| t.closed).collect();
        assert!(
            boxes.iter().any(|b| b.segments.len() == 4 && b.length_mm() > 250.0),
            "expected the tile box among {:?}",
            tiles[0].toolpaths.len()
        );
    }

    #[test]
    fn segment_crossing_a_tile_edge_is_trimmed_exactly() {
        let seg = Segment::Line {
            a: Point::new(-10.0, 50.0),
            b: Point::new(10.0, 50.0),
        };
        let clipped = clip_segment(&seg, (0.0, 0.0, 100.0, 100.0)).unwrap();
        assert_eq!(clipped.start(), Point::new(0.0, 50.0));
        assert_eq!(clipped.end(), Point::new(10.0, 50.0));
    }

    #[test]
    fn segment_entirely_outside_is_dropped() {
        let seg = Segment::Line {
            a: Point::new(-20.0, -20.0),
            b: Point::new(-10.0, -10.0),
        };
        assert!(clip_segment(&seg, (0.0, 0.0, 100.0, 100.0)).is_none());
    }
}
