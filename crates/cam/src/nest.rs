//! Nest independent designs onto a roll so material is not wasted.
//!
//! Roll nesting takes several separate jobs and lays them out on a
//! roll of fixed width, opening a new length as each row fills. VinylCraft's `tile` module
//! does the opposite job: it slices ONE design into a grid so it can be cut on a short
//! machine. Tiling cannot place two unrelated jobs next to each other, so a shop that cuts
//! many small decals still feeds the roll one job at a time and throws away the tail.
//!
//! This module is the missing half. It is a shelf (next-fit-decreasing) packer:
//!
//! 1. Measure each job's bounding box.
//! 2. Optionally turn a job 90 degrees when it is wider than the roll but fits on its side.
//! 3. Sort by height, tallest first, so shelves pack densely.
//! 4. Place each job on the first shelf with room, else open a new shelf.
//!
//! Shelf packing is not optimal — true nesting (arbitrary polygon packing) is NP-hard and
//! needs collision geometry this crate does not carry. It is, however, deterministic,
//! bounded, and beats "one job per roll" by the material it recovers. The result reports
//! the used roll length so the operator knows how much vinyl a batch actually consumes.
//!
//! Bounding boxes are computed from the toolpaths as they stand. A job is translated, never
//! reshaped: no vertex moves relative to its neighbours, so a cut is identical to the one
//! the design produced. Rotation is a rigid 90 degree turn for the same reason.

use crate::geom::{Point, Segment, Toolpath};

/// How to lay jobs out on the roll.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NestOptions {
    /// Usable width of the vinyl. Jobs are packed across this dimension; length is unbounded.
    pub roll_width_mm: f64,
    /// Clear space kept between jobs and between shelves. Zero butts jobs together, which
    /// leaves the blade offset nowhere to go on a shared edge.
    pub gap_mm: f64,
    /// Turn a job 90 degrees when it only fits the roll on its side. Off means an oversized
    /// job is reported as unplaced rather than silently reoriented.
    pub allow_rotation: bool,
}

impl Default for NestOptions {
    fn default() -> Self {
        Self {
            roll_width_mm: 610.0,
            gap_mm: 2.0,
            allow_rotation: false,
        }
    }
}

/// One job after placement. `origin` is where the job's bounding-box minimum landed; add it
/// to every point of the job's own coordinates to get the position on the roll.
#[derive(Debug, Clone, PartialEq)]
pub struct Placed {
    /// Index into the slice the caller passed to [`nest`].
    pub index: usize,
    pub origin: Point,
    /// True when the job was turned 90 degrees to fit the roll.
    pub rotated: bool,
    /// The job's geometry, already translated (and rotated) onto the roll.
    pub toolpaths: Vec<Toolpath>,
}

/// The layout for one batch.
#[derive(Debug, Clone, PartialEq)]
pub struct NestResult {
    pub placed: Vec<Placed>,
    /// Jobs that could not fit the roll at all. Empty when every job was placed.
    pub unplaced: Vec<usize>,
    /// Total roll length consumed, including the last shelf's height. This is the number to
    /// bill against material.
    pub used_length_mm: f64,
    pub roll_width_mm: f64,
}

/// Axis-aligned bounding box of a run of toolpaths, as `(min, max)`.
///
/// Returns `None` for an empty run or a run whose segments carry no points. A single point
/// gives a zero-size box, which is legitimate for a drill mark.
pub fn bounds(toolpaths: &[Toolpath]) -> Option<(Point, Point)> {
    let mut min = Point::new(f64::INFINITY, f64::INFINITY);
    let mut max = Point::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut seen = false;
    for tp in toolpaths {
        for seg in &tp.segments {
            for p in [seg.start(), seg.end()] {
                if p.x < min.x {
                    min.x = p.x;
                }
                if p.y < min.y {
                    min.y = p.y;
                }
                if p.x > max.x {
                    max.x = p.x;
                }
                if p.y > max.y {
                    max.y = p.y;
                }
                seen = true;
            }
        }
    }
    seen.then_some((min, max))
}

/// Turn a point 90 degrees counter-clockwise about the origin.
fn rotate_ccw(p: Point) -> Point {
    Point::new(-p.y, p.x)
}

/// Rigidly turn a job 90 degrees CCW and translate it so its bounding-box minimum sits at
/// `(0, 0)`. Returning geometry in a known quadrant keeps the shelf arithmetic simple.
fn rotate_to_origin(toolpaths: &[Toolpath]) -> Vec<Toolpath> {
    let mut out: Vec<Toolpath> = toolpaths
        .iter()
        .map(|tp| Toolpath {
            segments: tp
                .segments
                .iter()
                .map(|seg| match seg {
                    Segment::Line { from, to } => Segment::Line {
                        from: rotate_ccw(*from),
                        to: rotate_ccw(*to),
                    },
                })
                .collect(),
            closed: tp.closed,
        })
        .collect();
    if let Some((min, _)) = bounds(&out) {
        for tp in &mut out {
            tp.translate(-min.x, -min.y);
        }
    }
    out
}

/// Translate a job so its bounding-box minimum sits at `origin`.
fn move_to(toolpaths: &[Toolpath], origin: Point) -> Vec<Toolpath> {
    let mut out = toolpaths.to_vec();
    if let Some((min, _)) = bounds(&out) {
        let dx = origin.x - min.x;
        let dy = origin.y - min.y;
        for tp in &mut out {
            tp.translate(dx, dy);
        }
    }
    out
}

/// Nest `jobs` onto a roll.
///
/// Each element of `jobs` is one design's toolpaths. The returned [`Placed`] entries carry
/// translated geometry ready to emit as a single job; `used_length_mm` is the vinyl the
/// batch will consume.
pub fn nest(jobs: &[Vec<Toolpath>], opts: &NestOptions) -> NestResult {
    let roll_w = opts.roll_width_mm.max(1e-6);
    let gap = opts.gap_mm.max(0.0);

    // Measure every job once. A job with no geometry is dropped rather than packing a
    // zero-size box that would reserve a gap for nothing.
    struct Item {
        index: usize,
        w: f64,
        h: f64,
        rotated: bool,
        paths: Vec<Toolpath>,
    }

    let mut items: Vec<Item> = Vec::with_capacity(jobs.len());
    let mut unplaced: Vec<usize> = Vec::new();

    for (index, job) in jobs.iter().enumerate() {
        let Some((min, max)) = bounds(job) else {
            continue;
        };
        let mut w = max.x - min.x;
        let mut h = max.y - min.y;
        let mut paths = move_to(job, Point::new(0.0, 0.0));
        let mut rotated = false;

        if w > roll_w {
            if opts.allow_rotation && h <= roll_w {
                // The box as measured is wider than the roll but its height is not, so the
                // turned box (width = old height) fits.
                paths = rotate_to_origin(job);
                std::mem::swap(&mut w, &mut h);
                rotated = true;
            } else {
                unplaced.push(index);
                continue;
            }
        }

        items.push(Item {
            index,
            w,
            h,
            rotated,
            paths,
        });
    }

    // Tallest first: a short job left until last can fill the ragged top of the roll, while
    // a tall job left until last would force a whole new shelf for a little height.
    items.sort_by(|a, b| b.h.partial_cmp(&a.h).unwrap_or(std::cmp::Ordering::Equal));

    // Shelves run left to right across the roll; each new shelf sits below the last.
    struct Shelf {
        y: f64,
        used_x: f64,
        height: f64,
    }
    let mut shelves: Vec<Shelf> = Vec::new();
    let mut placed: Vec<Placed> = Vec::with_capacity(items.len());

    for item in items {
        let mut target: Option<usize> = None;
        for (i, shelf) in shelves.iter().enumerate() {
            let needed = if shelf.used_x > 0.0 {
                shelf.used_x + gap + item.w
            } else {
                item.w
            };
            // The item must fit the roll AND fit within the shelf's already-claimed height
            // band; otherwise a short job would poke into the shelf above it.
            if needed <= roll_w + 1e-9 && item.h <= shelf.height + 1e-9 {
                target = Some(i);
                break;
            }
        }

        let origin = match target {
            Some(i) => {
                let shelf = &mut shelves[i];
                let x = if shelf.used_x > 0.0 {
                    shelf.used_x + gap
                } else {
                    0.0
                };
                shelf.used_x = x + item.w;
                Point::new(x, shelf.y)
            }
            None => {
                // New shelf below all existing ones. A shelf's band is as tall as its tallest
                // job, so the next shelf starts after the tallest job in the previous band.
                let y = shelves
                    .last()
                    .map(|s| s.y + s.height + gap)
                    .unwrap_or(0.0);
                shelves.push(Shelf {
                    y,
                    used_x: item.w,
                    height: item.h,
                });
                Point::new(0.0, y)
            }
        };

        placed.push(Placed {
            index: item.index,
            origin,
            rotated: item.rotated,
            toolpaths: move_to(&item.paths, origin),
        });
    }

    let used_length_mm = shelves
        .last()
        .map(|s| s.y + s.height)
        .unwrap_or(0.0);

    NestResult {
        placed,
        unplaced,
        used_length_mm,
        roll_width_mm: roll_w,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(w: f64, h: f64) -> Vec<Toolpath> {
        vec![Toolpath::rectangle(0.0, 0.0, w, h)]
    }

    #[test]
    fn bounds_of_rectangle() {
        let (min, max) = bounds(&rect(10.0, 4.0)).unwrap();
        assert_eq!(min, Point::new(0.0, 0.0));
        assert_eq!(max, Point::new(10.0, 4.0));
    }

    #[test]
    fn bounds_of_empty_is_none() {
        assert!(bounds(&[]).is_none());
    }

    #[test]
    fn two_small_jobs_share_one_shelf() {
        let opts = NestOptions {
            roll_width_mm: 100.0,
            gap_mm: 0.0,
            allow_rotation: false,
        };
        let r = nest(&[rect(40.0, 20.0), rect(40.0, 20.0)], &opts);
        assert_eq!(r.placed.len(), 2);
        assert!(r.unplaced.is_empty());
        // Both on the same shelf: second starts at x=40.
        assert_eq!(r.placed[0].origin, Point::new(0.0, 0.0));
        assert_eq!(r.placed[1].origin, Point::new(40.0, 0.0));
        // One shelf 20 mm tall.
        assert!((r.used_length_mm - 20.0).abs() < 1e-9);
    }

    #[test]
    fn third_job_opens_a_second_shelf() {
        let opts = NestOptions {
            roll_width_mm: 100.0,
            gap_mm: 0.0,
            allow_rotation: false,
        };
        let jobs = vec![rect(40.0, 20.0), rect(40.0, 20.0), rect(40.0, 20.0)];
        let r = nest(&jobs, &opts);
        assert_eq!(r.placed.len(), 3);
        assert_eq!(r.placed[2].origin, Point::new(0.0, 20.0));
        assert!((r.used_length_mm - 40.0).abs() < 1e-9);
    }

    #[test]
    fn gap_is_honoured_between_jobs() {
        let opts = NestOptions {
            roll_width_mm: 100.0,
            gap_mm: 5.0,
            allow_rotation: false,
        };
        let r = nest(&[rect(40.0, 20.0), rect(40.0, 20.0)], &opts);
        // 40 + 5 + 40 = 85 <= 100, so same shelf with a 5 mm gap.
        assert_eq!(r.placed[1].origin, Point::new(45.0, 0.0));
    }

    #[test]
    fn oversized_job_is_reported_not_dropped() {
        let opts = NestOptions {
            roll_width_mm: 100.0,
            gap_mm: 0.0,
            allow_rotation: false,
        };
        let r = nest(&[rect(150.0, 20.0)], &opts);
        assert!(r.placed.is_empty());
        assert_eq!(r.unplaced, vec![0]);
    }

    #[test]
    fn rotation_saves_an_oversized_job() {
        let opts = NestOptions {
            roll_width_mm: 100.0,
            gap_mm: 0.0,
            allow_rotation: true,
        };
        // 40 wide x 90 tall: too wide as drawn, fits when turned.
        let r = nest(&[rect(40.0, 90.0)], &opts);
        // Actually 40 <= 100 already fits; use a job that really needs the turn.
        assert_eq!(r.placed.len(), 1);
        let _ = r;
        let r2 = nest(&[rect(120.0, 80.0)], &opts);
        assert_eq!(r2.placed.len(), 1);
        assert!(r2.placed[0].rotated);
        assert!(r2.placed[0].origin == Point::new(0.0, 0.0));
    }

    #[test]
    fn rotation_is_refused_when_it_would_not_help() {
        let opts = NestOptions {
            roll_width_mm: 100.0,
            gap_mm: 0.0,
            allow_rotation: true,
        };
        let r = nest(&[rect(120.0, 120.0)], &opts);
        assert!(r.placed.is_empty());
        assert_eq!(r.unplaced, vec![0]);
    }

    #[test]
    fn placed_geometry_keeps_its_shape() {
        let opts = NestOptions {
            roll_width_mm: 100.0,
            gap_mm: 0.0,
            allow_rotation: false,
        };
        let r = nest(&[rect(30.0, 10.0), rect(30.0, 10.0)], &opts);
        let (min, max) = bounds(&r.placed[1].toolpaths).unwrap();
        assert!((max.x - min.x - 30.0).abs() < 1e-9);
        assert!((max.y - min.y - 10.0).abs() < 1e-9);
    }

    #[test]
    fn tallest_shelf_first_reduces_length() {
        let opts = NestOptions {
            roll_width_mm: 100.0,
            gap_mm: 0.0,
            allow_rotation: false,
        };
        // Heights 10, 60, 10 across a 100 mm roll with 50-wide jobs: two per shelf.
        let jobs = vec![rect(50.0, 10.0), rect(50.0, 60.0), rect(50.0, 10.0)];
        let r = nest(&jobs, &opts);
        assert_eq!(r.placed.len(), 3);
        // Sorted 60, 10, 10 -> shelf 1: 60 tall band holding 60 and 10; shelf 2: 10 tall.
        // Length is the sum of the two shelf heights, not of the three job heights: 60 + 10.
        assert!((r.used_length_mm - 70.0).abs() < 1e-9);
    }
}
