//! Curves are not only an emission concern. Offset, boolean and tiling all reason about
//! polygons, and before the geometry moved to `Seg2` they took a curved contour's chord
//! without saying so. These tests pin the behaviour: the algorithms must see the shape.

use solidcraft_cam::boolean::{weld, FillMode};
use solidcraft_cam::geom::{Point, Toolpath};
use solidcraft_cam::offset::offset_closed;
use solidcraft_cam::tile::{tile, TileOptions};
use std::f64::consts::PI;

/// Absolute area by the shoelace formula over each segment's start point.
fn area_of(tp: &Toolpath) -> f64 {
    let pts: Vec<Point> = tp.segments.iter().map(|s| s.start()).collect();
    if pts.len() < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        sum += a.x * b.y - b.x * a.y;
    }
    (sum * 0.5).abs()
}

#[test]
fn offsetting_a_circle_follows_the_curve_not_its_chord() {
    // A circle has no chord to speak of: reading one start point per segment would give a
    // two-point "polygon" and no ring at all.
    let out = offset_closed(&Toolpath::circle(Point::ZERO, 20.0), 5.0);
    assert_eq!(out.len(), 1, "a circle should offset to one ring");
    let area = area_of(&out[0]);
    let expected = PI * 25.0 * 25.0;
    assert!(
        (area - expected).abs() / expected < 0.02,
        "offset ring area {area:.1} mm^2, expected about {expected:.1}"
    );
}

#[test]
fn welding_a_circle_keeps_its_area() {
    let out = weld(&[Toolpath::circle(Point::ZERO, 10.0)], FillMode::NonZero);
    let area: f64 = out.iter().map(area_of).sum();
    let expected = PI * 100.0;
    assert!(
        (area - expected).abs() / expected < 0.01,
        "welded area {area:.1} mm^2, expected about {expected:.1}"
    );
}

#[test]
fn tiling_a_circle_keeps_every_piece_inside_its_tile() {
    let circle = Toolpath::circle(Point::new(50.0, 50.0), 30.0);
    let opts = TileOptions {
        tile_w: 40.0,
        tile_h: 40.0,
        overlap_mm: 2.0,
        box_around_tile: false,
        box_margin_mm: 0.0,
    };
    let tiles = tile(&[circle], (Point::ZERO, Point::new(100.0, 100.0)), &opts);
    assert!(tiles.len() > 1, "a 60 mm circle must span more than one 40 mm tile");
    for t in &tiles {
        for path in &t.toolpaths {
            assert!(!path.segments.is_empty(), "no empty tiles");
            for s in &path.segments {
                for p in [s.start(), s.end()] {
                    assert!(
                        p.x >= t.origin.x - 1e-6 && p.x <= t.origin.x + opts.tile_w + 1e-6,
                        "point x {:.3} outside tile at {:.3}",
                        p.x,
                        t.origin.x
                    );
                    assert!(
                        p.y >= t.origin.y - 1e-6 && p.y <= t.origin.y + opts.tile_h + 1e-6,
                        "point y {:.3} outside tile at {:.3}",
                        p.y,
                        t.origin.y
                    );
                }
            }
        }
    }
}
