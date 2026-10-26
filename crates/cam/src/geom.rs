//! Minimal 2D geometry for CAM. Curves are flattened before they reach this
//! layer; a `Segment` is always a straight move for now.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Segment {
    Line { from: Point, to: Point },
}

impl Segment {
    pub fn start(&self) -> Point {
        match self {
            Segment::Line { from, .. } => *from,
        }
    }

    pub fn end(&self) -> Point {
        match self {
            Segment::Line { to, .. } => *to,
        }
    }

    /// Shift this segment by `(dx, dy)` millimetres.
    ///
    /// Kept here rather than letting callers match on the variant: a second segment type
    /// (arc, bevel) would otherwise force every translation site to change too.
    pub fn translate(&mut self, dx: f64, dy: f64) {
        match self {
            Segment::Line { from, to } => {
                from.x += dx;
                from.y += dy;
                to.x += dx;
                to.y += dy;
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Toolpath {
    pub segments: Vec<Segment>,
    pub closed: bool,
}

impl Toolpath {
    /// Shift every segment of this toolpath by `(dx, dy)` millimetres.
    pub fn translate(&mut self, dx: f64, dy: f64) {
        for seg in &mut self.segments {
            seg.translate(dx, dy);
        }
    }

    /// A rectangle from `(x, y)` with width `w` and height `h`, closed.
    pub fn rectangle(x: f64, y: f64, w: f64, h: f64) -> Self {
        let a = Point::new(x, y);
        let b = Point::new(x + w, y);
        let c = Point::new(x + w, y + h);
        let d = Point::new(x, y + h);
        Self {
            segments: vec![
                Segment::Line { from: a, to: b },
                Segment::Line { from: b, to: c },
                Segment::Line { from: c, to: d },
                Segment::Line { from: d, to: a },
            ],
            closed: true,
        }
    }

    /// Total path length in millimetres.
    pub fn length_mm(&self) -> f64 {
        self.segments
            .iter()
            .map(|s| {
                let (a, b) = (s.start(), s.end());
                ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt()
            })
            .sum()
    }
}
