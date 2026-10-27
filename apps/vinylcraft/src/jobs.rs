//! Job and message types shared by the app, the control server and the MCP loop.

use serde_json::Value;
use solidcraft_geom::Vec2;
use std::sync::mpsc;

use crate::geometry::bbox_of;

#[derive(Debug, Clone)]
pub(crate) enum JobMsg {
    Done(String),
    Err(String),
}

/// Commands a running GUI accepts over its local control socket.
///
/// The GUI is a separate process from any `--mcp` invocation, so "close the program" cannot
/// travel through an app's own state — it has to cross a process boundary. This enum is the
/// payload of that crossing.
pub(crate) enum CtrlMsg {
    State(mpsc::Sender<Value>),
    Call { name: String, args: Value, reply: mpsc::Sender<Result<Value, String>> },
}

/// One design queued for nesting.
///
/// The polylines are already in design space: `vinyl_job_add` bakes the current transform
/// (offset, scale, rotation, copies) in when the job is queued. That keeps a job's shape
/// frozen at the moment it was added, so changing the live design afterwards cannot silently
/// move a job that is already in the batch.
pub(crate) struct Job {
    pub(crate) name: String,
    pub(crate) polylines: Vec<Vec<Vec2>>,
    /// Colour label used to group jobs. Roll nesting packs only the jobs that
    /// share a colour into one queue, because a cutter holds one vinyl colour at a time:
    /// mixing colours in one pass would cut a single-colour design twice. Empty means "no
    /// colour declared" and nests with the other unlabelled jobs.
    pub(crate) color: String,
}

impl Job {
    /// Bounding box of this job's polylines, as `(min, max)`.
    pub(crate) fn bbox(&self) -> (Vec2, Vec2) {
        bbox_of(&self.polylines)
    }
}

/// One colour group after nesting: the packed placements and the command stream for that
/// colour.
///
/// A group is one run of the machine. Roll nesting puts same-colour jobs in a
/// single queue because the cutter holds one vinyl colour at a time — a multi-colour design
/// becomes several runs, each cut on its own vinyl.
pub(crate) struct NestedGroup {
    pub(crate) color: String,
    /// `(job name, origin in mm, rotated)` for each placed job.
    pub(crate) placed: Vec<(String, Vec2, bool)>,
    /// Job names that did not fit the roll in this group.
    pub(crate) unplaced: Vec<String>,
    pub(crate) used_length_mm: f64,
    pub(crate) code: String,
}
