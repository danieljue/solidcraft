//! Generic 2D CAM for SolidCraft.
//!
//! This crate is UI-free. It turns 2D toolpaths into machine commands
//! (DMPL, HPGL/GPGL) and carries a JSON machine-profile format.
//! Vinyl-specific compensation (blade offset, overcut, weed lines) belongs
//! in a separate `vinyl` crate; this crate stays useful for plasma/laser too.

pub mod align;
pub mod boolean;
pub mod cut_job;
pub mod filters;
pub mod geom;
pub mod marks;
pub mod nest;
pub mod offset;
pub mod post;
pub mod process;
pub mod profile;
pub mod tile;

pub use align::{Similarity, apply_to_toolpaths as apply_alignment, solve as solve_alignment};
pub use boolean::{BooleanOp, FillMode, boolean_paths, signed_area as boolean_signed_area, weld};
pub use cut_job::{
    CutJob, CutJobError, CutJobRequest, CutJobStats, CutOptions, project_and_generate_cut_job, region_to_toolpaths, section_to_toolpaths, slice_mesh,
};
pub use filters::apply as apply_filters;
pub use geom::{Point, Segment, Toolpath};
pub use marks::{MarkConfig, MarkPlacement, MarkStyle};
pub use nest::{NestOptions, NestResult, Placed, bounds, nest as nest_jobs};
pub use offset::{offset_closed, offset_closed_with_tolerance};
pub use process::Process;
pub use profile::{FlowControl, MachineProfile, Protocol, SerialSettings, ToolPreset, USCUTTER_SC2_PROFILE_JSON};
pub use tile::{Tile, TileOptions};
