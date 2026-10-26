//! 3D model → 2D section → cut paths → machine commands.
//!
//! This is the bridge between modelling and cutting: take a solid's triangle mesh, cut it with a
//! plane, and hand the resulting outlines to the CAM post-processors. It mirrors the CAD gesture
//! "create a sketch on a plane and cut its profile" — the plane becomes a section, the section's
//! loops become contours, and the contours become toolpaths.
//!
//! # Why the mesh, not the B-rep
//!
//! A B-rep face's exact surface would give an analytic section curve. The mesh gives a polygon
//! through the tessellation. That is the right trade for cutting: the tessellation is already what
//! the renderer and the collision checks use, it needs no surface-curve intersector, and a cut
//! path is flattened to short lines before it reaches a machine anyway. Each emitted curve is
//! declared `CurveAccuracy::Analytic` because it really is an exact straight line; how closely the
//! mesh follows the originating solid is the tessellator's claim, not this module's.
//!
//! # What the result means
//!
//! The section's loops are oriented as material demands: outer boundaries counter-clockwise and
//! holes clockwise, in the plane's x/y basis. That is the convention `SectionGeometry` documents
//! and the one kerf compensation relies on — an outer contour grows outward by half the kerf so the
//! finished part keeps its size, while a hole's path shrinks inward by the same amount so the hole
//! comes out the size drawn.
//!
//! # Limitations, stated plainly
//!
//! - A closed mesh gives closed loops. An open mesh still yields its cut segments, but as open
//!   wires with no region: the section reports what the triangles do, not what a solid would be.
//! - Triangles lying in the section plane contribute their boundary edges only. A solid whose
//!   faces sit exactly on the cut plane therefore still outlines correctly, but interior edges of
//!   a coplanar patch are not emitted.
//! - Where four or more cut segments meet at one node — two contours touching at a point — the walk
//!   picks the straightest continuation. A contour that genuinely touches itself at a point may be
//!   split or joined differently than the drawing intends. Real cut parts rarely do this.
//! - The mesh's tessellation is the accuracy ceiling. A coarse cylinder becomes a polygon, and the
//!   section inherits the polygon, not the circle.

use std::collections::HashMap;

use solidcraft_geom::section::{
    CurveAccuracy, CurveUse, SectionCurve, SectionGeometry, SectionRegion, SectionWire,
};
use solidcraft_geom::{Loop2, Mesh, Plane, Region2, Seg2, Vec2, Vec3};

use crate::geom::{Point, Segment, Toolpath};
use crate::post;
use crate::profile::MachineProfile;

/// Triangle-to-plane distance treated as "lying on the plane" (mm). The tessellation's own
/// round-off is far below this; a larger value would start dropping genuine crossings.
const ON_PLANE_MM: f64 = 1e-9;

/// A request to cut one section of one mesh.
///
/// `kerf_mm` is the full width of the cut — the tool diameter for a router or plasma torch, and
/// zero for a drag knife. The offset each contour receives is half of it, because the tool is
/// centred on its path.
pub struct CutJobRequest<'a> {
    pub mesh: &'a Mesh,
    pub plane: Plane,
    pub profile: &'a MachineProfile,
    /// Full cut width in mm. 0 leaves contours exactly on the section.
    pub kerf_mm: f64,
    /// Straight approach before each closed contour, in mm. Plasma pierces off the part and
    /// leads in; 0 starts the cut on the contour itself.
    pub lead_in_mm: f64,
    /// Straight departure after each closed contour, in mm. 0 ends on the contour.
    pub lead_out_mm: f64,
    /// Chord tolerance for flattening arcs and splines into machine moves, in mm.
    pub curve_tol_mm: f64,
    /// How far two section points may differ and still count as one node, in mm. Must lie in
    /// 1e-9..=1.0, matching `SectionGeometry::validate_structure`.
    pub closure_tol_mm: f64,
}

impl<'a> CutJobRequest<'a> {
    /// A request that cuts the section exactly, with no kerf and no leads.
    pub fn new(mesh: &'a Mesh, plane: Plane, profile: &'a MachineProfile) -> Self {
        Self {
            mesh,
            plane,
            profile,
            kerf_mm: 0.0,
            lead_in_mm: 0.0,
            lead_out_mm: 0.0,
            curve_tol_mm: 0.01,
            closure_tol_mm: 1e-6,
        }
    }
}

impl CutJobRequest<'_> {
    /// The cut settings alone, for callers that already hold 2D geometry and need no mesh.
    pub fn options(&self) -> CutOptions {
        CutOptions {
            kerf_mm: self.kerf_mm,
            lead_in_mm: self.lead_in_mm,
            lead_out_mm: self.lead_out_mm,
            curve_tol_mm: self.curve_tol_mm,
            closure_tol_mm: self.closure_tol_mm,
        }
    }
}

/// Kerf, leads and tolerances: what a cut needs to know that is not the model itself.
///
/// Kept separate from [`CutJobRequest`] so a caller who already has 2D profiles — a sketch's
/// `find_profiles` output, say — can cut them without inventing a mesh and a plane to satisfy the
/// mesh path. [`CutJobRequest::options`] hands one out for that path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CutOptions {
    /// Full cut width in mm. 0 leaves contours exactly on the profile.
    pub kerf_mm: f64,
    /// Straight approach before each closed contour, in mm.
    pub lead_in_mm: f64,
    /// Straight departure after each closed contour, in mm.
    pub lead_out_mm: f64,
    /// Chord tolerance for flattening arcs and splines into machine moves, in mm.
    pub curve_tol_mm: f64,
    /// How far two points may differ and still count as one node, in mm.
    pub closure_tol_mm: f64,
}

impl Default for CutOptions {
    fn default() -> Self {
        Self { kerf_mm: 0.0, lead_in_mm: 0.0, lead_out_mm: 0.0, curve_tol_mm: 0.01, closure_tol_mm: 1e-6 }
    }
}

/// What a job produced, in numbers a caller can show without re-reading the geometry.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CutJobStats {
    /// Closed loops found in the section.
    pub loops: usize,
    /// Open chains found in the section (an unclosed mesh, or a contour the walk could not close).
    pub open_chains: usize,
    /// Contours actually emitted as cut paths, after kerf offsetting and degeneracy removal.
    pub contours: usize,
    /// Total travelled path length in mm, leads and all.
    pub path_length_mm: f64,
    /// Area enclosed by the section's outer loops, less its holes, in mm².
    pub area_mm2: f64,
}

/// The finished job: the section it came from, the paths it will cut, and the bytes to send.
pub struct CutJob {
    /// The 2D sketch, in the plane's coordinates. Kept so a caller can draw or re-check it.
    pub section: SectionGeometry,
    /// Paths in cut order. Open when leads were added, closed otherwise.
    pub toolpaths: Vec<Toolpath>,
    /// Machine commands from the profile's protocol, ready to send.
    pub commands: String,
    pub stats: CutJobStats,
}

#[derive(Debug, thiserror::Error)]
pub enum CutJobError {
    #[error("cut job: {0}")]
    Request(&'static str),
    #[error("cut job: section geometry: {0}")]
    Section(#[from] solidcraft_geom::section::Invalid),
    #[error("cut job: post-processor: {0}")]
    Post(#[from] post::PostError),
}

/// Slice a mesh with a plane, cut the resulting section, and post the commands.
///
/// This is the whole pipeline in one call: mesh → section → toolpaths → machine bytes. Callers
/// that need to inspect or adjust the section first should use [`slice_mesh`] and
/// [`section_to_toolpaths`] instead.
pub fn project_and_generate_cut_job(req: &CutJobRequest<'_>) -> Result<CutJob, CutJobError> {
    let section = slice_mesh(req.mesh, req.plane, req.closure_tol_mm)?;
    let toolpaths = section_to_toolpaths(&section, req)?;
    let commands = post::emit(req.profile, &toolpaths)?;
    let stats = stats_of(&section, &toolpaths);
    Ok(CutJob { section, toolpaths, commands, stats })
}

/// Cut a mesh with a plane and return the section as 2D geometry in that plane.
///
/// Outer loops come back counter-clockwise and holes clockwise, both in the plane's x/y basis.
/// Open chains — what an unclosed mesh leaves — are reported as open wires and never closed.
pub fn slice_mesh(mesh: &Mesh, plane: Plane, closure_tol_mm: f64) -> Result<SectionGeometry, CutJobError> {
    if !(1e-9..=1.0).contains(&closure_tol_mm) {
        return Err(CutJobError::Request("closure tolerance must be between 1e-9 and 1 mm"));
    }
    let heights: Vec<f64> = mesh.positions.iter().map(|p| plane.height(*p)).collect();
    let mut segments: Vec<(Vec2, Vec2)> = Vec::new();
    let mut contacts: Vec<Vec2> = Vec::new();
    // Triangles lying in the plane: their boundary edges become the outline of a face cut
    // exactly on the plane. Collected first, emitted after, because only then is "shared edge"
    // known for the whole coplanar patch.
    let mut coplanar: Vec<[u32; 3]> = Vec::new();

    for tri in &mesh.triangles {
        let [ia, ib, ic] = *tri;
        let (Some(&ha), Some(&hb), Some(&hc)) = (
            heights.get(ia as usize),
            heights.get(ib as usize),
            heights.get(ic as usize),
        ) else {
            return Err(CutJobError::Request("a triangle references a vertex the mesh does not have"));
        };
        let idx = [ia, ib, ic];
        let h = [ha, hb, hc];
        let on = h.map(|v| v.abs() <= ON_PLANE_MM);
        let pos = h.map(|v| v > ON_PLANE_MM);
        let neg = h.map(|v| v < -ON_PLANE_MM);
        let on_count = on.iter().filter(|v| **v).count();

        if on_count == 3 {
            // Wholly in the plane: its edges belong to the patch, handled after the loop.
            coplanar.push(*tri);
            continue;
        }
        if on_count == 2 {
            // Two vertices in the plane: the shared edge is the intersection. The neighbour on
            // the other side emits the same edge, so duplicates are dropped when stitching.
            let mut pair: Vec<u32> = (0..3).filter(|k| on[*k]).map(|k| idx[k]).collect();
            pair.sort_unstable();
            if let [u, v] = pair[..] {
                let a = plane.to_local(mesh.positions[u as usize]);
                let b = plane.to_local(mesh.positions[v as usize]);
                if a.dist(b) > ON_PLANE_MM {
                    segments.push((a, b));
                }
            }
            continue;
        }
        if on_count == 1 {
            let k = (0..3).find(|k| on[*k]).unwrap_or(0);
            let (i, j) = ((k + 1) % 3, (k + 2) % 3);
            if pos[i] != pos[j] {
                // The plane passes through one vertex and through the opposite edge.
                let v = mesh.positions[idx[k] as usize];
                let h_i = heights[idx[i] as usize];
                let h_j = heights[idx[j] as usize];
                if let Some(cross) =
                    edge_crossing(mesh.positions[idx[i] as usize], h_i, mesh.positions[idx[j] as usize], h_j)
                {
                    let a = plane.to_local(v);
                    let b = plane.to_local(cross);
                    if a.dist(b) > ON_PLANE_MM {
                        segments.push((a, b));
                    }
                }
            } else {
                // A lone vertex touch: a contact, not a contour.
                contacts.push(plane.to_local(mesh.positions[idx[k] as usize]));
            }
            continue;
        }
        // No vertex in the plane. Only a straddling triangle has an intersection.
        let p = pos.iter().filter(|v| **v).count();
        let n = neg.iter().filter(|v| **v).count();
        if p == 0 || n == 0 {
            continue;
        }
        let mut hits: Vec<Vec2> = Vec::new();
        for (u, v) in [(0usize, 1usize), (1, 2), (2, 0)] {
            let (hu, hv) = (heights[idx[u] as usize], heights[idx[v] as usize]);
            if (hu > ON_PLANE_MM && hv < -ON_PLANE_MM) || (hu < -ON_PLANE_MM && hv > ON_PLANE_MM) {
                if let Some(cross) = edge_crossing(mesh.positions[idx[u] as usize], hu, mesh.positions[idx[v] as usize], hv) {
                    hits.push(plane.to_local(cross));
                }
            }
        }
        if let [a, b] = hits[..] {
            if a.dist(b) > ON_PLANE_MM {
                segments.push((a, b));
            }
        }
    }

    // A patch lying in the plane: its edges not shared with another triangle of the patch are the
    // patch's outline. Sharing is decided on vertex indices, so a duplicated vertex across faces
    // reads as a boundary — which is what the geometry is, even though the mesh may be closed.
    if !coplanar.is_empty() {
        let mut edge_use: HashMap<(u32, u32), usize> = HashMap::new();
        for tri in &coplanar {
            for (u, v) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                let key = if u < v { (u, v) } else { (v, u) };
                *edge_use.entry(key).or_insert(0) += 1;
            }
        }
        for ((u, v), count) in edge_use {
            if count != 1 {
                continue;
            }
            let a = plane.to_local(mesh.positions[u as usize]);
            let b = plane.to_local(mesh.positions[v as usize]);
            if a.dist(b) > ON_PLANE_MM {
                segments.push((a, b));
            }
        }
    }

    // Adjacent triangles compute the same crossing point from opposite ends of a shared edge, so
    // the two results differ only by round-off. Snap them, and drop the duplicate segments that
    // coplanar patches and both-sides-on-plane triangles produce.
    let segments = snap_and_dedupe(segments, closure_tol_mm);
    let (loops, open) = stitch(&segments, closure_tol_mm);
    let nesting = analyse_nesting(&loops, closure_tol_mm);
    let oriented = orient(&loops, &nesting);
    let mut section = build_section(plane, oriented, &nesting, open, contacts);
    // A contact that already sits on an emitted contour is not isolated; keep only the ones that
    // stand alone, which is what the field documents.
    let endpoints: Vec<Vec2> = section
        .curves
        .iter()
        .flat_map(|c| [c.geometry.start(), c.geometry.end()])
        .collect();
    section.contacts.retain(|p| !endpoints.iter().any(|q| q.dist(*p) <= closure_tol_mm));
    section.validate_structure(closure_tol_mm)?;
    Ok(section)
}

/// A crossing point on an edge, or `None` when the edge runs parallel to the plane.
fn edge_crossing(a: Vec3, ha: f64, b: Vec3, hb: f64) -> Option<Vec3> {
    let den = ha - hb;
    if den.abs() < 1e-15 {
        return None;
    }
    let t = ha / den;
    if !(-1e-9..=1.0 + 1e-9).contains(&t) {
        return None;
    }
    Some(a + (b - a) * t)
}

/// Sort segment endpoints so an edge seen from either triangle compares equal, then drop repeats.
fn snap_and_dedupe(mut segments: Vec<(Vec2, Vec2)>, tol: f64) -> Vec<(Vec2, Vec2)> {
    for (a, b) in &mut segments {
        if lex_less(*b, *a) {
            std::mem::swap(a, b);
        }
    }
    segments.sort_by(|x, y| lex_cmp(x.0, y.0).then_with(|| lex_cmp(x.1, y.1)));
    let mut out: Vec<(Vec2, Vec2)> = Vec::with_capacity(segments.len());
    for seg in segments {
        if let Some(last) = out.last() {
            if same_point(last.0, seg.0, tol) && same_point(last.1, seg.1, tol) {
                continue;
            }
        }
        out.push(seg);
    }
    out
}

fn lex_less(a: Vec2, b: Vec2) -> bool {
    lex_cmp(a, b) == std::cmp::Ordering::Less
}

fn lex_cmp(a: Vec2, b: Vec2) -> std::cmp::Ordering {
    a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y))
}

fn same_point(a: Vec2, b: Vec2, tol: f64) -> bool {
    a.dist(b) <= tol
}

/// Join segment endpoints into nodes and walk closed loops out of the resulting graph.
///
/// Returns the closed loops (as point rings without a repeated last point) and the open chains
/// the walk could not close. A node with more than two segments picks the straightest
/// continuation, which keeps a contour going straight through a point where two contours touch.
fn stitch(segments: &[(Vec2, Vec2)], tol: f64) -> (Vec<Vec<Vec2>>, Vec<Vec<Vec2>>) {
    let mut nodes: Vec<Vec2> = Vec::new();
    let mut index: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    let cell = tol.max(1e-9);

    // Each segment becomes an edge between two node indices. Self-loops (both ends one node) are
    // dropped: they enclose no area a cutter could follow.
    let mut edges: Vec<(usize, usize)> = Vec::with_capacity(segments.len());
    for (a, b) in segments {
        let u = node_of(*a, tol, cell, &mut nodes, &mut index);
        let v = node_of(*b, tol, cell, &mut nodes, &mut index);
        if u != v {
            edges.push((u, v));
        }
    }

    let mut incident: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    for (i, (u, v)) in edges.iter().enumerate() {
        incident[*u].push(i);
        incident[*v].push(i);
    }

    // Open chains read better when the walk starts at an end, so edges touching a degree-one node
    // come first. A closed contour has no such node and its order is unaffected.
    let mut order: Vec<usize> = (0..edges.len()).collect();
    order.sort_by_key(|&e| {
        let (u, v) = edges[e];
        incident[u].len().min(incident[v].len()) != 1
    });

    let mut used = vec![false; edges.len()];
    let mut closed: Vec<Vec<Vec2>> = Vec::new();
    let mut open: Vec<Vec<Vec2>> = Vec::new();

    for start_edge in order {
        if used[start_edge] {
            continue;
        }
        let (u0, v0) = edges[start_edge];
        used[start_edge] = true;
        let start_node = u0;
        let mut ring: Vec<Vec2> = vec![nodes[start_node], nodes[v0]];
        let mut at = v0;
        let mut closed_loop = false;
        // A cut contour through a closed mesh is far shorter than the segment count; the bound
        // only guards against a malformed graph cycling without consuming an edge.
        for _ in 0..=edges.len() {
            if at == start_node {
                closed_loop = true;
                break;
            }
            let Some(next) = continuation(&incident[at], &edges, &used, at, &nodes, ring[ring.len() - 1]) else { break };
            used[next] = true;
            let (a, b) = edges[next];
            at = if a == at { b } else { a };
            ring.push(nodes[at]);
        }
        // The walk pushed the start node's duplicate only when it returned to it; drop it so a
        // ring carries each point once.
        if closed_loop && ring.len() > 1 && same_point(ring[0], ring[ring.len() - 1], tol) {
            ring.pop();
        }
        if closed_loop && ring.len() >= 3 {
            closed.push(ring);
        } else if ring.len() >= 2 {
            open.push(ring);
        }
    }
    (closed, open)
}

/// The next unused edge at `at`, preferring the one that continues the incoming direction.
fn continuation(
    incident: &[usize],
    edges: &[(usize, usize)],
    used: &[bool],
    at: usize,
    nodes: &[Vec2],
    came_from: Vec2,
) -> Option<usize> {
    let here = *nodes.get(at)?;
    let incoming = here - came_from;
    let incoming = if incoming.len() > 1e-15 { incoming } else { Vec2::new(1.0, 0.0) };
    let incoming_len = incoming.len();
    let mut best: Option<(f64, usize)> = None;
    for &e in incident {
        if used.get(e).copied().unwrap_or(true) {
            continue;
        }
        let (a, b) = *edges.get(e)?;
        let far = if a == at { b } else if b == at { a } else { continue };
        let dir = *nodes.get(far)? - here;
        let len = dir.len();
        if len <= 1e-15 {
            continue;
        }
        // Cosine of the turn: 1 leaves straight on, -1 doubles back.
        let dot = (dir.x * incoming.x + dir.y * incoming.y) / (len * incoming_len);
        if best.is_none_or(|(best_dot, _)| dot > best_dot) {
            best = Some((dot, e));
        }
    }
    best.map(|(_, e)| e)
}

/// Snap a point to an existing node within `tol`, or create one.
fn node_of(p: Vec2, tol: f64, cell: f64, nodes: &mut Vec<Vec2>, index: &mut HashMap<(i64, i64), Vec<usize>>) -> usize {
    let key = ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64);
    for dx in -1..=1 {
        for dy in -1..=1 {
            if let Some(candidates) = index.get(&(key.0 + dx, key.1 + dy)) {
                for &n in candidates {
                    if let Some(q) = nodes.get(n) {
                        if same_point(*q, p, tol) {
                            return n;
                        }
                    }
                }
            }
        }
    }
    let n = nodes.len();
    nodes.push(p);
    index.entry(key).or_default().push(n);
    n
}

/// How deeply each loop nests inside the others, and each hole's immediate parent.
struct Nesting {
    depth: Vec<usize>,
    parent: Vec<Option<usize>>,
}

/// Depth is the number of other loops that surround this loop's whole boundary. A hole's parent is
/// the containing loop of greatest depth, so an island inside a hole becomes its own outer region
/// rather than a hole of the original outer.
///
/// Containment is tested on the loop's *boundary*, not its centre. A plate's outer square has its
/// centre inside its own hole, so a centre test would call the outer a hole of the hole and emit no
/// region at all (#cut_job).
fn analyse_nesting(loops: &[Vec<Vec2>], tol: f64) -> Nesting {
    let n = loops.len();
    let containers: Vec<Vec<usize>> = (0..n)
        .map(|i| (0..n).filter(|&j| j != i && loop_inside(&loops[i], &loops[j], tol)).collect())
        .collect();
    let depth: Vec<usize> = containers.iter().map(Vec::len).collect();
    let parent: Vec<Option<usize>> = containers
        .iter()
        .map(|c| c.iter().copied().max_by_key(|&j| depth.get(j).copied().unwrap_or(0)))
        .collect();
    Nesting { depth, parent }
}

/// Point every outer loop counter-clockwise and every hole clockwise, as the section documents.
fn orient(loops: &[Vec<Vec2>], nesting: &Nesting) -> Vec<Vec<Vec2>> {
    loops
        .iter()
        .enumerate()
        .map(|(i, ring)| {
            let want_ccw = nesting.depth.get(i).copied().unwrap_or(0) % 2 == 0;
            let ccw = ring_area(ring) >= 0.0;
            if ccw == want_ccw {
                ring.clone()
            } else {
                let mut reversed = ring.clone();
                reversed.reverse();
                reversed
            }
        })
        .collect()
}

/// Does `inner` lie wholly inside `outer`? Every vertex must be inside or on the boundary, which
/// is the right question for disjoint section contours and avoids the trap a centre test falls
/// into: the centre of a plate's outer square sits inside its own hole.
fn loop_inside(inner: &[Vec2], outer: &[Vec2], tol: f64) -> bool {
    !inner.is_empty() && inner.iter().all(|p| point_in_ring(*p, outer) || near_edge(*p, outer, tol))
}

/// Is `p` within `tol` of any edge of `ring`? A vertex sitting exactly on another contour would
/// otherwise be classified by the ray test's tie-breaking rather than by position.
fn near_edge(p: Vec2, ring: &[Vec2], tol: f64) -> bool {
    let n = ring.len();
    (0..n).any(|i| dist_to_segment(p, ring[i], ring[(i + 1) % n]) <= tol)
}

fn dist_to_segment(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let ab = b - a;
    let len2 = ab.x * ab.x + ab.y * ab.y;
    if len2 <= 1e-30 {
        return p.dist(a);
    }
    let t = (((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2).clamp(0.0, 1.0);
    p.dist(Vec2::new(a.x + ab.x * t, a.y + ab.y * t))
}

/// Shoelace area; positive when the ring runs counter-clockwise.
fn ring_area(ring: &[Vec2]) -> f64 {
    let n = ring.len();
    if n < 3 {
        return 0.0;
    }
    let mut a = 0.0;
    for i in 0..n {
        let p = ring[i];
        let q = ring[(i + 1) % n];
        a += p.x * q.y - p.y * q.x;
    }
    a * 0.5
}

/// Even-odd containment of `p` in `ring`.
fn point_in_ring(p: Vec2, ring: &[Vec2]) -> bool {
    let n = ring.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    for i in 0..n {
        let a = ring[i];
        let b = ring[(i + 1) % n];
        if (a.y > p.y) != (b.y > p.y) {
            let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if x > p.x {
                inside = !inside;
            }
        }
    }
    inside
}

/// Assemble the section DTO: one closed wire per loop and in loop order, so a region's wire
/// reference is its loop index. Open chains follow the closed loops.
fn build_section(
    plane: Plane,
    loops: Vec<Vec<Vec2>>,
    nesting: &Nesting,
    open: Vec<Vec<Vec2>>,
    contacts: Vec<Vec2>,
) -> SectionGeometry {
    let mut curves: Vec<SectionCurve> = Vec::new();
    let mut wires: Vec<SectionWire> = Vec::new();
    for ring in &loops {
        let base = curves.len() as u32;
        let n = ring.len();
        for i in 0..n {
            let a = ring[i];
            let b = ring[(i + 1) % n];
            curves.push(SectionCurve { geometry: Seg2::Line { a, b }, accuracy: CurveAccuracy::Analytic });
        }
        wires.push(SectionWire {
            curves: (0..n as u32).map(|k| CurveUse { curve: base + k, forward: true }).collect(),
            closed: true,
        });
    }
    for chain in &open {
        let base = curves.len() as u32;
        let n = chain.len().saturating_sub(1);
        for i in 0..n {
            let a = chain[i];
            let b = chain[i + 1];
            curves.push(SectionCurve { geometry: Seg2::Line { a, b }, accuracy: CurveAccuracy::Analytic });
        }
        wires.push(SectionWire {
            curves: (0..n as u32).map(|k| CurveUse { curve: base + k, forward: true }).collect(),
            closed: false,
        });
    }
    let mut regions: Vec<SectionRegion> = Vec::new();
    for i in 0..loops.len() {
        if nesting.depth.get(i).copied().unwrap_or(0) % 2 != 0 {
            continue;
        }
        let holes: Vec<u32> = (0..loops.len())
            .filter(|&j| nesting.parent.get(j).copied().flatten() == Some(i))
            .map(|j| j as u32)
            .collect();
        regions.push(SectionRegion { outer: i as u32, holes });
    }
    SectionGeometry { plane, curves, wires, regions, contacts }
}

/// Turn a section's contours into cut paths, compensating for kerf and adding leads.
///
/// An outer boundary grows by half the kerf so the part keeps its size; a hole's path shrinks by
/// half the kerf so the hole comes out the size drawn. A closed wire the regions do not mention
/// is cut as an outer contour. Open wires are cut as they are — there is nothing to offset.
pub fn section_to_toolpaths(section: &SectionGeometry, req: &CutJobRequest<'_>) -> Result<Vec<Toolpath>, CutJobError> {
    section.validate_structure(req.closure_tol_mm)?;
    let options = req.options();
    let half = options.kerf_mm * 0.5;
    let mut out: Vec<Toolpath> = Vec::new();
    let mut referenced = vec![false; section.wires.len()];

    for (ri, region) in section.regions.iter().enumerate() {
        let r = section.to_region(ri as u32, req.closure_tol_mm)?;
        for id in std::iter::once(&region.outer).chain(&region.holes) {
            if let Some(flag) = referenced.get_mut(*id as usize) {
                *flag = true;
            }
        }
        out.extend(region_to_toolpaths(&r, &options));
    }
    for (wi, wire) in section.wires.iter().enumerate() {
        if !wire.closed || referenced.get(wi).copied().unwrap_or(false) {
            continue;
        }
        let l = section.to_loop(wi as u32, req.closure_tol_mm)?;
        emit_loop(&l, half, &options, &mut out);
    }
    Ok(out)
}

/// Cut a planar region: its outer boundary as an outer contour, its holes as hole contours.
///
/// This is the integration point for callers that already have 2D profiles. A sketch's
/// `find_profiles` returns `Profile`s whose `.region` is exactly a `Region2`, and `Region2` lives in
/// `solidcraft-geom`, which this crate already depends on — so a profile needs no conversion at all
/// before it can be cut. Nothing here knows about sketches, so the dependency stays one-way.
///
/// The outer loop is offset outward by half the kerf and each hole inward, which is why
/// [`Region2`] documents its outer loop as counter-clockwise and its holes as clockwise.
pub fn region_to_toolpaths(region: &Region2, options: &CutOptions) -> Vec<Toolpath> {
    let half = options.kerf_mm * 0.5;
    let mut out: Vec<Toolpath> = Vec::new();
    emit_loop(&region.outer, half, options, &mut out);
    for hole in &region.holes {
        emit_loop(hole, -half, options, &mut out);
    }
    out
}

/// Emit one loop as cut paths: flatten it, offset it by `offset`, then add leads to each result.
fn emit_loop(loop2: &Loop2, offset: f64, options: &CutOptions, out: &mut Vec<Toolpath>) {
    let pts: Vec<Vec2> = loop2.polyline(options.curve_tol_mm).into_vec();
    let Some(path) = ring_to_toolpath(&pts, options.closure_tol_mm) else { return };
    if offset.abs() < 1e-12 {
        out.push(add_leads(&path, options.lead_in_mm, options.lead_out_mm));
        return;
    }
    for shifted in crate::offset::offset_closed(&path, offset) {
        out.push(add_leads(&shifted, options.lead_in_mm, options.lead_out_mm));
    }
}

/// A closed toolpath through `pts`, dropping repeated points so the machine never gets a zero move.
fn ring_to_toolpath(pts: &[Vec2], tol: f64) -> Option<Toolpath> {
    let mut clean: Vec<Vec2> = Vec::with_capacity(pts.len());
    for p in pts {
        if clean.last().map_or(true, |q: &Vec2| q.dist(*p) > tol) {
            clean.push(*p);
        }
    }
    if clean.len() > 1 && clean[0].dist(clean[clean.len() - 1]) <= tol {
        clean.pop();
    }
    let n = clean.len();
    if n < 3 {
        return None;
    }
    let mut segments = Vec::with_capacity(n);
    for i in 0..n {
        let a = clean[i];
        let b = clean[(i + 1) % n];
        segments.push(Segment::Line { from: Point::new(a.x, a.y), to: Point::new(b.x, b.y) });
    }
    Some(Toolpath { segments, closed: true })
}

/// Add a straight lead-in and lead-out along the contour's departure direction.
///
/// A closed path becomes open when either lead is non-zero: the machine approaches from outside,
/// cuts the contour, and leaves along the same tangent. With both zero the path stays closed.
fn add_leads(tp: &Toolpath, lead_in: f64, lead_out: f64) -> Toolpath {
    if !tp.closed || (lead_in <= 0.0 && lead_out <= 0.0) {
        return tp.clone();
    }
    let Some(first) = tp.segments.first() else { return tp.clone() };
    let p0 = first.start();
    let p1 = first.end();
    let d = Vec2::new(p1.x - p0.x, p1.y - p0.y);
    let Some(dir) = d.normalized() else { return tp.clone() };
    let mut segments: Vec<Segment> = Vec::with_capacity(tp.segments.len() + 2);
    if lead_in > 0.0 {
        segments.push(Segment::Line {
            from: Point::new(p0.x - dir.x * lead_in, p0.y - dir.y * lead_in),
            to: p0,
        });
    }
    segments.extend(tp.segments.iter().cloned());
    if lead_out > 0.0 {
        segments.push(Segment::Line {
            from: p0,
            to: Point::new(p0.x + dir.x * lead_out, p0.y + dir.y * lead_out),
        });
    }
    Toolpath { segments, closed: false }
}

fn stats_of(section: &SectionGeometry, toolpaths: &[Toolpath]) -> CutJobStats {
    let mut stats = CutJobStats {
        loops: section.wires.iter().filter(|w| w.closed).count(),
        open_chains: section.wires.iter().filter(|w| !w.closed).count(),
        contours: toolpaths.len(),
        path_length_mm: toolpaths.iter().map(Toolpath::length_mm).sum(),
        area_mm2: 0.0,
    };
    // A region's area is its outer less its holes, which is what a part's section area means.
    for ri in 0..section.regions.len() {
        if let Ok(r) = section.to_region(ri as u32, 1e-6) {
            stats.area_mm2 += r.area();
        }
    }
    stats
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A closed box as 12 triangles, each face with its own four vertices.
    fn cuboid(lo: Vec3, hi: Vec3) -> Mesh {
        let v = |x: bool, y: bool, z: bool| {
            Vec3::new(if x { hi.x } else { lo.x }, if y { hi.y } else { lo.y }, if z { hi.z } else { lo.z })
        };
        let quads = [
            [v(false, false, false), v(false, true, false), v(true, true, false), v(true, false, false)],
            [v(false, false, true), v(true, false, true), v(true, true, true), v(false, true, true)],
            [v(false, false, false), v(true, false, false), v(true, false, true), v(false, false, true)],
            [v(false, true, false), v(false, true, true), v(true, true, true), v(true, true, false)],
            [v(false, false, false), v(false, false, true), v(false, true, true), v(false, true, false)],
            [v(true, false, false), v(true, true, false), v(true, true, true), v(true, false, true)],
        ];
        let mut m = Mesh::default();
        for q in quads {
            let base = m.positions.len() as u32;
            m.positions.extend_from_slice(&q);
            m.normals.extend_from_slice(&[Vec3::Z; 4]);
            m.triangles.push([base, base + 1, base + 2]);
            m.triangles.push([base, base + 2, base + 3]);
        }
        m
    }

    /// The walls of a plate with a rectangular hole: four per ring, each wall its own four
    /// vertices. Enough for a section at mid-height: the top and bottom faces never meet the
    /// plane. Winding is irrelevant here because the section's orientation comes from nesting.
    fn walls_with_hole(outer: (f64, f64), hole: (f64, f64, f64, f64), top: f64) -> Mesh {
        let mut m = Mesh::default();
        let mut wall = |a: (f64, f64), b: (f64, f64)| {
            let base = m.positions.len() as u32;
            m.positions.push(Vec3::new(a.0, a.1, 0.0));
            m.positions.push(Vec3::new(b.0, b.1, 0.0));
            m.positions.push(Vec3::new(b.0, b.1, top));
            m.positions.push(Vec3::new(a.0, a.1, top));
            m.triangles.push([base, base + 1, base + 2]);
            m.triangles.push([base, base + 2, base + 3]);
        };
        let (w, h) = outer;
        let (hx0, hy0, hx1, hy1) = hole;
        wall((0.0, 0.0), (w, 0.0));
        wall((w, 0.0), (w, h));
        wall((w, h), (0.0, h));
        wall((0.0, h), (0.0, 0.0));
        wall((hx0, hy0), (hx0, hy1));
        wall((hx0, hy1), (hx1, hy1));
        wall((hx1, hy1), (hx1, hy0));
        wall((hx1, hy0), (hx0, hy0));
        m
    }

    fn gcode_profile() -> MachineProfile {
        serde_json::from_value(serde_json::json!({
            "name": "test plasma",
            "protocol": "gcode",
            "units_per_mm": 1.0,
            "header": "G21;G90;",
            "footer": "M2;"
        }))
        .expect("a G-code profile")
    }

    #[test]
    fn a_box_sliced_across_its_height_gives_one_square_loop() {
        let mesh = cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 6.0, 2.0));
        let plane = Plane::named("XY").expect("the XY plane").offset(1.0);
        let section = slice_mesh(&mesh, plane, 1e-6).expect("a section");
        assert_eq!(section.regions.len(), 1, "one outer region");
        assert_eq!(section.wires.iter().filter(|w| w.closed).count(), 1, "one closed loop");
        let r = section.to_region(0, 1e-6).expect("the region");
        assert!((r.area() - 60.0).abs() < 1e-6, "10 x 6 = 60, got {}", r.area());
        assert!(r.outer.is_ccw(), "an outer boundary runs counter-clockwise");
        assert!(r.holes.is_empty(), "a box has no hole");
    }

    #[test]
    fn a_plate_with_a_hole_gives_a_region_with_one_hole() {
        let mesh = walls_with_hole((10.0, 10.0), (3.0, 3.0, 7.0, 7.0), 2.0);
        let plane = Plane::named("XY").expect("the XY plane").offset(1.0);
        let section = slice_mesh(&mesh, plane, 1e-6).expect("a section");
        assert_eq!(section.regions.len(), 1, "one outer region");
        assert_eq!(section.wires.len(), 2, "an outer loop and a hole loop");
        let r = section.to_region(0, 1e-6).expect("the region");
        assert_eq!(r.holes.len(), 1, "one hole");
        assert!((r.area() - 84.0).abs() < 1e-6, "100 - 16 = 84, got {}", r.area());
        assert!(r.outer.is_ccw(), "an outer boundary runs counter-clockwise");
        assert!(!r.holes[0].is_ccw(), "a hole runs clockwise");
    }

    #[test]
    fn a_plate_lying_in_the_plane_is_outlined_by_its_boundary_edges() {
        let mesh = cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 1.0));
        let plane = Plane::named("XY").expect("the XY plane");
        let section = slice_mesh(&mesh, plane, 1e-6).expect("a section");
        let r = section.to_region(0, 1e-6).expect("the region");
        assert!((r.area() - 100.0).abs() < 1e-6, "10 x 10 = 100, got {}", r.area());
    }

    #[test]
    fn kerf_grows_the_outer_and_shrinks_the_hole() {
        let mesh = walls_with_hole((10.0, 10.0), (3.0, 3.0, 7.0, 7.0), 2.0);
        let profile = gcode_profile();
        let plane = Plane::named("XY").expect("the XY plane").offset(1.0);
        let mut req = CutJobRequest::new(&mesh, plane, &profile);
        req.kerf_mm = 2.0;
        let section = slice_mesh(&mesh, req.plane, req.closure_tol_mm).expect("a section");
        let paths = section_to_toolpaths(&section, &req).expect("paths");
        assert_eq!(paths.len(), 2, "one outer and one hole path");
        // Half the kerf is 1 mm: the outer path grows past the section's 10x10 and the hole path
        // shrinks inside the 4x4 hole.
        let mut areas: Vec<f64> = paths
            .iter()
            .map(|tp| {
                let pts: Vec<Vec2> = tp.segments.iter().map(|s| Vec2::new(s.start().x, s.start().y)).collect();
                ring_area(&pts).abs()
            })
            .collect();
        areas.sort_by(f64::total_cmp);
        assert!(areas[0] < 16.0, "the hole path shrank inside the 4x4 hole, got {}", areas[0]);
        assert!(areas[1] > 100.0, "the outer path grew past 10x10, got {}", areas[1]);
    }

    #[test]
    fn leads_open_the_path_and_lengthen_it() {
        let mesh = cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 2.0));
        let profile = gcode_profile();
        let plane = Plane::named("XY").expect("the XY plane").offset(1.0);
        let mut req = CutJobRequest::new(&mesh, plane, &profile);
        req.lead_in_mm = 2.0;
        req.lead_out_mm = 2.0;
        let section = slice_mesh(&mesh, req.plane, req.closure_tol_mm).expect("a section");
        let paths = section_to_toolpaths(&section, &req).expect("paths");
        assert_eq!(paths.len(), 1);
        assert!(!paths[0].closed, "a lead makes the path open");
        // 40 mm around plus 2 in and 2 out.
        assert!((paths[0].length_mm() - 44.0).abs() < 1e-6, "got {}", paths[0].length_mm());
    }

    #[test]
    fn the_whole_job_posts_machine_commands() {
        let mesh = cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 2.0));
        let profile = gcode_profile();
        let plane = Plane::named("XY").expect("the XY plane").offset(1.0);
        let req = CutJobRequest::new(&mesh, plane, &profile);
        let job = project_and_generate_cut_job(&req).expect("a job");
        assert_eq!(job.stats.loops, 1);
        assert_eq!(job.stats.contours, 1);
        assert!((job.stats.area_mm2 - 100.0).abs() < 1e-6);
        assert!(job.commands.contains("G21;G90;"), "the header is emitted");
        assert!(job.commands.contains("M2;"), "the footer is emitted");
        assert!(job.commands.contains("G1 X"), "a feed move is emitted");
    }

    /// The integration path: a `Region2` — what a sketch's `find_profiles` puts in `Profile.region`
    /// — cuts with no mesh and no plane, which is the whole point of `CutOptions`.
    #[test]
    fn a_region_cuts_its_outer_and_its_holes_without_a_mesh() {
        let outer = Loop2::polygon(&[
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ]);
        let hole = Loop2::polygon(&[
            Vec2::new(3.0, 3.0),
            Vec2::new(7.0, 3.0),
            Vec2::new(7.0, 7.0),
            Vec2::new(3.0, 7.0),
        ])
        .reversed();
        let region = Region2 { outer, holes: vec![hole] };
        let paths = region_to_toolpaths(&region, &CutOptions::default());
        assert_eq!(paths.len(), 2, "one outer and one hole");
        let mut areas: Vec<f64> = paths
            .iter()
            .map(|tp| {
                let pts: Vec<Vec2> = tp.segments.iter().map(|s| Vec2::new(s.start().x, s.start().y)).collect();
                ring_area(&pts).abs()
            })
            .collect();
        areas.sort_by(f64::total_cmp);
        assert!((areas[0] - 16.0).abs() < 1e-6, "the hole is 4x4, got {}", areas[0]);
        assert!((areas[1] - 100.0).abs() < 1e-6, "the outer is 10x10, got {}", areas[1]);
    }
}
