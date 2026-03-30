use bevy::prelude::*;
use bevy::math::DVec3;

use space_sim::gravity::G;
use space_sim::orbital_elements;

use crate::sim::{BodyData, LiveDominantBody, PhysicsState, ShipConfig, TrailFrame};
use super::camera::CameraFocus;
use super::prediction::PredictionCache;
use super::target::TargetBody;
use super::{RENDER_SCALE, BODY_COLORS};

/// Number of sample points per body orbit line.
const ORBIT_SAMPLES: usize = 256;

// --- Resources ---

/// Cached N-body orbit trails for celestial bodies. Computed once at startup.
#[derive(Resource, Default)]
pub struct BodyOrbitCache {
    /// `orbits[child_idx]` = `Some((parent_idx, positions_relative_to_parent))`
    pub orbits: Vec<Option<(usize, Vec<DVec3>)>>,
}

// --- Plugin ---

pub struct TrajectoryPlugin;

impl Plugin for TrajectoryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<BodyOrbitCache>()
            .add_systems(Startup, compute_body_orbits.after(crate::sim::setup))
            .add_systems(Update, render_trajectories);
    }
}

// --- Systems ---

/// Propagate N-body sim to cache one full orbit for each celestial body.
fn compute_body_orbits(
    physics: Res<PhysicsState>,
    ship_config: Res<ShipConfig>,
    body_data: Res<BodyData>,
    mut orbit_cache: ResMut<BodyOrbitCache>,
) {
    let num_bodies = physics.state.positions.len();
    let ship_idx = ship_config.sim_index;
    orbit_cache.orbits = vec![None; num_bodies];

    for body_idx in 0..num_bodies {
        if body_idx == ship_idx {
            continue;
        }

        let parent_idx = find_grav_parent(body_idx, ship_idx, &physics, &body_data);
        let Some(parent_idx) = parent_idx else { continue };

        let rel_pos = physics.state.positions[body_idx] - physics.state.positions[parent_idx];
        let rel_vel = physics.state.velocities[body_idx] - physics.state.velocities[parent_idx];
        let mu = G * physics.state.masses[parent_idx];
        let elements = orbital_elements(rel_pos, rel_vel, mu);

        if elements.eccentricity >= 1.0 || elements.semi_major_axis <= 0.0 || elements.period.is_infinite() {
            continue;
        }

        let period = elements.period;
        let dt = period / ORBIT_SAMPLES as f64;
        let mut ghost = physics.state.clone();
        let accel_fn = |s: &space_sim::SimState| physics.force_model.compute_accelerations(s);

        let mut trail = Vec::with_capacity(ORBIT_SAMPLES + 1);
        trail.push(ghost.positions[body_idx] - ghost.positions[parent_idx]);

        for _ in 0..ORBIT_SAMPLES {
            physics.integrator.step(&mut ghost, &accel_fn, dt);
            trail.push(ghost.positions[body_idx] - ghost.positions[parent_idx]);
        }

        orbit_cache.orbits[body_idx] = Some((parent_idx, trail));
    }
}

/// Render all trajectories: body orbits, ship trail, and target ghost trail.
fn render_trajectories(
    cache: Res<PredictionCache>,
    trail_frame: Res<TrailFrame>,
    target: Res<TargetBody>,
    physics: Res<PhysicsState>,
    camera_focus: Res<CameraFocus>,
    ship_config: Res<ShipConfig>,
    live_dom: Res<LiveDominantBody>,
    orbit_cache: Res<BodyOrbitCache>,
    mut gizmos: Gizmos,
) {
    // Body orbit lines (independent of prediction cache — renders on first frame)
    let focus_idx = camera_focus.active_frame;
    let parent_idx = if focus_idx == ship_config.sim_index {
        live_dom.index
    } else {
        focus_idx
    };
    render_cached_orbits(parent_idx, &physics, &orbit_cache, &trail_frame, &mut gizmos);

    // Everything below requires prediction data
    let Some(ref result) = cache.result else {
        return;
    };

    // Ship trail — original formula: relative to trail_frame body, continuous line
    for seg in &result.segments {
        if seg.points.len() < 2 {
            continue;
        }

        for i in 1..seg.points.len() {
            let dom = seg.dominant_body[i];
            let color = trail_color(dom, i, seg.points.len());

            let p0 = frame_relative_pos(
                seg.points[i - 1],
                &seg.body_positions[i - 1],
                trail_frame.index,
            ) + trail_frame.offset;
            let p1 = frame_relative_pos(
                seg.points[i],
                &seg.body_positions[i],
                trail_frame.index,
            ) + trail_frame.offset;

            gizmos.line(p0, p1, color);
        }
    }

    // Target ghost trail (if target set)
    let Some(target_idx) = target.target else { return };
    let target_offset = target_frame_offset(&physics, target_idx, &trail_frame);

    for seg in &result.segments {
        if seg.points.len() < 2 {
            continue;
        }

        for i in 1..seg.points.len() {
            let frame_pos_prev = if target_idx < seg.body_positions[i - 1].len() {
                seg.body_positions[i - 1][target_idx]
            } else {
                continue;
            };
            let frame_pos_curr = if target_idx < seg.body_positions[i].len() {
                seg.body_positions[i][target_idx]
            } else {
                continue;
            };

            let p0 = ((seg.points[i - 1] - frame_pos_prev) * RENDER_SCALE).as_vec3() + target_offset;
            let p1 = ((seg.points[i] - frame_pos_curr) * RENDER_SCALE).as_vec3() + target_offset;

            let color = Color::srgba(0.8, 0.6, 1.0, 0.3);
            gizmos.line(p0, p1, color);
        }
    }
}

// --- Helpers ---

/// Compute render-space offset for the target body relative to the camera frame.
fn target_frame_offset(physics: &PhysicsState, target_idx: usize, trail_frame: &TrailFrame) -> Vec3 {
    let target_pos = physics.state.positions.get(target_idx).copied().unwrap_or(DVec3::ZERO);
    let trail_pos = physics.state.positions.get(trail_frame.index).copied().unwrap_or(DVec3::ZERO);
    ((target_pos - trail_pos) * RENDER_SCALE).as_vec3() + trail_frame.offset
}

pub fn frame_relative_pos(
    ship_pos: DVec3,
    body_positions: &[DVec3],
    frame_body: usize,
) -> Vec3 {
    let frame_pos = if frame_body < body_positions.len() {
        body_positions[frame_body]
    } else {
        DVec3::ZERO
    };
    ((ship_pos - frame_pos) * RENDER_SCALE).as_vec3()
}

/// Find the gravitational parent of a body (smallest Hill sphere containing it).
/// For bodies not inside any Hill sphere, returns the root body (mass-largest with hill=0).
fn find_grav_parent(
    body_idx: usize,
    ship_idx: usize,
    physics: &PhysicsState,
    body_data: &BodyData,
) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for i in 0..physics.state.positions.len() {
        if i == body_idx || i == ship_idx {
            continue;
        }
        let hill = body_data.hill_radii.get(i).copied().unwrap_or(0.0);
        if hill > 0.0 {
            let dist = (physics.state.positions[body_idx] - physics.state.positions[i]).length();
            if dist < hill && (best.is_none() || hill < best.unwrap().1) {
                best = Some((i, hill));
            }
        }
    }
    if let Some((idx, _)) = best {
        return Some(idx);
    }
    for i in 0..physics.state.positions.len() {
        if i == body_idx || i == ship_idx {
            continue;
        }
        let hill = body_data.hill_radii.get(i).copied().unwrap_or(0.0);
        if hill == 0.0 && physics.state.masses.get(i).copied().unwrap_or(0.0) > 0.0 {
            return Some(i);
        }
    }
    None
}

/// Render cached N-body orbit trails for children of `parent_idx`.
/// If `parent_idx` is a leaf node, show its own orbit around its parent instead.
fn render_cached_orbits(
    parent_idx: usize,
    physics: &PhysicsState,
    orbit_cache: &BodyOrbitCache,
    trail_frame: &TrailFrame,
    gizmos: &mut Gizmos,
) {
    let mut to_render: Vec<usize> = Vec::new();
    for (body_idx, entry) in orbit_cache.orbits.iter().enumerate() {
        if let Some((cached_parent, _)) = entry {
            if *cached_parent == parent_idx {
                to_render.push(body_idx);
            }
        }
    }

    if to_render.is_empty() {
        if let Some(Some((_, _))) = orbit_cache.orbits.get(parent_idx) {
            to_render.push(parent_idx);
        }
    }

    for body_idx in to_render {
        let Some(Some((cached_parent, trail))) = orbit_cache.orbits.get(body_idx) else {
            continue;
        };
        if trail.len() < 2 {
            continue;
        }

        let base_color = if body_idx < BODY_COLORS.len() {
            BODY_COLORS[body_idx]
        } else {
            Color::WHITE
        };
        let color = base_color.with_alpha(0.15);

        let parent_pos = physics.state.positions.get(*cached_parent).copied().unwrap_or(DVec3::ZERO);
        let frame_pos = physics.state.positions.get(trail_frame.index).copied().unwrap_or(DVec3::ZERO);
        let offset = ((parent_pos - frame_pos) * RENDER_SCALE).as_vec3() + trail_frame.offset;

        for i in 1..trail.len() {
            let p0 = (trail[i - 1] * RENDER_SCALE).as_vec3() + offset;
            let p1 = (trail[i] * RENDER_SCALE).as_vec3() + offset;
            gizmos.line(p0, p1, color);
        }
    }
}

fn trail_color(dominant_body: usize, point_idx: usize, total_points: usize) -> Color {
    let base = if dominant_body < BODY_COLORS.len() {
        BODY_COLORS[dominant_body]
    } else {
        Color::WHITE
    };

    let progress = point_idx as f32 / total_points as f32;
    let alpha = 0.3 + 0.7 * (1.0 - progress);

    base.with_alpha(alpha)
}
