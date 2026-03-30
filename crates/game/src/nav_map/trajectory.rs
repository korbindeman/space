use bevy::prelude::*;
use bevy::math::DVec3;

use crate::sim::{PhysicsState, TrailFrame};
use super::prediction::PredictionCache;
use super::target::TargetBody;
use super::{RENDER_SCALE, BODY_COLORS};

// --- Plugin ---

pub struct TrajectoryPlugin;

impl Plugin for TrajectoryPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, render_trajectories);
    }
}

// --- Systems ---

/// Render all trajectories: ship trail in trail frame, plus target ghost trail.
fn render_trajectories(
    cache: Res<PredictionCache>,
    trail_frame: Res<TrailFrame>,
    target: Res<TargetBody>,
    physics: Res<PhysicsState>,
    mut gizmos: Gizmos,
) {
    let Some(ref result) = cache.result else {
        return;
    };

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
