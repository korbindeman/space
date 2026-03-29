use bevy::prelude::*;

use crate::sim::*;
use crate::camera::CameraFocus;
use crate::prediction::PredictionCache;

/// Body colors for trail segments.
const BODY_COLORS: &[Color] = &[
    Color::srgb(1.0, 0.95, 0.3),  // Sun - yellow
    Color::srgb(0.3, 0.5, 1.0),   // Earth - blue
    Color::srgb(0.6, 0.6, 0.6),   // Moon - grey
    Color::srgb(0.0, 1.0, 0.5),   // Ship - green
];

// --- Plugin ---

pub struct TrailPlugin;

impl Plugin for TrailPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, render_trail);
    }
}

// --- Systems ---

/// Draw predicted trail using gizmos. Trail is rendered in the dominant body's
/// reference frame, then offset to the camera frame (which may be the ship).
fn render_trail(
    cache: Res<PredictionCache>,
    camera_focus: Res<CameraFocus>,
    live_dom: Res<LiveDominantBody>,
    physics: Res<PhysicsState>,
    mut gizmos: Gizmos,
) {
    let Some(ref result) = cache.result else {
        return;
    };

    let trail_frame = live_dom.index;
    let camera_frame = camera_focus.active_frame;
    let offset = trail_frame_offset(&physics, trail_frame, camera_frame);

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
                trail_frame,
            ) + offset;
            let p1 = frame_relative_pos(
                seg.points[i],
                &seg.body_positions[i],
                trail_frame,
            ) + offset;

            gizmos.line(p0, p1, color);
        }
    }

    // Draw closest approach markers
    for approach in &result.approaches {
        let camera_pos = if camera_frame < physics.state.positions.len() {
            physics.state.positions[camera_frame]
        } else {
            bevy::math::DVec3::ZERO
        };
        let p = ((approach.position - camera_pos) * RENDER_SCALE).as_vec3();
        gizmos.sphere(Isometry3d::from_translation(p), 0.1, Color::srgb(1.0, 1.0, 0.0));
    }
}

// --- Helpers ---

/// Compute the render-space offset from trail_frame to camera_frame.
pub fn trail_frame_offset(
    physics: &PhysicsState,
    trail_frame: usize,
    camera_frame: usize,
) -> Vec3 {
    if trail_frame == camera_frame {
        return Vec3::ZERO;
    }
    let trail_pos = if trail_frame < physics.state.positions.len() {
        physics.state.positions[trail_frame]
    } else {
        bevy::math::DVec3::ZERO
    };
    let cam_pos = if camera_frame < physics.state.positions.len() {
        physics.state.positions[camera_frame]
    } else {
        bevy::math::DVec3::ZERO
    };
    ((trail_pos - cam_pos) * RENDER_SCALE).as_vec3()
}

fn frame_relative_pos(
    ship_pos: bevy::math::DVec3,
    body_positions: &[bevy::math::DVec3],
    frame_body: usize,
) -> Vec3 {
    let frame_pos = if frame_body < body_positions.len() {
        body_positions[frame_body]
    } else {
        bevy::math::DVec3::ZERO
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
