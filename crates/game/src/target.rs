use bevy::prelude::*;
use bevy::math::DVec3;
use bevy::window::PrimaryWindow;
use bevy_egui::{egui, EguiContexts};

use space_prediction::types::CaptureStatus;

use crate::sim::*;
use crate::camera::CameraFocus;
use crate::prediction::PredictionCache;
use crate::trail;

// --- Resources ---

#[derive(Resource, Default)]
pub struct TargetBody {
    pub target: Option<usize>,
}

// --- Plugin ---

pub struct TargetPlugin;

impl Plugin for TargetPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TargetBody>()
            .add_systems(Update, (
                target_input,
                render_target_ghost_trail,
            ))
            .add_systems(bevy_egui::EguiPrimaryContextPass, encounter_info_panel);
    }
}

// --- Systems ---

/// Handle right-click targeting and T key.
fn target_input(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_q: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    physics: Res<PhysicsState>,
    camera_focus: Res<CameraFocus>,
    bodies: Query<(&SimBody, &CelestialBody)>,
    mut target: ResMut<TargetBody>,
) {
    if !keys.just_pressed(KeyCode::KeyT) && !mouse.just_pressed(MouseButton::Right) {
        return;
    }

    if mouse.just_pressed(MouseButton::Right) && !keys.pressed(KeyCode::ShiftLeft) {
        return;
    }

    let Ok(window) = windows.single() else { return };
    let Some(cursor_pos) = window.cursor_position() else { return };
    let Ok((camera, cam_transform)) = camera_q.single() else { return };

    let frame_body = camera_focus.active_frame;
    let frame_pos = if frame_body < physics.state.positions.len() {
        physics.state.positions[frame_body]
    } else {
        DVec3::ZERO
    };

    let mut closest_idx = None;
    let mut closest_dist = 30.0_f32;

    for (sim_body, _cb) in &bodies {
        let world = ((physics.state.positions[sim_body.0] - frame_pos) * RENDER_SCALE).as_vec3();
        let Some(screen) = camera.world_to_viewport(cam_transform, world).ok() else {
            continue;
        };
        let dist = (screen - cursor_pos).length();
        if dist < closest_dist {
            closest_dist = dist;
            closest_idx = Some(sim_body.0);
        }
    }

    if let Some(idx) = closest_idx {
        if target.target == Some(idx) {
            target.target = None;
        } else {
            target.target = Some(idx);
        }
    } else if keys.just_pressed(KeyCode::KeyT) {
        target.target = None;
    }
}

/// Right panel: Encounter info (when target set and encounter exists).
fn encounter_info_panel(
    mut contexts: EguiContexts,
    cache: Res<PredictionCache>,
    target: Res<TargetBody>,
    clock: Res<SimClock>,
    bodies: Query<(&SimBody, &CelestialBody)>,
) {
    let Some(target_idx) = target.target else { return };

    let body_name = bodies
        .iter()
        .find(|(sb, _)| sb.0 == target_idx)
        .map(|(_, cb)| cb.name.as_str())
        .unwrap_or("Unknown");

    let Ok(ctx) = contexts.ctx_mut() else { return };
    egui::SidePanel::right("encounter_info").default_width(220.0).show(ctx, |ui| {
        let Some(ref result) = cache.result else {
            ui.heading(format!("{} (targeted)", body_name));
            ui.label("No prediction data");
            return;
        };

        let encounter = result.encounters.iter().find(|e| e.body_idx == target_idx);

        if let Some(enc) = encounter {
            ui.heading(format!("{} Encounter", body_name));
            ui.separator();

            let body_radius = bodies
                .iter()
                .find(|(sb, _)| sb.0 == target_idx)
                .map(|(_, cb)| cb.radius)
                .unwrap_or(0.0);

            ui.label(format!("Closest approach: {}", format_distance(enc.closest_approach - body_radius)));
            ui.label(format!("Relative velocity: {:.0} m/s", enc.relative_velocity));

            let time_to = enc.entry_time - clock.time;
            if time_to > 0.0 {
                ui.label(format!("Time to encounter: {}", format_duration(time_to)));
            } else {
                ui.label("Encounter in progress");
            }

            let capture_display = match &enc.capture {
                CaptureStatus::Flyby => format!("Flyby (e = {:.2})", enc.eccentricity),
                CaptureStatus::Captured => format!("Captured (e = {:.2})", enc.eccentricity),
                CaptureStatus::Impact => "Impact!".to_string(),
                CaptureStatus::Graze { altitude } => format!("Graze ({:.0} km)", altitude / 1000.0),
            };
            ui.label(format!("Orbit type: {}", capture_display));
            ui.label(format!("Inclination: {:.1}°", enc.inclination.to_degrees()));
            ui.label(format!("Periapsis alt: {}", format_distance(enc.periapsis_altitude)));
        } else {
            ui.heading(format!("{} (targeted)", body_name));
            ui.separator();
            ui.label("No encounter detected");

            if let Some(approach) = result.approaches.iter().find(|a| a.body_idx == target_idx) {
                let body_radius = bodies
                    .iter()
                    .find(|(sb, _)| sb.0 == target_idx)
                    .map(|(_, cb)| cb.radius)
                    .unwrap_or(0.0);
                ui.label(format!(
                    "Distance at closest pass: {}",
                    format_distance(approach.distance - body_radius)
                ));
            }
        }
    });
}

/// Draw ghost trail in target body's frame at lower opacity.
fn render_target_ghost_trail(
    cache: Res<PredictionCache>,
    target: Res<TargetBody>,
    camera_focus: Res<CameraFocus>,
    physics: Res<PhysicsState>,
    mut gizmos: Gizmos,
) {
    let Some(target_idx) = target.target else { return };
    let Some(ref result) = cache.result else { return };

    let offset = trail::trail_frame_offset(&physics, target_idx, camera_focus.active_frame);

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

            let p0 = ((seg.points[i - 1] - frame_pos_prev) * RENDER_SCALE).as_vec3() + offset;
            let p1 = ((seg.points[i] - frame_pos_curr) * RENDER_SCALE).as_vec3() + offset;

            let color = Color::srgba(0.8, 0.6, 1.0, 0.3);
            gizmos.line(p0, p1, color);
        }
    }
}

// --- Helpers ---

fn format_distance(meters: f64) -> String {
    let abs = meters.abs();
    if abs < 1_000.0 {
        format!("{:.0} m", meters)
    } else if abs < 1_000_000.0 {
        format!("{:.1} km", meters / 1_000.0)
    } else if abs < 1e9 {
        format!("{:.0} km", meters / 1_000.0)
    } else {
        format!("{:.3} AU", meters / 1.496e11)
    }
}

fn format_duration(seconds: f64) -> String {
    if seconds.is_infinite() || seconds.is_nan() {
        return "\u{221e}".to_string();
    }
    let s = seconds.abs();
    if s < 60.0 {
        format!("{:.0}s", s)
    } else if s < 3600.0 {
        format!("{:.0}m {:.0}s", s / 60.0, s % 60.0)
    } else if s < 86400.0 {
        format!("{:.0}h {:.0}m", s / 3600.0, (s % 3600.0) / 60.0)
    } else {
        format!("{:.1}d", s / 86400.0)
    }
}
