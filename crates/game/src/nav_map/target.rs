use bevy::prelude::*;
use bevy::math::DVec3;
use bevy::window::PrimaryWindow;
use bevy_egui::{egui, EguiContexts};

use space_prediction::types::CaptureStatus;

use crate::sim::*;
use super::camera::CameraFocus;
use super::prediction::PredictionCache;
use super::{RENDER_SCALE, format_distance, format_duration};

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
            .add_systems(Update, target_input)
            .add_systems(bevy_egui::EguiPrimaryContextPass, encounter_info_panel);
    }
}

// --- Systems ---

/// Handle right-click targeting and T key.
fn target_input(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_q: Query<(&Camera, &GlobalTransform), (With<Camera3d>, Without<super::camera::OverlayCamera>)>,
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
            ui.label(format!("Inclination: {:.1}\u{00b0}", enc.inclination.to_degrees()));
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
