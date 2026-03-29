use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use space_sim::*;

use crate::sim::*;
use super::maneuver::ManeuverPlan;
use super::prediction::*;
use super::{format_distance, format_duration};

// --- Plugin ---

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(bevy_egui::EguiPrimaryContextPass, (
            time_control_panel,
            orbital_info_panel,
        ));
    }
}

// --- Systems ---

/// Top bar: Time control panel.
fn time_control_panel(
    mut contexts: EguiContexts,
    mut clock: ResMut<SimClock>,
    cache: Res<PredictionCache>,
    mut predict_further: ResMut<PredictFurther>,
    mut plan: ResMut<ManeuverPlan>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    egui::TopBottomPanel::top("time_control").show(ctx, |ui| {
        ui.horizontal(|ui| {
            if ui.button(if clock.paused() { "\u{25b6} Resume" } else { "\u{23f8} Pause" }).clicked() {
                clock.toggle_pause();
            }

            ui.separator();

            ui.label("Warp:");
            if ui.button("<").clicked() && clock.warp_index > 0 {
                clock.warp_index -= 1;
                clock.warp = clock.warp_levels[clock.warp_index];
            }
            let warp_label = if clock.warp >= 1_000_000.0 {
                format!("{:.0}Mx", clock.warp / 1_000_000.0)
            } else if clock.warp >= 1_000.0 {
                format!("{:.0}kx", clock.warp / 1_000.0)
            } else if clock.warp > 0.0 {
                format!("{:.0}x", clock.warp)
            } else {
                "||".to_string()
            };
            ui.label(warp_label);
            if ui.button(">").clicked() && clock.warp_index + 1 < clock.warp_levels.len() {
                clock.warp_index += 1;
                clock.warp = clock.warp_levels[clock.warp_index];
            }

            ui.separator();

            let total_seconds = clock.time;
            let days = (total_seconds / 86400.0) as u64;
            let hours = ((total_seconds % 86400.0) / 3600.0) as u64;
            let minutes = ((total_seconds % 3600.0) / 60.0) as u64;
            ui.label(format!("T+ {}d {:02}h {:02}m", days, hours, minutes));

            ui.separator();

            if let Some(ref result) = cache.result {
                if matches!(result.termination, space_prediction::TerminationReason::BudgetExhausted) {
                    ui.colored_label(egui::Color32::YELLOW, "Prediction limit reached");
                }
            }

            if ui.button("Predict further (P)").clicked() {
                predict_further.count += 1;
                plan.dirty = true;
            }
        });
    });
}

/// Left panel: Orbital info.
fn orbital_info_panel(
    mut contexts: EguiContexts,
    physics: Res<PhysicsState>,
    live_dom: Res<LiveDominantBody>,
    ship_config: Res<ShipConfig>,
    bodies: Query<(&SimBody, &CelestialBody)>,
) {
    let ship_idx = ship_config.sim_index;
    let dom_idx = live_dom.index;

    let body_name = bodies
        .iter()
        .find(|(sb, _)| sb.0 == dom_idx)
        .map(|(_, cb)| cb.name.as_str())
        .unwrap_or("Unknown");

    let body_radius = bodies
        .iter()
        .find(|(sb, _)| sb.0 == dom_idx)
        .map(|(_, cb)| cb.radius)
        .unwrap_or(0.0);

    let rel_pos = physics.state.positions[ship_idx] - physics.state.positions[dom_idx];
    let rel_vel = physics.state.velocities[ship_idx] - physics.state.velocities[dom_idx];
    let mu = G * physics.state.masses[dom_idx];
    let elements = orbital_elements(rel_pos, rel_vel, mu);

    let altitude = rel_pos.length() - body_radius;
    let velocity = rel_vel.length();

    let Ok(ctx) = contexts.ctx_mut() else { return };
    egui::SidePanel::left("orbital_info").default_width(220.0).show(ctx, |ui| {
        ui.heading("Orbital Info");
        ui.separator();

        ui.label(format!("Reference: {}", body_name));
        ui.label(format!("Altitude: {}", format_distance(altitude)));
        ui.label(format!("Velocity: {:.1} m/s", velocity));

        ui.separator();

        ui.label(format!("SMA: {}", format_distance(elements.semi_major_axis)));
        ui.label(format!("Eccentricity: {:.4}", elements.eccentricity));
        ui.label(format!("Inclination: {:.1}\u{00b0}", elements.inclination.to_degrees()));
        ui.label(format!("Period: {}", format_duration(elements.period)));
        ui.label(format!("Periapsis: {}", format_distance(elements.periapsis - body_radius)));
        if elements.eccentricity < 1.0 {
            ui.label(format!("Apoapsis: {}", format_distance(elements.apoapsis - body_radius)));
        }
    });
}
