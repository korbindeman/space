use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::sim::*;
use super::maneuver::ManeuverPlan;
use super::prediction::*;

// --- Plugin ---

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(bevy_egui::EguiPrimaryContextPass, time_control_panel);
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

