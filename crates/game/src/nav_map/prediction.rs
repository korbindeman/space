use bevy::prelude::*;

use space_prediction::PredictionResult;

use crate::sim::{PhysicsState, SimClock, ShipConfig, BodyData};
use super::maneuver::ManeuverPlan;
use super::target::TargetBody;

// --- Resources ---

#[derive(Resource)]
pub struct PredictionCache {
    pub result: Option<PredictionResult>,
    pub computed_at: f64,
    /// Incremented each time the prediction is recomputed.
    pub generation: u64,
}

impl Default for PredictionCache {
    fn default() -> Self {
        Self {
            result: None,
            computed_at: f64::NEG_INFINITY,
            generation: 0,
        }
    }
}

/// "Predict further" press count.
#[derive(Resource, Default)]
pub struct PredictFurther {
    pub count: usize,
}

// --- Plugin ---

pub struct PredictionPlugin;

impl Plugin for PredictionPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PredictionCache>()
            .init_resource::<PredictFurther>()
            .add_systems(Update, (
                predict_further_input,
                run_prediction,
            ));
    }
}

// --- Systems ---

/// Handle P key to extend prediction length.
fn predict_further_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut predict_further: ResMut<PredictFurther>,
    mut plan: ResMut<ManeuverPlan>,
) {
    if keys.just_pressed(KeyCode::KeyP) {
        predict_further.count += 1;
        plan.dirty = true;
    }
}

/// Run prediction when plan is dirty or periodically.
fn run_prediction(
    physics: Res<PhysicsState>,
    clock: Res<SimClock>,
    mut plan: ResMut<ManeuverPlan>,
    mut cache: ResMut<PredictionCache>,
    ship_config: Res<ShipConfig>,
    target: Res<TargetBody>,
    body_data: Res<BodyData>,
    predict_further: Res<PredictFurther>,
) {
    let needs_update = plan.dirty || (clock.time - cache.computed_at).abs() > 0.5;
    if !needs_update {
        return;
    }

    let config = space_prediction::PredictionConfig {
        max_steps: 10_000,
        base_dt: 60.0,
        adaptive_dt: true,
        target_body: target.target,
        body_radii: body_data.radii.clone(),
        body_hill_radii: body_data.hill_radii.clone(),
        body_parent: vec![None; body_data.radii.len()],
        extend_count: predict_further.count,
    };

    let result = space_prediction::predict(
        &physics.state,
        ship_config.sim_index,
        &plan.nodes,
        physics.force_model.as_ref(),
        physics.integrator.as_ref(),
        &config,
        clock.time,
    );

    cache.result = Some(result);
    cache.computed_at = clock.time;
    cache.generation += 1;
    plan.dirty = false;
}
