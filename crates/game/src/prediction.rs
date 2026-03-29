use bevy::prelude::*;

use space_prediction::PredictionResult;

use crate::sim::*;
use crate::maneuver::ManeuverPlan;
use crate::target::TargetBody;

// --- Resources ---

#[derive(Resource)]
pub struct PredictionCache {
    pub result: Option<PredictionResult>,
    pub computed_at: f64,
}

impl Default for PredictionCache {
    fn default() -> Self {
        Self {
            result: None,
            computed_at: f64::NEG_INFINITY,
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
    sim_bodies: Query<(&SimBody, &CelestialBody)>,
    predict_further: Res<PredictFurther>,
) {
    let needs_update = plan.dirty || (clock.time - cache.computed_at).abs() > 0.5;
    if !needs_update {
        return;
    }

    // Build prediction config from current state
    let mut body_radii = Vec::new();
    let mut body_hill_radii = Vec::new();
    let mut body_parent = Vec::new();

    // Collect body data sorted by sim index
    let mut body_data: Vec<(usize, &CelestialBody)> = sim_bodies
        .iter()
        .map(|(sb, cb)| (sb.0, cb))
        .collect();
    body_data.sort_by_key(|(idx, _)| *idx);

    for (_, cb) in &body_data {
        body_radii.push(cb.radius);
        body_hill_radii.push(cb.hill_radius);
        body_parent.push(None); // simplified for now
    }

    let config = space_prediction::PredictionConfig {
        max_steps: 10_000,
        base_dt: 60.0,
        adaptive_dt: true,
        target_body: target.target,
        body_radii,
        body_hill_radii,
        body_parent,
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
    plan.dirty = false;
}
