use glam::DVec3;

use space_sim::{ForceModel, Integrator, ManeuverNode, SimState};

use super::{
    phase::{dominant_body, determine_initial_phase, update_phase, PredictionPhase, TerminationReason},
    types::{PredictionConfig, PredictionResult, RawEncounter, TrailSegment},
    encounter::{detect_encounters, find_closest_approaches},
};

/// Reference altitude for adaptive step sizing (meters). Roughly Earth-radius scale.
const REFERENCE_ALTITUDE: f64 = 1e7;

/// Main prediction function. Propagates a ghost copy of the sim state forward,
/// generating trail data, detecting encounters, and tracking the phase machine.
pub fn predict(
    state: &SimState,
    ship_idx: usize,
    nodes: &[ManeuverNode],
    force_model: &dyn ForceModel,
    integrator: &dyn Integrator,
    config: &PredictionConfig,
    t0: f64,
) -> PredictionResult {
    let mut ghost = state.clone();
    let mut t = t0;

    // Sort nodes by time
    let mut sorted_nodes: Vec<&ManeuverNode> = nodes.iter().collect();
    sorted_nodes.sort_by(|a, b| {
        a.burn.time_window().0.partial_cmp(&b.burn.time_window().0).unwrap()
    });

    let mut phase = determine_initial_phase(&ghost, ship_idx, config);
    let mut segments = vec![TrailSegment::new(None)];
    let mut raw_encounters: Vec<RawEncounter> = Vec::new();
    let mut current_encounter: Option<RawEncounter> = None;
    let mut extend_remaining = config.extend_count;

    // Track which nodes have been passed (for segment splitting)
    let mut next_node_idx = 0;
    // Skip nodes that are already in the past
    while next_node_idx < sorted_nodes.len()
        && sorted_nodes[next_node_idx].burn.time_window().1 < t
    {
        next_node_idx += 1;
    }

    // Record initial position at t0
    {
        let dom = dominant_body(&ghost, ship_idx);
        let segment = segments.last_mut().unwrap();
        segment.push_point(
            ghost.positions[ship_idx],
            ghost.positions.clone(),
            t,
            dom,
            phase.clone(),
        );
    }

    for _step in 0..config.max_steps {
        // Adaptive dt
        let dt = compute_dt(&ghost, ship_idx, config, &phase);

        // Check if an impulse node falls within this step and needs substep splitting
        let mut remaining_dt = dt;
        let mut step_start = t;

        while remaining_dt > 1e-10 {
            // Find the earliest impulse in [step_start, step_start + remaining_dt)
            let mut earliest_impulse: Option<(usize, f64)> = None;
            for (ni, node) in sorted_nodes.iter().enumerate() {
                let (burn_start, burn_end) = node.burn.time_window();
                // Impulse burn: start == end
                if (burn_end - burn_start).abs() < 1e-10 {
                    if burn_start >= step_start && burn_start < step_start + remaining_dt {
                        if earliest_impulse.is_none() || burn_start < earliest_impulse.unwrap().1 {
                            earliest_impulse = Some((ni, burn_start));
                        }
                    }
                }
            }

            if let Some((ni, impulse_time)) = earliest_impulse {
                let dt_before = impulse_time - step_start;
                if dt_before > 1e-10 {
                    // Integrate to impulse time (gravity only, no burns)
                    do_gravity_step(&mut ghost, force_model, integrator, dt_before);
                    step_start += dt_before;
                    remaining_dt -= dt_before;
                }

                // Record pre-burn point in the current segment at the impulse time
                let dom = dominant_body(&ghost, ship_idx);
                {
                    let segment = segments.last_mut().unwrap();
                    segment.push_point(
                        ghost.positions[ship_idx],
                        ghost.positions.clone(),
                        step_start,
                        dom,
                        phase.clone(),
                    );
                }

                // Apply impulse as direct velocity change
                let _frame = space_sim::orbital_frame(
                    ghost.positions[ship_idx],
                    ghost.velocities[ship_idx],
                    ghost.positions[dom],
                    ghost.velocities[dom],
                );
                let node = &sorted_nodes[ni];
                if let Some(accel) = node.burn.acceleration(&ghost, ship_idx, dom, step_start, 1.0) {
                    ghost.velocities[ship_idx] += accel; // accel * 1.0 = world_dv
                }

                // Skip past this impulse
                step_start += 1e-10;
                remaining_dt -= 1e-10;

                // Segment split after this node
                if next_node_idx < sorted_nodes.len() {
                    let (_, end) = sorted_nodes[next_node_idx].burn.time_window();
                    if step_start > end {
                        let node_id = sorted_nodes[next_node_idx].id;
                        next_node_idx += 1;
                        segments.push(TrailSegment::new(Some(node_id)));

                        // Record post-burn point as first point in new segment
                        let segment = segments.last_mut().unwrap();
                        segment.push_point(
                            ghost.positions[ship_idx],
                            ghost.positions.clone(),
                            step_start,
                            dom,
                            phase.clone(),
                        );
                    }
                }
            } else {
                // No impulse in remaining interval — integrate normally (gravity only)
                do_gravity_step(&mut ghost, force_model, integrator, remaining_dt);
                step_start += remaining_dt;
                remaining_dt = 0.0;
            }
        }

        t = step_start;

        // Record trail point
        let dom = dominant_body(&ghost, ship_idx);
        {
            let segment = segments.last_mut().unwrap();
            segment.push_point(
                ghost.positions[ship_idx],
                ghost.positions.clone(),
                t,
                dom,
                phase.clone(),
            );
        }

        // Track encounter data
        let seg_idx = segments.len() - 1;
        let pt_idx = segments.last().unwrap().points.len() - 1;
        match &phase {
            PredictionPhase::Encounter {
                body_idx,
                entered_at,
                closest_so_far,
                ..
            } => {
                if current_encounter.is_none() {
                    current_encounter = Some(RawEncounter {
                        body_idx: *body_idx,
                        entry_time: *entered_at,
                        exit_time: t,
                        closest_distance: *closest_so_far,
                        closest_time: t,
                        closest_segment_idx: seg_idx,
                        closest_point_idx: pt_idx,
                        captured: false,
                        collided: false,
                    });
                }
                if let Some(ref mut enc) = current_encounter {
                    let dist = (ghost.positions[ship_idx] - ghost.positions[enc.body_idx]).length();
                    if dist < enc.closest_distance {
                        enc.closest_distance = dist;
                        enc.closest_time = t;
                        enc.closest_segment_idx = seg_idx;
                        enc.closest_point_idx = pt_idx;
                    }
                    enc.exit_time = t;
                }
            }
            _ => {
                if let Some(mut enc) = current_encounter.take() {
                    match &phase {
                        PredictionPhase::Orbiting { body_idx, .. } if *body_idx == enc.body_idx => {
                            enc.captured = true;
                        }
                        PredictionPhase::Done { reason: TerminationReason::Collision { .. } } => {
                            enc.collided = true;
                        }
                        _ => {}
                    }
                    raw_encounters.push(enc);
                }
            }
        }

        // Check for segment splits from non-impulse burns
        if next_node_idx < sorted_nodes.len() {
            let (_, end) = sorted_nodes[next_node_idx].burn.time_window();
            if t > end + 1e-10 {
                let node_id = sorted_nodes[next_node_idx].id;
                next_node_idx += 1;
                segments.push(TrailSegment::new(Some(node_id)));
            }
        }

        // Update phase machine
        let has_nodes_ahead = next_node_idx < sorted_nodes.len()
            || sorted_nodes.iter().any(|n| n.burn.time_window().1 > t);
        phase = update_phase(phase, &ghost, ship_idx, config, t, has_nodes_ahead);

        // Check termination
        if let PredictionPhase::Done { reason: _ } = phase {
            if extend_remaining > 0 {
                extend_remaining -= 1;
                let dom = dominant_body(&ghost, ship_idx);
                phase = PredictionPhase::Orbiting {
                    body_idx: dom,
                    start_angle: {
                        let r = ghost.positions[ship_idx] - ghost.positions[dom];
                        r.z.atan2(r.x)
                    },
                    crossed_half: false,
                };
            } else {
                break;
            }
        }
    }

    // Finalize any in-progress encounter
    if let Some(enc) = current_encounter.take() {
        raw_encounters.push(enc);
    }

    // If we hit the step limit without a Done phase, mark as budget exhausted
    let termination = match &phase {
        PredictionPhase::Done { reason } => reason.clone(),
        _ => TerminationReason::BudgetExhausted,
    };

    // Post-processing: detect encounters and closest approaches
    let encounters = detect_encounters(
        &raw_encounters,
        &segments,
        &config.body_radii,
        &state.masses,
    );

    let approaches = find_closest_approaches(
        &segments,
        ship_idx,
        &config.body_radii,
    );

    PredictionResult {
        segments,
        encounters,
        approaches,
        termination,
    }
}

/// Compute adaptive dt.
fn compute_dt(
    state: &SimState,
    ship_idx: usize,
    config: &PredictionConfig,
    phase: &PredictionPhase,
) -> f64 {
    if !config.adaptive_dt {
        return config.base_dt;
    }

    let mut min_altitude = f64::MAX;
    for i in 0..state.body_count() {
        if i == ship_idx {
            continue;
        }
        let dist = (state.positions[i] - state.positions[ship_idx]).length();
        let radius = if i < config.body_radii.len() {
            config.body_radii[i]
        } else {
            0.0
        };
        let alt = (dist - radius).max(1.0);
        min_altitude = min_altitude.min(alt);
    }

    let mut dt = config.base_dt * (min_altitude / REFERENCE_ALTITUDE).sqrt().clamp(0.01, 10.0);

    // Tighter dt when approaching target
    if let Some(target) = config.target_body {
        if target < state.body_count() {
            let to_target = state.positions[target] - state.positions[ship_idx];
            let vel = state.velocities[ship_idx] - state.velocities[target];
            let closing = -to_target.normalize().dot(vel);
            if closing > 0.0 {
                dt = dt.min(config.base_dt * 0.2);
            }
        }
    }

    // Finer dt during encounters
    if matches!(phase, PredictionPhase::Encounter { .. }) {
        dt = dt.min(config.base_dt * 0.1);
    }

    dt.max(0.01)
}

/// Perform one integration step with gravity only (no burns).
fn do_gravity_step(
    ghost: &mut SimState,
    force_model: &dyn ForceModel,
    integrator: &dyn Integrator,
    dt: f64,
) {
    let accel_fn = |s: &SimState| -> Vec<DVec3> {
        force_model.compute_accelerations(s)
    };
    integrator.step(ghost, &accel_fn, dt);
}
