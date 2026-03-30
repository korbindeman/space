use glam::DVec3;

use space_sim::{gravity::G, orbital_elements, SimState};

use super::types::PredictionConfig;

#[derive(Clone, Debug)]
pub enum PredictionPhase {
    Orbiting {
        body_idx: usize,
        start_angle: f64,
        crossed_half: bool,
    },
    Transfer {
        origin_body: usize,
    },
    Encounter {
        body_idx: usize,
        entered_at: f64,
        closest_so_far: f64,
        passed_closest: bool,
    },
    Escaping {
        from_body: usize,
        escape_distance: f64,
    },
    Done {
        reason: TerminationReason,
    },
}

#[derive(Clone, Debug)]
pub enum TerminationReason {
    OrbitClosed,
    Collision { body_idx: usize },
    EncounterResolved,
    EscapeComplete,
    BudgetExhausted,
}

/// Compute the radial angle of the ship relative to a body in the XZ plane.
fn radial_angle(ship_pos: DVec3, body_pos: DVec3) -> f64 {
    let r = ship_pos - body_pos;
    r.z.atan2(r.x)
}

/// Find the dominant body for the ship using Hill sphere membership.
/// Picks the body whose Hill sphere contains the ship and is the smallest
/// (most local). Falls back to strongest gravitational acceleration if not
/// inside any Hill sphere.
pub fn dominant_body(state: &SimState, ship_idx: usize, hill_radii: &[f64]) -> usize {
    let ship_pos = state.positions[ship_idx];

    // Find the smallest enclosing Hill sphere
    let mut best_hill_idx: Option<usize> = None;
    let mut best_hill_radius = f64::MAX;

    for i in 0..state.body_count() {
        if i == ship_idx {
            continue;
        }
        let hill_r = if i < hill_radii.len() { hill_radii[i] } else { 0.0 };
        if hill_r <= 0.0 {
            continue;
        }
        let dist = (state.positions[i] - ship_pos).length();
        if dist < hill_r && hill_r < best_hill_radius {
            best_hill_idx = Some(i);
            best_hill_radius = hill_r;
        }
    }

    if let Some(idx) = best_hill_idx {
        return idx;
    }

    // Fallback: strongest gravitational acceleration (for Sun or bodies without Hill radii)
    let mut best_idx = 0;
    let mut best_accel = 0.0_f64;

    for i in 0..state.body_count() {
        if i == ship_idx {
            continue;
        }
        let r = (state.positions[i] - ship_pos).length();
        let accel = G * state.masses[i] / (r * r);
        if accel > best_accel {
            best_accel = accel;
            best_idx = i;
        }
    }
    best_idx
}

/// Determine the initial prediction phase.
pub fn determine_initial_phase(
    state: &SimState,
    ship_idx: usize,
    config: &PredictionConfig,
) -> PredictionPhase {
    let dom = dominant_body(state, ship_idx, &config.body_hill_radii);
    let rel_pos = state.positions[ship_idx] - state.positions[dom];
    let rel_vel = state.velocities[ship_idx] - state.velocities[dom];
    let mu = G * state.masses[dom];

    let specific_energy = rel_vel.length_squared() / 2.0 - mu / rel_pos.length();

    if specific_energy < 0.0 {
        PredictionPhase::Orbiting {
            body_idx: dom,
            start_angle: radial_angle(state.positions[ship_idx], state.positions[dom]),
            crossed_half: false,
        }
    } else {
        PredictionPhase::Transfer {
            origin_body: dom,
        }
    }
}

/// Update the prediction phase based on current state.
pub fn update_phase(
    phase: PredictionPhase,
    state: &SimState,
    ship_idx: usize,
    config: &PredictionConfig,
    time: f64,
    has_nodes_ahead: bool,
) -> PredictionPhase {
    let dom = dominant_body(state, ship_idx, &config.body_hill_radii);

    match phase {
        PredictionPhase::Orbiting {
            body_idx,
            start_angle,
            crossed_half,
        } => {
            let ship_pos = state.positions[ship_idx];
            let body_pos = state.positions[body_idx];
            let rel_pos = ship_pos - body_pos;
            let rel_vel = state.velocities[ship_idx] - state.velocities[body_idx];
            let dist = rel_pos.length();
            let mu = G * state.masses[body_idx];

            // Check for collision
            if body_idx < config.body_radii.len() && dist < config.body_radii[body_idx] {
                return PredictionPhase::Done {
                    reason: TerminationReason::Collision { body_idx },
                };
            }

            // Check if dominant body changed (transition to transfer)
            if dom != body_idx {
                if dom < config.body_hill_radii.len() {
                    let dist_to_dom = (ship_pos - state.positions[dom]).length();
                    if dist_to_dom < config.body_hill_radii[dom] {
                        return PredictionPhase::Encounter {
                            body_idx: dom,
                            entered_at: time,
                            closest_so_far: dist_to_dom,
                            passed_closest: false,
                        };
                    }
                }
                return PredictionPhase::Transfer {
                    origin_body: body_idx,
                };
            }

            // Check specific orbital energy — became unbound?
            let specific_energy = rel_vel.length_squared() / 2.0 - mu / dist;
            if specific_energy > 0.0 {
                return PredictionPhase::Transfer {
                    origin_body: body_idx,
                };
            }

            // Check orbit closure via radial angle crossing
            let current_angle = radial_angle(ship_pos, body_pos);
            let angle_diff = angle_wrap(current_angle - start_angle);

            if !crossed_half && angle_diff.abs() > std::f64::consts::PI * 0.4 {
                return PredictionPhase::Orbiting {
                    body_idx,
                    start_angle,
                    crossed_half: true,
                };
            }

            if crossed_half {
                if angle_diff.abs() < 0.1 {
                    let _radial_vel = rel_pos.normalize().dot(rel_vel);
                    let _initial_r = rel_pos.length();

                    if !has_nodes_ahead
                        && config.target_body.is_none()
                    {
                        return PredictionPhase::Done {
                            reason: TerminationReason::OrbitClosed,
                        };
                    }
                }
            }

            PredictionPhase::Orbiting {
                body_idx,
                start_angle,
                crossed_half,
            }
        }

        PredictionPhase::Transfer { origin_body } => {
            let ship_pos = state.positions[ship_idx];

            // Check each body's Hill sphere for encounter entry
            for body_idx in 0..state.body_count() {
                if body_idx == ship_idx {
                    continue;
                }
                let dist = (ship_pos - state.positions[body_idx]).length();

                if body_idx < config.body_hill_radii.len()
                    && config.body_hill_radii[body_idx] > 0.0
                    && dist < config.body_hill_radii[body_idx]
                    && body_idx != origin_body
                {
                    return PredictionPhase::Encounter {
                        body_idx,
                        entered_at: time,
                        closest_so_far: dist,
                        passed_closest: false,
                    };
                }

                // Check collision
                if body_idx < config.body_radii.len() && dist < config.body_radii[body_idx] {
                    return PredictionPhase::Done {
                        reason: TerminationReason::Collision { body_idx },
                    };
                }
            }

            // Check if we're escaping
            let rel_pos = ship_pos - state.positions[dom];
            let rel_vel = state.velocities[ship_idx] - state.velocities[dom];
            let mu = G * state.masses[dom];
            let specific_energy = rel_vel.length_squared() / 2.0 - mu / rel_pos.length();
            let radial_vel = rel_pos.normalize().dot(rel_vel);

            if specific_energy > 0.0 && radial_vel > 0.0 {
                return PredictionPhase::Escaping {
                    from_body: dom,
                    escape_distance: rel_pos.length(),
                };
            }

            // Check if captured by a new body
            if dom != origin_body {
                let rel_pos = ship_pos - state.positions[dom];
                let rel_vel = state.velocities[ship_idx] - state.velocities[dom];
                let mu = G * state.masses[dom];
                let specific_energy = rel_vel.length_squared() / 2.0 - mu / rel_pos.length();

                if specific_energy < 0.0 {
                    return PredictionPhase::Orbiting {
                        body_idx: dom,
                        start_angle: radial_angle(ship_pos, state.positions[dom]),
                        crossed_half: false,
                    };
                }
            }

            PredictionPhase::Transfer { origin_body }
        }

        PredictionPhase::Encounter {
            body_idx,
            entered_at,
            closest_so_far,
            passed_closest,
        } => {
            let ship_pos = state.positions[ship_idx];
            let body_pos = state.positions[body_idx];
            let dist = (ship_pos - body_pos).length();

            // Check collision
            if body_idx < config.body_radii.len() && dist < config.body_radii[body_idx] {
                return PredictionPhase::Done {
                    reason: TerminationReason::Collision { body_idx },
                };
            }

            // Update closest approach tracking
            let (new_closest, new_passed) = if dist < closest_so_far {
                (dist, false)
            } else if !passed_closest && dist > closest_so_far * 1.01 {
                (closest_so_far, true)
            } else {
                (closest_so_far, passed_closest)
            };

            // Check if exited Hill sphere
            if body_idx < config.body_hill_radii.len()
                && dist > config.body_hill_radii[body_idx]
            {
                let rel_pos = ship_pos - body_pos;
                let rel_vel = state.velocities[ship_idx] - state.velocities[body_idx];
                let mu = G * state.masses[body_idx];
                let specific_energy = rel_vel.length_squared() / 2.0 - mu / rel_pos.length();

                if specific_energy < 0.0 {
                    return PredictionPhase::Orbiting {
                        body_idx,
                        start_angle: radial_angle(ship_pos, body_pos),
                        crossed_half: false,
                    };
                }

                // Not captured by encounter body — check if bound to parent
                let dom = dominant_body(state, ship_idx, &config.body_hill_radii);
                let dom_rel_pos = ship_pos - state.positions[dom];
                let dom_rel_vel = state.velocities[ship_idx] - state.velocities[dom];
                let dom_mu = G * state.masses[dom];
                let dom_energy = dom_rel_vel.length_squared() / 2.0 - dom_mu / dom_rel_pos.length();

                if dom_energy < 0.0 {
                    return PredictionPhase::Orbiting {
                        body_idx: dom,
                        start_angle: radial_angle(ship_pos, state.positions[dom]),
                        crossed_half: false,
                    };
                }

                return PredictionPhase::Transfer {
                    origin_body: body_idx,
                };
            }

            // Check capture
            if new_passed {
                let rel_pos = ship_pos - body_pos;
                let rel_vel = state.velocities[ship_idx] - state.velocities[body_idx];
                let mu = G * state.masses[body_idx];
                let elements = orbital_elements(rel_pos, rel_vel, mu);

                if elements.eccentricity < 1.0 {
                    return PredictionPhase::Orbiting {
                        body_idx,
                        start_angle: radial_angle(ship_pos, body_pos),
                        crossed_half: false,
                    };
                }
            }

            PredictionPhase::Encounter {
                body_idx,
                entered_at,
                closest_so_far: new_closest,
                passed_closest: new_passed,
            }
        }

        PredictionPhase::Escaping {
            from_body,
            escape_distance,
        } => {
            let ship_pos = state.positions[ship_idx];
            let dist = (ship_pos - state.positions[from_body]).length();

            if dom != from_body {
                let rel_pos = ship_pos - state.positions[dom];
                let rel_vel = state.velocities[ship_idx] - state.velocities[dom];
                let mu = G * state.masses[dom];
                let specific_energy = rel_vel.length_squared() / 2.0 - mu / rel_pos.length();

                if specific_energy < 0.0 {
                    return PredictionPhase::Orbiting {
                        body_idx: dom,
                        start_angle: radial_angle(ship_pos, state.positions[dom]),
                        crossed_half: false,
                    };
                } else {
                    return PredictionPhase::Transfer {
                        origin_body: from_body,
                    };
                }
            }

            let hill_multiplier = 4.0;
            if from_body < config.body_hill_radii.len()
                && dist > config.body_hill_radii[from_body] * hill_multiplier
            {
                return PredictionPhase::Done {
                    reason: TerminationReason::EscapeComplete,
                };
            }

            if from_body >= config.body_hill_radii.len() && dist > escape_distance * 10.0 {
                return PredictionPhase::Done {
                    reason: TerminationReason::EscapeComplete,
                };
            }

            PredictionPhase::Escaping {
                from_body,
                escape_distance: escape_distance.max(dist),
            }
        }

        done @ PredictionPhase::Done { .. } => done,
    }
}

/// Wrap angle to [-π, π].
fn angle_wrap(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= std::f64::consts::TAU;
    }
    while angle < -std::f64::consts::PI {
        angle += std::f64::consts::TAU;
    }
    angle
}
