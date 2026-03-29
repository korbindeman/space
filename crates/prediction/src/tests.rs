#[cfg(test)]
mod tests {
    use glam::DVec3;
    use std::f64::consts::TAU;

    use space_sim::*;
    use crate::*;

    // Physical constants
    const EARTH_MASS: f64 = 5.972e24;
    const EARTH_RADIUS: f64 = 6.371e6;
    const SUN_MASS: f64 = 1.989e30;
    const MOON_MASS: f64 = 7.342e22;
    const AU: f64 = 1.496e11;
    const EARTH_MOON_DIST: f64 = 3.844e8;
    const MOON_RADIUS: f64 = 1.737e6;
    const SUN_RADIUS: f64 = 6.957e8;

    fn sun_earth_moon_state() -> SimState {
        let earth_v = (G * SUN_MASS / AU).sqrt();
        let moon_v = (G * EARTH_MASS / EARTH_MOON_DIST).sqrt();
        SimState {
            positions: vec![
                DVec3::ZERO,
                DVec3::new(AU, 0.0, 0.0),
                DVec3::new(AU + EARTH_MOON_DIST, 0.0, 0.0),
            ],
            velocities: vec![
                DVec3::ZERO,
                DVec3::new(0.0, 0.0, earth_v),
                DVec3::new(0.0, 0.0, earth_v + moon_v),
            ],
            masses: vec![SUN_MASS, EARTH_MASS, MOON_MASS],
        }
    }

    fn leo_scenario() -> (SimState, usize) {
        let mut state = sun_earth_moon_state();
        let earth_pos = state.positions[1];
        let earth_vel = state.velocities[1];
        let r = EARTH_RADIUS + 400_000.0;
        let v = (G * EARTH_MASS / r).sqrt();
        state.positions.push(earth_pos + DVec3::new(r, 0.0, 0.0));
        state.velocities.push(earth_vel + DVec3::new(0.0, 0.0, v));
        state.masses.push(1000.0);
        (state, 3)
    }

    fn leo_config() -> PredictionConfig {
        let earth_hill = hill_radius(AU, EARTH_MASS, SUN_MASS);
        let moon_hill = hill_radius(EARTH_MOON_DIST, MOON_MASS, EARTH_MASS);
        PredictionConfig {
            max_steps: 10_000,
            base_dt: 60.0,
            adaptive_dt: true,
            target_body: None,
            body_radii: vec![SUN_RADIUS, EARTH_RADIUS, MOON_RADIUS],
            body_hill_radii: vec![0.0, earth_hill, moon_hill],
            body_parent: vec![None, Some(0), Some(1)],
            extend_count: 0,
        }
    }

    fn circular_orbit_state_2body(altitude_km: f64) -> (SimState, f64) {
        let r = EARTH_RADIUS + altitude_km * 1000.0;
        let mu = G * EARTH_MASS;
        let v = (mu / r).sqrt();
        let period = TAU * (r.powi(3) / mu).sqrt();
        let state = SimState {
            positions: vec![DVec3::ZERO, DVec3::new(r, 0.0, 0.0)],
            velocities: vec![DVec3::ZERO, DVec3::new(0.0, 0.0, v)],
            masses: vec![EARTH_MASS, 1.0],
        };
        (state, period)
    }

    fn simple_config_2body() -> PredictionConfig {
        PredictionConfig {
            max_steps: 10_000,
            base_dt: 10.0,
            adaptive_dt: false,
            target_body: None,
            body_radii: vec![EARTH_RADIUS],
            body_hill_radii: vec![],
            body_parent: vec![],
            extend_count: 0,
        }
    }

    // ========================================================================
    // Trail generation tests
    // ========================================================================

    #[test]
    fn test_closed_orbit_trail() {
        let (state, period) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let config = simple_config_2body();

        let result = predict(&state, 1, &[], &force, &integrator, &config, 0.0);

        assert!(!result.segments.is_empty());
        let seg = &result.segments[0];
        assert!(seg.points.len() > 10, "Trail should have many points");

        let first = seg.points[0];
        let last = seg.points.last().unwrap();
        let closure_error = (*last - first).length();
        assert!(
            closure_error < 1_000_000.0,
            "Trail should close: error {closure_error:.0} m"
        );

        let trail_time = *seg.times.last().unwrap() - seg.times[0];
        let time_error = (trail_time - period).abs() / period;
        assert!(
            time_error < 0.2,
            "Trail time {trail_time:.0}s should be ~{period:.0}s (error {time_error:.2})"
        );

        assert!(
            matches!(result.termination, TerminationReason::OrbitClosed),
            "Should terminate with OrbitClosed, got {:?}",
            result.termination
        );
    }

    #[test]
    fn test_trail_segment_count_with_node() {
        let (state, period) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let config = simple_config_2body();

        let node = ManeuverNode {
            id: NodeId(1),
            burn: Box::new(ImpulseBurn {
                time: period / 2.0,
                delta_v: DVec3::new(10.0, 0.0, 0.0),
            }),
        };

        let result = predict(&state, 1, &[node], &force, &integrator, &config, 0.0);

        assert!(
            result.segments.len() >= 2,
            "Should have at least 2 segments after a node, got {}",
            result.segments.len()
        );

        assert_eq!(result.segments[1].after_node, Some(NodeId(1)));

        let last_of_first = *result.segments[0].points.last().unwrap();
        let first_of_second = result.segments[1].points[0];
        let gap = (first_of_second - last_of_first).length();
        assert!(
            gap < 100_000.0,
            "Segment boundary gap {gap:.0} m should be small"
        );
    }

    #[test]
    fn test_trail_body_positions_stored() {
        let (state, _) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let mut config = simple_config_2body();
        config.max_steps = 100;

        let result = predict(&state, 1, &[], &force, &integrator, &config, 0.0);

        let seg = &result.segments[0];
        assert_eq!(seg.body_positions.len(), seg.points.len());

        for bp in &seg.body_positions {
            assert_eq!(bp.len(), 2, "Should store positions for all bodies");
        }

        let earth_drift = seg.body_positions.last().unwrap()[0].length();
        assert!(earth_drift < 1.0, "Earth should barely move: {earth_drift}");
    }

    #[test]
    fn test_dominant_body_transitions() {
        let (mut state, ship_idx) = leo_scenario();
        let integrator = RK4Integrator;
        let force = NBodyGravity::with_exclusions(vec![ship_idx]);

        let earth_pos = state.positions[1];
        let ship_pos = state.positions[ship_idx];
        let frame = orbital_frame(ship_pos, state.velocities[ship_idx], earth_pos);
        state.velocities[ship_idx] += frame * DVec3::new(3200.0, 0.0, 0.0);

        let config = leo_config();
        let result = predict(&state, ship_idx, &[], &force, &integrator, &config, 0.0);

        let mut saw_earth = false;
        for seg in &result.segments {
            for &dom in &seg.dominant_body {
                if dom == 1 {
                    saw_earth = true;
                }
            }
        }
        assert!(saw_earth, "Should start with Earth as dominant");
        assert!(result.segments[0].points.len() > 10);
    }

    // ========================================================================
    // Termination condition tests
    // ========================================================================

    #[test]
    fn test_collision_terminates() {
        let (state, _) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let mut config = simple_config_2body();
        config.adaptive_dt = false;
        config.base_dt = 10.0;

        let node = ManeuverNode {
            id: NodeId(1),
            burn: Box::new(ImpulseBurn {
                time: 5.0,
                delta_v: DVec3::new(-7000.0, 0.0, 0.0),
            }),
        };

        let result = predict(&state, 1, &[node], &force, &integrator, &config, 0.0);

        assert!(
            matches!(result.termination, TerminationReason::Collision { body_idx: 0 }),
            "Should terminate with collision, got {:?}",
            result.termination
        );
    }

    #[test]
    fn test_escape_terminates() {
        let (mut state, _) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let mut config = simple_config_2body();
        config.base_dt = 60.0;
        config.adaptive_dt = false;
        config.body_hill_radii = vec![1e9];

        let v_escape = (2.0 * G * EARTH_MASS / (EARTH_RADIUS + 400_000.0)).sqrt();
        let frame = orbital_frame(state.positions[1], state.velocities[1], state.positions[0]);
        state.velocities[1] += frame * DVec3::new(v_escape * 0.5, 0.0, 0.0);

        let result = predict(&state, 1, &[], &force, &integrator, &config, 0.0);

        assert!(
            matches!(
                result.termination,
                TerminationReason::EscapeComplete | TerminationReason::BudgetExhausted
            ),
            "Should escape or exhaust budget, got {:?}",
            result.termination
        );
    }

    #[test]
    fn test_budget_exhaustion() {
        let (state, _) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let mut config = simple_config_2body();
        config.max_steps = 50;
        config.adaptive_dt = false;
        config.base_dt = 1.0;

        let result = predict(&state, 1, &[], &force, &integrator, &config, 0.0);

        let total_points: usize = result.segments.iter().map(|s| s.points.len()).sum();
        assert_eq!(total_points, 51, "Should have max_steps + 1 points (initial + steps)");
        assert!(
            matches!(result.termination, TerminationReason::BudgetExhausted),
            "Should exhaust budget, got {:?}",
            result.termination
        );
    }

    // ========================================================================
    // Phase state machine tests
    // ========================================================================

    #[test]
    fn test_phase_initial_orbiting() {
        let (state, ship_idx) = leo_scenario();
        let config = leo_config();
        let phase = phase::determine_initial_phase(&state, ship_idx, &config);

        match phase {
            PredictionPhase::Orbiting { body_idx, .. } => {
                assert_eq!(body_idx, 1, "Ship in LEO should orbit Earth (index 1)");
            }
            other => panic!("Expected Orbiting, got {:?}", other),
        }
    }

    #[test]
    fn test_phase_orbit_closure() {
        let (state, _) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let config = simple_config_2body();

        let result = predict(&state, 1, &[], &force, &integrator, &config, 0.0);

        assert!(
            matches!(result.termination, TerminationReason::OrbitClosed),
            "Circular orbit should close, got {:?}",
            result.termination
        );
    }

    #[test]
    fn test_phase_extend_count() {
        let (state, _) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let mut config = simple_config_2body();
        config.extend_count = 1;

        let result = predict(&state, 1, &[], &force, &integrator, &config, 0.0);

        let total_time: f64 = result.segments.iter()
            .flat_map(|s| s.times.last().copied())
            .last()
            .unwrap_or(0.0);

        let period = circular_orbit_state_2body(400.0).1;
        assert!(
            total_time > period * 1.5,
            "With extend, trail time {total_time:.0}s should be > 1.5 periods ({:.0}s)",
            period * 1.5
        );
    }

    // ========================================================================
    // Adaptive step size tests
    // ========================================================================

    #[test]
    fn test_adaptive_dt_budget_respected() {
        let (state, _) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let mut config = simple_config_2body();
        config.max_steps = 200;
        config.adaptive_dt = true;
        config.base_dt = 1.0;

        let result = predict(&state, 1, &[], &force, &integrator, &config, 0.0);

        let total_points: usize = result.segments.iter().map(|s| s.points.len()).sum();
        assert!(
            total_points <= 201,
            "Total points {total_points} should not exceed max_steps + 1"
        );
    }

    // ========================================================================
    // Encounter detection tests (basic)
    // ========================================================================

    #[test]
    fn test_no_false_encounter_leo() {
        let (state, ship_idx) = leo_scenario();
        let integrator = RK4Integrator;
        let force = NBodyGravity::with_exclusions(vec![ship_idx]);
        let mut config = leo_config();
        config.max_steps = 2000;
        config.base_dt = 10.0;

        let result = predict(&state, ship_idx, &[], &force, &integrator, &config, 0.0);

        let moon_encounters: Vec<_> = result.encounters.iter()
            .filter(|e| e.body_idx == 2)
            .collect();
        assert!(
            moon_encounters.is_empty(),
            "Stable LEO should not produce Moon encounter"
        );
    }

    #[test]
    fn test_closest_approach_found() {
        let (state, _) = circular_orbit_state_2body(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let mut config = simple_config_2body();
        config.max_steps = 1000;

        let result = predict(&state, 1, &[], &force, &integrator, &config, 0.0);

        let earth_approach = result.approaches.iter().find(|a| a.body_idx == 0);
        assert!(earth_approach.is_some(), "Should find closest approach to Earth");

        let approach = earth_approach.unwrap();
        let expected_dist = EARTH_RADIUS + 400_000.0;
        let error = (approach.distance - expected_dist).abs() / expected_dist;
        assert!(
            error < 0.01,
            "Closest approach {:.0} km should be near orbital altitude {:.0} km",
            approach.distance / 1000.0,
            expected_dist / 1000.0
        );
    }

    // ========================================================================
    // Hohmann transfer test (integration)
    // ========================================================================

    #[test]
    fn test_hohmann_transfer_prediction() {
        let r1 = EARTH_RADIUS + 400_000.0;
        let r2 = EARTH_RADIUS + 35_786_000.0;
        let mu = G * EARTH_MASS;

        let v1 = (mu / r1).sqrt();
        let v_transfer_periapsis = (mu * (2.0 / r1 - 2.0 / (r1 + r2))).sqrt();
        let dv1 = v_transfer_periapsis - v1;

        let v_transfer_apoapsis = (mu * (2.0 / r2 - 2.0 / (r1 + r2))).sqrt();
        let v2 = (mu / r2).sqrt();
        let dv2 = v2 - v_transfer_apoapsis;

        let transfer_period = std::f64::consts::PI * ((r1 + r2) / 2.0).powf(1.5) / mu.sqrt();

        let state = SimState {
            positions: vec![DVec3::ZERO, DVec3::new(r1, 0.0, 0.0)],
            velocities: vec![DVec3::ZERO, DVec3::new(0.0, 0.0, v1)],
            masses: vec![EARTH_MASS, 1.0],
        };

        let node1 = ManeuverNode {
            id: NodeId(1),
            burn: Box::new(ImpulseBurn {
                time: 100.0,
                delta_v: DVec3::new(dv1, 0.0, 0.0),
            }),
        };
        let node2 = ManeuverNode {
            id: NodeId(2),
            burn: Box::new(ImpulseBurn {
                time: 100.0 + transfer_period,
                delta_v: DVec3::new(dv2, 0.0, 0.0),
            }),
        };

        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        let mut config = simple_config_2body();
        config.max_steps = 20_000;
        config.base_dt = 30.0;
        config.adaptive_dt = true;

        let result = predict(
            &state,
            1,
            &[node1, node2],
            &force,
            &integrator,
            &config,
            0.0,
        );

        assert!(
            result.segments.len() >= 3,
            "Hohmann should produce at least 3 segments, got {}",
            result.segments.len()
        );

        let last_seg = result.segments.last().unwrap();
        if last_seg.points.len() > 10 {
            let radii: Vec<f64> = last_seg
                .points
                .iter()
                .zip(last_seg.body_positions.iter())
                .map(|(p, bp)| (*p - bp[0]).length())
                .collect();

            let mean_radius = radii.iter().sum::<f64>() / radii.len() as f64;
            let radius_error = (mean_radius - r2).abs() / r2;
            assert!(
                radius_error < 0.05,
                "Final orbit radius {:.0} km should be near GEO {:.0} km (error {radius_error:.3})",
                mean_radius / 1000.0,
                r2 / 1000.0
            );
        }
    }
}
