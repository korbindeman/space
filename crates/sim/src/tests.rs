#[cfg(test)]
mod tests {
    use glam::DVec3;
    use std::f64::consts::{PI, TAU};

    use crate::*;

    // --- Physical constants ---
    const EARTH_MASS: f64 = 5.972e24;
    const EARTH_RADIUS: f64 = 6.371e6;
    const SUN_MASS: f64 = 1.989e30;
    const MOON_MASS: f64 = 7.342e22;
    const AU: f64 = 1.496e11;
    const EARTH_MOON_DIST: f64 = 3.844e8;

    // --- Tolerance constants ---
    const ANGLE_TOLERANCE_RAD: f64 = 0.01;

    // --- Test helpers ---

    /// Two-body: Earth at origin, test mass in circular orbit at given altitude.
    fn circular_orbit_state(altitude_km: f64) -> (SimState, f64) {
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

    /// Two-body: Earth at origin, test mass in elliptical orbit.
    fn elliptical_orbit_state(periapsis_km: f64, apoapsis_km: f64) -> (SimState, f64) {
        let rp = EARTH_RADIUS + periapsis_km * 1000.0;
        let ra = EARTH_RADIUS + apoapsis_km * 1000.0;
        let mu = G * EARTH_MASS;
        let a = (rp + ra) / 2.0;
        let v_periapsis = (mu * (2.0 / rp - 1.0 / a)).sqrt();
        let period = TAU * (a.powi(3) / mu).sqrt();

        let state = SimState {
            positions: vec![DVec3::ZERO, DVec3::new(rp, 0.0, 0.0)],
            velocities: vec![DVec3::ZERO, DVec3::new(0.0, 0.0, v_periapsis)],
            masses: vec![EARTH_MASS, 1.0],
        };
        (state, period)
    }

    /// Three-body: Sun-Earth-Moon at known positions.
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

    fn specific_orbital_energy(state: &SimState, body_idx: usize, central_idx: usize) -> f64 {
        let r = (state.positions[body_idx] - state.positions[central_idx]).length();
        let v = (state.velocities[body_idx] - state.velocities[central_idx]).length();
        let mu = G * state.masses[central_idx];
        v * v / 2.0 - mu / r
    }

    fn specific_angular_momentum(state: &SimState, body_idx: usize, central_idx: usize) -> DVec3 {
        let r = state.positions[body_idx] - state.positions[central_idx];
        let v = state.velocities[body_idx] - state.velocities[central_idx];
        r.cross(v)
    }

    fn propagate(state: &mut SimState, integrator: &dyn Integrator, force: &dyn ForceModel, dt: f64, steps: usize) {
        for _ in 0..steps {
            integrator.step(state, &|s| force.compute_accelerations(s), dt);
        }
    }

    // ========================================================================
    // Integrator tests
    // ========================================================================

    fn test_circular_orbit_conservation(dt: f64, radius_tol: f64, energy_tol: f64) {
        let (mut state, period) = circular_orbit_state(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();

        let initial_radius = (state.positions[1] - state.positions[0]).length();
        let initial_energy = specific_orbital_energy(&state, 1, 0);

        let steps = (period / dt).round() as usize;
        propagate(&mut state, &integrator, &force, dt, steps);

        let final_radius = (state.positions[1] - state.positions[0]).length();
        let final_energy = specific_orbital_energy(&state, 1, 0);

        let radius_error = (final_radius - initial_radius).abs();
        let energy_error = ((final_energy - initial_energy) / initial_energy).abs();

        assert!(
            radius_error < radius_tol,
            "Radius error {radius_error:.6} m exceeds tolerance {radius_tol} m (dt={dt}s)"
        );
        assert!(
            energy_error < energy_tol,
            "Energy relative error {energy_error:.2e} exceeds tolerance {energy_tol:.2e} (dt={dt}s)"
        );
    }

    #[test]
    fn test_circular_orbit_conservation_dt_1s() {
        test_circular_orbit_conservation(1.0, 0.001, 1e-12);
    }

    #[test]
    fn test_circular_orbit_conservation_dt_10s() {
        test_circular_orbit_conservation(10.0, 1.0, 1e-8);
    }

    #[test]
    fn test_circular_orbit_conservation_dt_100s() {
        test_circular_orbit_conservation(100.0, 1000.0, 1e-4);
    }

    #[test]
    fn test_circular_orbit_convergence_rate() {
        let (state1, period) = circular_orbit_state(400.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();

        let dt1 = 20.0;
        let dt2 = 10.0;
        let initial_energy = specific_orbital_energy(&state1, 1, 0);

        let total_time = (period / dt2).round() * dt2;

        let mut s1 = state1.clone();
        let steps1 = (total_time / dt1).round() as usize;
        propagate(&mut s1, &integrator, &force, dt1, steps1);
        let err1 = ((specific_orbital_energy(&s1, 1, 0) - initial_energy) / initial_energy).abs();

        let mut s2 = state1;
        let steps2 = (total_time / dt2).round() as usize;
        propagate(&mut s2, &integrator, &force, dt2, steps2);
        let err2 = ((specific_orbital_energy(&s2, 1, 0) - initial_energy) / initial_energy).abs();

        let ratio = err1 / err2;
        assert!(
            ratio > 4.0 && ratio < 64.0,
            "Convergence ratio {ratio:.1} not consistent with 4th-order (expected ~16, err1={err1:.2e}, err2={err2:.2e})"
        );
    }

    #[test]
    fn test_elliptical_orbit_energy_conservation() {
        let (mut state, period) = elliptical_orbit_state(200.0, 35_786.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();

        let initial_energy = specific_orbital_energy(&state, 1, 0);
        let dt = 10.0;
        let steps = (period / dt).round() as usize;
        propagate(&mut state, &integrator, &force, dt, steps);

        let final_energy = specific_orbital_energy(&state, 1, 0);
        let error = ((final_energy - initial_energy) / initial_energy).abs();

        assert!(error < 1e-6, "Energy drift {error:.2e} too large for elliptical orbit");
    }

    #[test]
    fn test_elliptical_orbit_angular_momentum_conservation() {
        let (mut state, period) = elliptical_orbit_state(200.0, 35_786.0);
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();

        let initial_h = specific_angular_momentum(&state, 1, 0);
        let dt = 10.0;
        let steps = (period / dt).round() as usize;
        propagate(&mut state, &integrator, &force, dt, steps);

        let final_h = specific_angular_momentum(&state, 1, 0);
        let error = (final_h - initial_h).length() / initial_h.length();

        assert!(error < 1e-8, "Angular momentum drift {error:.2e}");
    }

    #[test]
    fn test_hyperbolic_trajectory() {
        let r = EARTH_RADIUS + 400_000.0;
        let mu = G * EARTH_MASS;
        let v_escape = (2.0 * mu / r).sqrt();
        let v = v_escape * 1.5;

        let mut state = SimState {
            positions: vec![DVec3::ZERO, DVec3::new(r, 0.0, 0.0)],
            velocities: vec![DVec3::ZERO, DVec3::new(0.0, 0.0, v)],
            masses: vec![EARTH_MASS, 1.0],
        };

        let initial_energy = specific_orbital_energy(&state, 1, 0);
        assert!(initial_energy > 0.0, "Should be hyperbolic (positive energy)");

        let integrator = RK4Integrator;
        let force = NBodyGravity::new();
        propagate(&mut state, &integrator, &force, 10.0, 10_000);

        let final_dist = (state.positions[1] - state.positions[0]).length();
        assert!(final_dist > r * 10.0, "Should have moved far away: {final_dist}");

        let final_energy = specific_orbital_energy(&state, 1, 0);
        let error = ((final_energy - initial_energy) / initial_energy).abs();
        assert!(error < 1e-6, "Energy should be conserved: {error:.2e}");

        let v_inf_expected = (2.0 * initial_energy).sqrt();
        let v_actual = (state.velocities[1] - state.velocities[0]).length();
        let v_error = (v_actual - v_inf_expected).abs() / v_inf_expected;
        assert!(v_error < 0.01, "Asymptotic velocity error {v_error:.4}");
    }

    #[test]
    fn test_three_body_qualitative_behavior() {
        let mut state = sun_earth_moon_state();
        let integrator = RK4Integrator;
        let force = NBodyGravity::new();

        let lunar_month: f64 = 27.3 * 86400.0;
        let dt: f64 = 60.0;
        let steps = (lunar_month / dt).round() as usize;
        propagate(&mut state, &integrator, &force, dt, steps);

        let moon_rel = state.positions[2] - state.positions[1];
        let moon_dist = moon_rel.length();
        assert!(
            (moon_dist - EARTH_MOON_DIST).abs() / EARTH_MOON_DIST < 0.2,
            "Moon distance {:.0} km drifted too far from {:.0} km",
            moon_dist / 1000.0,
            EARTH_MOON_DIST / 1000.0
        );

        let earth_angle = state.positions[1].z.atan2(state.positions[1].x);
        let expected_angle = TAU / 12.0;
        assert!(
            (earth_angle - expected_angle).abs() < 0.1,
            "Earth angle {:.3} rad, expected ~{:.3} rad",
            earth_angle,
            expected_angle
        );
    }

    // ========================================================================
    // Force model tests
    // ========================================================================

    #[test]
    fn test_gravity_symmetry() {
        let state = SimState {
            positions: vec![DVec3::ZERO, DVec3::new(1e7, 0.0, 0.0)],
            velocities: vec![DVec3::ZERO, DVec3::ZERO],
            masses: vec![EARTH_MASS, MOON_MASS],
        };

        let force = NBodyGravity::new();
        let accels = force.compute_accelerations(&state);

        let f0 = accels[0] * state.masses[0];
        let f1 = accels[1] * state.masses[1];
        let diff = (f0 + f1).length();
        let scale = f0.length().max(f1.length());
        let relative_error = diff / scale;
        assert!(relative_error < 1e-10, "Third law violated: relative error {relative_error:.2e}");
    }

    #[test]
    fn test_gravity_analytical_leo() {
        let r = EARTH_RADIUS + 400_000.0;
        let state = SimState {
            positions: vec![DVec3::ZERO, DVec3::new(r, 0.0, 0.0)],
            velocities: vec![DVec3::ZERO, DVec3::ZERO],
            masses: vec![EARTH_MASS, 1.0],
        };

        let force = NBodyGravity::new();
        let accels = force.compute_accelerations(&state);

        let expected = G * EARTH_MASS / (r * r);
        let actual = accels[1].length();
        let error = (actual - expected).abs() / expected;
        assert!(error < 1e-10, "LEO gravity error: {error:.2e}");
        assert!(accels[1].x < 0.0, "Gravity should point toward Earth");
    }

    #[test]
    fn test_gravity_analytical_geo() {
        let r = EARTH_RADIUS + 35_786_000.0;
        let state = SimState {
            positions: vec![DVec3::ZERO, DVec3::new(r, 0.0, 0.0)],
            velocities: vec![DVec3::ZERO, DVec3::ZERO],
            masses: vec![EARTH_MASS, 1.0],
        };

        let force = NBodyGravity::new();
        let accels = force.compute_accelerations(&state);

        let expected = G * EARTH_MASS / (r * r);
        let actual = accels[1].length();
        let error = (actual - expected).abs() / expected;
        assert!(error < 1e-10, "GEO gravity error: {error:.2e}");
    }

    #[test]
    fn test_gravity_analytical_moon() {
        let state = SimState {
            positions: vec![DVec3::ZERO, DVec3::new(EARTH_MOON_DIST, 0.0, 0.0)],
            velocities: vec![DVec3::ZERO, DVec3::ZERO],
            masses: vec![EARTH_MASS, 1.0],
        };

        let force = NBodyGravity::new();
        let accels = force.compute_accelerations(&state);

        let expected = G * EARTH_MASS / (EARTH_MOON_DIST * EARTH_MOON_DIST);
        let actual = accels[1].length();
        let error = (actual - expected).abs() / expected;
        assert!(error < 1e-10, "Moon distance gravity error: {error:.2e}");
    }

    #[test]
    fn test_gravity_superposition() {
        let state = SimState {
            positions: vec![
                DVec3::ZERO,
                DVec3::new(1e7, 0.0, 0.0),
                DVec3::new(2e7, 0.0, 0.0),
            ],
            velocities: vec![DVec3::ZERO; 3],
            masses: vec![EARTH_MASS, EARTH_MASS, EARTH_MASS],
        };

        let force = NBodyGravity::new();
        let accels = force.compute_accelerations(&state);

        assert!(
            accels[1].length() < 1e-10,
            "Middle body should have near-zero net acceleration: {:?}",
            accels[1]
        );
    }

    #[test]
    fn test_gravity_exclusion_mask() {
        let state = SimState {
            positions: vec![
                DVec3::ZERO,
                DVec3::new(1e7, 0.0, 0.0),
                DVec3::new(0.0, 1e7, 0.0),
            ],
            velocities: vec![DVec3::ZERO; 3],
            masses: vec![EARTH_MASS, EARTH_MASS, 1.0],
        };

        let force_full = NBodyGravity::new();
        let force_excluded = NBodyGravity::with_exclusions(vec![2]);

        let accels_full = force_full.compute_accelerations(&state);
        let accels_excl = force_excluded.compute_accelerations(&state);

        let diff_body2 = (accels_full[2] - accels_excl[2]).length();
        assert!(diff_body2 < 1e-20, "Excluded body should still receive gravity");

        let expected_0 = G * EARTH_MASS / 1e14;
        let actual_0 = accels_excl[0].length();
        let error = (actual_0 - expected_0).abs() / expected_0;
        assert!(error < 1e-10, "Exclusion changed non-excluded gravity");
    }

    #[test]
    fn test_gravity_precision_at_1au() {
        let earth_pos = DVec3::new(AU, 0.0, 0.0);
        let ship_pos = earth_pos + DVec3::new(0.0, EARTH_RADIUS + 400_000.0, 0.0);

        let state = SimState {
            positions: vec![DVec3::ZERO, earth_pos, ship_pos],
            velocities: vec![DVec3::ZERO; 3],
            masses: vec![SUN_MASS, EARTH_MASS, 1.0],
        };

        let force = NBodyGravity::new();
        let accels = force.compute_accelerations(&state);

        let r_ship_earth = (ship_pos - earth_pos).length();
        let expected_earth_accel = G * EARTH_MASS / (r_ship_earth * r_ship_earth);

        let to_earth = (earth_pos - ship_pos).normalize();
        let earth_component = accels[2].dot(to_earth);

        let error = (earth_component - expected_earth_accel).abs() / expected_earth_accel;
        assert!(
            error < 1e-6,
            "Precision loss at 1 AU: earth gravity component error {error:.2e}"
        );
    }

    // ========================================================================
    // Burn model tests
    // ========================================================================

    #[test]
    fn test_impulse_before_window() {
        let burn = ImpulseBurn {
            time: 100.0,
            delta_v: DVec3::new(100.0, 0.0, 0.0),
        };
        let (state, _) = circular_orbit_state(400.0);
        let result = burn.acceleration(&state, 1, 0, 50.0, 10.0);
        assert!(result.is_none(), "Should return None before burn time");
    }

    #[test]
    fn test_impulse_during_window() {
        let burn = ImpulseBurn {
            time: 100.0,
            delta_v: DVec3::new(100.0, 0.0, 0.0),
        };
        let (state, _) = circular_orbit_state(400.0);
        let dt = 10.0;
        let result = burn.acceleration(&state, 1, 0, 95.0, dt);
        assert!(result.is_some(), "Should return acceleration during burn window");
        let accel = result.unwrap();
        assert!(
            (accel.length() - 100.0 / dt).abs() < 1.0,
            "Acceleration magnitude {:.2} should be ~{:.2}",
            accel.length(),
            100.0 / dt
        );
    }

    #[test]
    fn test_impulse_after_window() {
        let burn = ImpulseBurn {
            time: 100.0,
            delta_v: DVec3::new(100.0, 0.0, 0.0),
        };
        let (state, _) = circular_orbit_state(400.0);
        let result = burn.acceleration(&state, 1, 0, 110.0, 10.0);
        assert!(result.is_none(), "Should return None after burn time");
    }

    #[test]
    fn test_impulse_exact_boundary() {
        let burn = ImpulseBurn {
            time: 100.0,
            delta_v: DVec3::new(100.0, 0.0, 0.0),
        };
        let (state, _) = circular_orbit_state(400.0);
        let result = burn.acceleration(&state, 1, 0, 100.0, 10.0);
        assert!(result.is_some(), "Should fire when impulse time = substep start");
        let result = burn.acceleration(&state, 1, 0, 90.0, 10.0);
        assert!(result.is_some(), "Should fire when impulse time = substep end");
    }

    // ========================================================================
    // Orbital frame tests
    // ========================================================================

    #[test]
    fn test_orbital_frame_orthonormality() {
        let (state, _) = circular_orbit_state(400.0);
        let frame = orbital_frame(state.positions[1], state.velocities[1], state.positions[0]);

        let prograde = frame.col(0);
        let normal = frame.col(1);
        let radial = frame.col(2);

        assert!((prograde.length() - 1.0).abs() < 1e-10, "Prograde not unit");
        assert!((normal.length() - 1.0).abs() < 1e-10, "Normal not unit");
        assert!((radial.length() - 1.0).abs() < 1e-10, "Radial not unit");
        assert!(prograde.dot(normal).abs() < 1e-10, "Prograde·Normal != 0");
        assert!(prograde.dot(radial).abs() < 1e-10, "Prograde·Radial != 0");
        assert!(normal.dot(radial).abs() < 1e-10, "Normal·Radial != 0");
    }

    #[test]
    fn test_orbital_frame_prograde_parallel() {
        let (state, _) = circular_orbit_state(400.0);
        let frame = orbital_frame(state.positions[1], state.velocities[1], state.positions[0]);

        let prograde = frame.col(0);
        let vel_dir = state.velocities[1].normalize();
        let dot = prograde.dot(vel_dir);
        assert!(
            (dot - 1.0).abs() < 1e-10,
            "Prograde should be parallel to velocity, dot={dot}"
        );
    }

    #[test]
    fn test_orbital_frame_at_multiple_angles() {
        let mu = G * EARTH_MASS;
        let r = EARTH_RADIUS + 400_000.0;
        let v = (mu / r).sqrt();

        for angle_deg in [0.0_f64, 90.0, 180.0, 270.0] {
            let angle = angle_deg.to_radians();
            let pos = DVec3::new(r * angle.cos(), 0.0, r * angle.sin());
            let vel = DVec3::new(-v * angle.sin(), 0.0, v * angle.cos());

            let frame = orbital_frame(pos, vel, DVec3::ZERO);
            let prograde = frame.col(0);
            let vel_dir = vel.normalize();
            let dot = prograde.dot(vel_dir);
            assert!(
                (dot - 1.0).abs() < 1e-10,
                "Prograde not along velocity at {angle_deg}°: dot={dot}"
            );
        }
    }

    #[test]
    fn test_orbital_frame_eccentric_orbit() {
        let (state, _) = elliptical_orbit_state(200.0, 35_786.0);
        let frame = orbital_frame(state.positions[1], state.velocities[1], state.positions[0]);

        let prograde = frame.col(0);
        assert!(
            (prograde.dot(DVec3::Z) - 1.0).abs() < 1e-10,
            "Prograde at periapsis should be +Z"
        );
    }

    // ========================================================================
    // End-to-end burn tests
    // ========================================================================

    #[test]
    fn test_prograde_impulse_raises_apoapsis() {
        let (mut state, _) = circular_orbit_state(400.0);
        let mu = G * EARTH_MASS;
        let r0 = (state.positions[1] - state.positions[0]).length();

        let frame = orbital_frame(state.positions[1], state.velocities[1], state.positions[0]);
        let dv_world = frame * DVec3::new(100.0, 0.0, 0.0);
        state.velocities[1] += dv_world;

        let rel_pos = state.positions[1] - state.positions[0];
        let rel_vel = state.velocities[1] - state.velocities[0];
        let elements = orbital_elements(rel_pos, rel_vel, mu);

        assert!(
            elements.apoapsis > r0 * 1.01,
            "Apoapsis {:.0} should be higher than circular radius {:.0}",
            elements.apoapsis,
            r0
        );
        assert!(
            (elements.periapsis - r0).abs() / r0 < 0.01,
            "Periapsis should stay near original radius"
        );
    }

    #[test]
    fn test_retrograde_impulse_lowers_apoapsis() {
        let (mut state, _) = circular_orbit_state(400.0);
        let mu = G * EARTH_MASS;
        let r0 = (state.positions[1] - state.positions[0]).length();

        let frame = orbital_frame(state.positions[1], state.velocities[1], state.positions[0]);
        let dv_world = frame * DVec3::new(-100.0, 0.0, 0.0);
        state.velocities[1] += dv_world;

        let rel_pos = state.positions[1] - state.positions[0];
        let rel_vel = state.velocities[1] - state.velocities[0];
        let elements = orbital_elements(rel_pos, rel_vel, mu);

        assert!(
            elements.periapsis < r0 * 0.99,
            "Periapsis {:.0} should be lower after retrograde burn",
            elements.periapsis
        );
    }

    #[test]
    fn test_normal_impulse_changes_inclination() {
        let (mut state, _) = circular_orbit_state(400.0);
        let mu = G * EARTH_MASS;

        let frame = orbital_frame(state.positions[1], state.velocities[1], state.positions[0]);
        let dv_world = frame * DVec3::new(0.0, 200.0, 0.0);
        state.velocities[1] += dv_world;

        let rel_pos = state.positions[1] - state.positions[0];
        let rel_vel = state.velocities[1] - state.velocities[0];
        let elements = orbital_elements(rel_pos, rel_vel, mu);

        assert!(
            elements.inclination > 0.01,
            "Inclination {:.4} rad should have changed after normal burn",
            elements.inclination
        );
    }

    #[test]
    fn test_radial_impulse_rotates_apse_line() {
        let (mut state, _) = circular_orbit_state(400.0);
        let mu = G * EARTH_MASS;

        let frame = orbital_frame(state.positions[1], state.velocities[1], state.positions[0]);
        let dv_world = frame * DVec3::new(0.0, 0.0, 100.0);
        state.velocities[1] += dv_world;

        let rel_pos = state.positions[1] - state.positions[0];
        let rel_vel = state.velocities[1] - state.velocities[0];
        let elements = orbital_elements(rel_pos, rel_vel, mu);

        assert!(
            elements.eccentricity > 0.001,
            "Eccentricity {:.6} should be non-zero after radial burn",
            elements.eccentricity
        );
        assert!(
            elements.true_anomaly > 0.1 && elements.true_anomaly < PI - 0.1
                || elements.true_anomaly > PI + 0.1 && elements.true_anomaly < TAU - 0.1,
            "True anomaly {:.3} should indicate rotated apse line",
            elements.true_anomaly
        );
    }

    // ========================================================================
    // Orbital math tests
    // ========================================================================

    #[test]
    fn test_keplerian_elements_circular() {
        let r = EARTH_RADIUS + 400_000.0;
        let mu = G * EARTH_MASS;
        let v = (mu / r).sqrt();

        let pos = DVec3::new(r, 0.0, 0.0);
        let vel = DVec3::new(0.0, 0.0, v);

        let elements = orbital_elements(pos, vel, mu);

        assert!((elements.semi_major_axis - r).abs() / r < 1e-10, "SMA wrong");
        assert!(elements.eccentricity < 1e-10, "Should be circular: e={}", elements.eccentricity);
        assert!((elements.period - TAU * (r.powi(3) / mu).sqrt()).abs() < 1.0, "Period wrong");
    }

    #[test]
    fn test_keplerian_elements_eccentric() {
        let rp = EARTH_RADIUS + 200_000.0;
        let ra = EARTH_RADIUS + 35_786_000.0;
        let a = (rp + ra) / 2.0;
        let e_expected = (ra - rp) / (ra + rp);
        let mu = G * EARTH_MASS;
        let v = (mu * (2.0 / rp - 1.0 / a)).sqrt();

        let pos = DVec3::new(rp, 0.0, 0.0);
        let vel = DVec3::new(0.0, 0.0, v);

        let elements = orbital_elements(pos, vel, mu);

        assert!(
            (elements.semi_major_axis - a).abs() / a < 1e-8,
            "SMA {:.0} vs expected {:.0}", elements.semi_major_axis, a
        );
        assert!(
            (elements.eccentricity - e_expected).abs() < 1e-8,
            "Eccentricity {:.8} vs expected {:.8}", elements.eccentricity, e_expected
        );
    }

    #[test]
    fn test_keplerian_elements_polar() {
        let r = EARTH_RADIUS + 400_000.0;
        let mu = G * EARTH_MASS;
        let v = (mu / r).sqrt();

        let pos = DVec3::new(r, 0.0, 0.0);
        let vel = DVec3::new(0.0, 0.0, v);

        let elements = orbital_elements(pos, vel, mu);

        assert!(
            (elements.inclination - PI / 2.0).abs() < ANGLE_TOLERANCE_RAD,
            "Inclination {:.4} should be ~π/2 for polar orbit",
            elements.inclination
        );
    }

    #[test]
    fn test_keplerian_elements_retrograde() {
        let r = EARTH_RADIUS + 400_000.0;
        let mu = G * EARTH_MASS;
        let v = (mu / r).sqrt();

        let pos = DVec3::new(r, 0.0, 0.0);
        let vel2 = DVec3::new(0.0, -v, 0.0);
        let elements2 = orbital_elements(pos, vel2, mu);

        assert!(
            elements2.inclination > PI / 2.0,
            "Retrograde orbit should have inclination > 90°: {:.4}",
            elements2.inclination
        );
    }

    #[test]
    fn test_keplerian_elements_degenerate_circular_equatorial() {
        let r = EARTH_RADIUS + 400_000.0;
        let mu = G * EARTH_MASS;
        let v = (mu / r).sqrt();

        let pos = DVec3::new(r, 0.0, 0.0);
        let vel = DVec3::new(0.0, v, 0.0);

        let elements = orbital_elements(pos, vel, mu);

        assert!(!elements.semi_major_axis.is_nan(), "SMA is NaN");
        assert!(!elements.eccentricity.is_nan(), "Eccentricity is NaN");
        assert!(!elements.inclination.is_nan(), "Inclination is NaN");
        assert!(!elements.longitude_of_ascending_node.is_nan(), "RAAN is NaN");
        assert!(!elements.argument_of_periapsis.is_nan(), "AoP is NaN");
        assert!(!elements.true_anomaly.is_nan(), "True anomaly is NaN");
        assert!(!elements.period.is_nan(), "Period is NaN");
    }

    #[test]
    fn test_keplerian_elements_degenerate_exactly_circular() {
        let r = EARTH_RADIUS + 400_000.0;
        let mu = G * EARTH_MASS;
        let v = (mu / r).sqrt();

        let pos = DVec3::new(r, 0.0, 0.0);
        let vel = DVec3::new(0.0, 0.0, v);

        let elements = orbital_elements(pos, vel, mu);

        assert!(elements.eccentricity < 1e-10);
        assert!(!elements.argument_of_periapsis.is_nan());
        assert!(!elements.true_anomaly.is_nan());
    }

    #[test]
    fn test_orbital_period_iss() {
        let r = EARTH_RADIUS + 408_000.0;
        let mu = G * EARTH_MASS;
        let v = (mu / r).sqrt();

        let elements = orbital_elements(
            DVec3::new(r, 0.0, 0.0),
            DVec3::new(0.0, 0.0, v),
            mu,
        );

        let expected_period_min = 92.0 * 60.0;
        let error = (elements.period - expected_period_min).abs() / expected_period_min;
        assert!(
            error < 0.02,
            "ISS period {:.1} min, expected ~92 min",
            elements.period / 60.0
        );
    }

    #[test]
    fn test_orbital_period_moon() {
        let mu = G * EARTH_MASS;
        let v = (mu / EARTH_MOON_DIST).sqrt();

        let elements = orbital_elements(
            DVec3::new(EARTH_MOON_DIST, 0.0, 0.0),
            DVec3::new(0.0, 0.0, v),
            mu,
        );

        let expected_days = 27.3;
        let actual_days = elements.period / 86400.0;
        let error = (actual_days - expected_days).abs() / expected_days;
        assert!(error < 0.02, "Moon period {actual_days:.1} days, expected ~{expected_days}");
    }

    #[test]
    fn test_orbital_period_earth() {
        let mu = G * SUN_MASS;
        let v = (mu / AU).sqrt();

        let elements = orbital_elements(
            DVec3::new(AU, 0.0, 0.0),
            DVec3::new(0.0, 0.0, v),
            mu,
        );

        let expected_days = 365.25;
        let actual_days = elements.period / 86400.0;
        let error = (actual_days - expected_days).abs() / expected_days;
        assert!(error < 0.01, "Earth period {actual_days:.1} days, expected ~{expected_days}");
    }

    #[test]
    fn test_hill_radius_earth() {
        let r_hill = hill_radius(AU, EARTH_MASS, SUN_MASS);
        let expected_km = 1_500_000.0;
        let actual_km = r_hill / 1000.0;
        let error = (actual_km - expected_km).abs() / expected_km;
        assert!(
            error < 0.05,
            "Earth Hill radius {actual_km:.0} km, expected ~{expected_km:.0} km"
        );
    }

    #[test]
    fn test_hill_radius_moon() {
        let r_hill = hill_radius(EARTH_MOON_DIST, MOON_MASS, EARTH_MASS);
        let expected_km = 60_000.0;
        let actual_km = r_hill / 1000.0;
        let error = (actual_km - expected_km).abs() / expected_km;
        assert!(
            error < 0.1,
            "Moon Hill radius {actual_km:.0} km, expected ~{expected_km:.0} km"
        );
    }
}
