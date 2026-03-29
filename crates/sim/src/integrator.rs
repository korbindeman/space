use glam::DVec3;

use super::SimState;

pub trait Integrator: Send + Sync {
    fn step(
        &self,
        state: &mut SimState,
        accel_fn: &dyn Fn(&SimState) -> Vec<DVec3>,
        dt: f64,
    );
}

/// Classical 4th-order Runge-Kutta integrator.
pub struct RK4Integrator;

impl Integrator for RK4Integrator {
    fn step(
        &self,
        state: &mut SimState,
        accel_fn: &dyn Fn(&SimState) -> Vec<DVec3>,
        dt: f64,
    ) {
        let n = state.body_count();

        // k1: accelerations at current state
        let k1_a = accel_fn(state);
        let k1_v: Vec<DVec3> = state.velocities.clone();

        // k2: state at t + dt/2 using k1
        let mut s2 = state.clone();
        for i in 0..n {
            s2.positions[i] = state.positions[i] + k1_v[i] * (dt / 2.0);
            s2.velocities[i] = state.velocities[i] + k1_a[i] * (dt / 2.0);
        }
        let k2_a = accel_fn(&s2);
        let k2_v: Vec<DVec3> = s2.velocities.clone();

        // k3: state at t + dt/2 using k2
        let mut s3 = state.clone();
        for i in 0..n {
            s3.positions[i] = state.positions[i] + k2_v[i] * (dt / 2.0);
            s3.velocities[i] = state.velocities[i] + k2_a[i] * (dt / 2.0);
        }
        let k3_a = accel_fn(&s3);
        let k3_v: Vec<DVec3> = s3.velocities.clone();

        // k4: state at t + dt using k3
        let mut s4 = state.clone();
        for i in 0..n {
            s4.positions[i] = state.positions[i] + k3_v[i] * dt;
            s4.velocities[i] = state.velocities[i] + k3_a[i] * dt;
        }
        let k4_a = accel_fn(&s4);
        let k4_v: Vec<DVec3> = s4.velocities.clone();

        // Combine: state += (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        for i in 0..n {
            state.positions[i] += (k1_v[i] + k2_v[i] * 2.0 + k3_v[i] * 2.0 + k4_v[i]) * (dt / 6.0);
            state.velocities[i] += (k1_a[i] + k2_a[i] * 2.0 + k3_a[i] * 2.0 + k4_a[i]) * (dt / 6.0);
        }
    }
}
