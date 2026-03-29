use glam::DVec3;

use super::{NodeId, SimState};

pub trait BurnModel: Send + Sync {
    /// Thrust acceleration to apply this substep (world-space, m/s^2).
    fn acceleration(
        &self,
        state: &SimState,
        ship_idx: usize,
        dominant_body_idx: usize,
        time: f64,
        dt: f64,
    ) -> Option<DVec3>;

    /// Time window during which this burn is active. (start, end).
    fn time_window(&self) -> (f64, f64);

    /// Total delta-v magnitude.
    fn total_delta_v(&self) -> f64;

    /// Delta-v in orbital frame (prograde, normal, radial). Default: zero.
    fn delta_v_orbital(&self) -> DVec3 { DVec3::ZERO }

    /// Clone into a new Box.
    fn clone_box(&self) -> Box<dyn BurnModel>;
}

#[derive(Clone)]
pub struct ImpulseBurn {
    /// Simulation time of the impulse.
    pub time: f64,
    /// Delta-v in orbital frame: (prograde, normal, radial).
    pub delta_v: DVec3,
}

impl BurnModel for ImpulseBurn {
    fn acceleration(
        &self,
        state: &SimState,
        ship_idx: usize,
        dominant_body_idx: usize,
        time: f64,
        dt: f64,
    ) -> Option<DVec3> {
        // Active only during the single substep containing self.time.
        let step_start = time;
        let step_end = time + dt;
        if self.time < step_start || self.time > step_end {
            return None;
        }

        // Convert orbital-frame delta-v to world-space acceleration.
        let frame = super::orbital_frame(
            state.positions[ship_idx],
            state.velocities[ship_idx],
            state.positions[dominant_body_idx],
        );
        let world_dv = frame * self.delta_v;
        Some(world_dv / dt)
    }

    fn time_window(&self) -> (f64, f64) {
        (self.time, self.time)
    }

    fn total_delta_v(&self) -> f64 {
        self.delta_v.length()
    }

    fn delta_v_orbital(&self) -> DVec3 {
        self.delta_v
    }

    fn clone_box(&self) -> Box<dyn BurnModel> {
        Box::new(self.clone())
    }
}

pub struct ManeuverNode {
    pub id: NodeId,
    pub burn: Box<dyn BurnModel>,
}

impl Clone for ManeuverNode {
    fn clone(&self) -> Self {
        Self {
            id: self.id,
            burn: self.burn.clone_box(),
        }
    }
}
