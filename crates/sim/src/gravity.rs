use glam::DVec3;

use super::SimState;

pub trait ForceModel: Send + Sync {
    fn compute_accelerations(&self, state: &SimState) -> Vec<DVec3>;
}

/// Standard gravitational constant (m^3 kg^-1 s^-2).
pub const G: f64 = 6.674_301_5e-11;

/// N-body pairwise gravitational acceleration.
///
/// `exclude_from_sources`: indices that don't gravitationally influence others
/// (e.g. a ship with negligible mass). They still *receive* gravity from all bodies.
pub struct NBodyGravity {
    pub exclude_from_sources: Vec<usize>,
}

impl NBodyGravity {
    pub fn new() -> Self {
        Self {
            exclude_from_sources: Vec::new(),
        }
    }

    pub fn with_exclusions(exclude: Vec<usize>) -> Self {
        Self {
            exclude_from_sources: exclude,
        }
    }
}

impl ForceModel for NBodyGravity {
    fn compute_accelerations(&self, state: &SimState) -> Vec<DVec3> {
        let n = state.body_count();
        let mut accels = vec![DVec3::ZERO; n];

        for i in 0..n {
            for j in 0..n {
                if i == j {
                    continue;
                }
                // Skip if j is excluded as a source (its mass is negligible)
                if self.exclude_from_sources.contains(&j) {
                    continue;
                }
                let r = state.positions[j] - state.positions[i];
                let dist_sq = r.length_squared();
                let dist = dist_sq.sqrt();
                // a = G * m_j * r_hat / r^2 = G * m_j * r / r^3
                accels[i] += r * (G * state.masses[j] / (dist_sq * dist));
            }
        }

        accels
    }
}
