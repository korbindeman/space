use glam::DVec3;

use space_sim::NodeId;

use super::phase::{PredictionPhase, TerminationReason};

#[derive(Clone, Debug)]
pub struct TrailSegment {
    pub points: Vec<DVec3>,
    /// Exact ship velocity at each trail point (from ghost sim state, not finite differences).
    pub velocities: Vec<DVec3>,
    /// body_positions[step_idx][body_idx] — every body's position at every trail timestep.
    pub body_positions: Vec<Vec<DVec3>>,
    /// body_velocities[step_idx][body_idx] — every body's velocity at every trail timestep.
    pub body_velocities: Vec<Vec<DVec3>>,
    pub times: Vec<f64>,
    /// Per-point dominant body index.
    pub dominant_body: Vec<usize>,
    /// Per-point prediction phase (for rendering style).
    pub phase: Vec<PredictionPhase>,
    /// Which maneuver node this segment follows (None for the first segment).
    pub after_node: Option<NodeId>,
}

impl TrailSegment {
    pub fn new(after_node: Option<NodeId>) -> Self {
        Self {
            points: Vec::new(),
            velocities: Vec::new(),
            body_positions: Vec::new(),
            body_velocities: Vec::new(),
            times: Vec::new(),
            dominant_body: Vec::new(),
            phase: Vec::new(),
            after_node,
        }
    }

    pub fn push_point(
        &mut self,
        ship_pos: DVec3,
        ship_vel: DVec3,
        all_body_positions: Vec<DVec3>,
        all_body_velocities: Vec<DVec3>,
        time: f64,
        dominant: usize,
        phase: PredictionPhase,
    ) {
        self.points.push(ship_pos);
        self.velocities.push(ship_vel);
        self.body_positions.push(all_body_positions);
        self.body_velocities.push(all_body_velocities);
        self.times.push(time);
        self.dominant_body.push(dominant);
        self.phase.push(phase);
    }
}

#[derive(Clone, Debug)]
pub struct ClosestApproach {
    pub body_idx: usize,
    pub position: DVec3,
    pub body_position: DVec3,
    pub distance: f64,
    pub time: f64,
    pub is_collision: bool,
}

#[derive(Clone, Debug)]
pub struct Encounter {
    pub body_idx: usize,
    pub entry_time: f64,
    pub exit_time: f64,
    pub closest_approach: f64,
    pub closest_time: f64,
    pub relative_velocity: f64,
    pub capture: CaptureStatus,
    pub periapsis_altitude: f64,
    pub eccentricity: f64,
    pub inclination: f64,
    /// Segment index of closest approach point (for looking up body positions).
    pub closest_segment_idx: usize,
    /// Point index within segment of closest approach.
    pub closest_point_idx: usize,
}

#[derive(Clone, Debug)]
pub enum CaptureStatus {
    Flyby,
    Captured,
    Impact,
    Graze { altitude: f64 },
}

#[derive(Clone, Debug)]
pub struct PredictionResult {
    pub segments: Vec<TrailSegment>,
    pub encounters: Vec<Encounter>,
    pub approaches: Vec<ClosestApproach>,
    pub termination: TerminationReason,
}

pub struct PredictionConfig {
    pub max_steps: usize,
    pub base_dt: f64,
    pub adaptive_dt: bool,
    pub target_body: Option<usize>,
    pub body_radii: Vec<f64>,
    pub body_hill_radii: Vec<f64>,
    pub body_parent: Vec<Option<usize>>,
    pub extend_count: usize,
}

impl Default for PredictionConfig {
    fn default() -> Self {
        Self {
            max_steps: 10_000,
            base_dt: 60.0,
            adaptive_dt: true,
            target_body: None,
            body_radii: Vec::new(),
            body_hill_radii: Vec::new(),
            body_parent: Vec::new(),
            extend_count: 0,
        }
    }
}

/// Raw encounter data accumulated during propagation by the phase machine.
#[derive(Clone, Debug)]
pub struct RawEncounter {
    pub body_idx: usize,
    pub entry_time: f64,
    pub exit_time: f64,
    pub closest_distance: f64,
    pub closest_time: f64,
    /// Index into the trail segment where closest approach occurred.
    pub closest_segment_idx: usize,
    pub closest_point_idx: usize,
    pub captured: bool,
    pub collided: bool,
}
