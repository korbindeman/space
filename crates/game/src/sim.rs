use bevy::math::DVec3;
use bevy::prelude::*;

use space_prediction::phase::dominant_body;
use space_sim::*;

/// Maximum substep size in seconds.
const MAX_SUBSTEP: f64 = 100.0;
/// Maximum substeps per frame. Caps CPU cost; sim falls behind real-time at extreme warp.
const MAX_SUBSTEPS_PER_FRAME: usize = 500;

// --- Components ---

/// Maps a Bevy entity to its index in SimState.
#[derive(Component)]
pub struct SimBody(pub usize);

#[derive(Component)]
pub struct CelestialBody {
    pub name: String,
    pub radius: f64,
    pub hill_radius: f64,
    pub color: Color,
    pub is_star: bool,
}

#[derive(Component)]
pub struct Ship;

// --- Resources ---

#[derive(Resource)]
pub struct PhysicsState {
    pub state: SimState,
    pub integrator: Box<dyn Integrator>,
    pub force_model: Box<dyn ForceModel>,
}

#[derive(Resource)]
pub struct SimClock {
    pub time: f64,
    pub warp: f64,
    pub warp_levels: Vec<f64>,
    pub warp_index: usize,
    /// Remembers the last non-zero warp index so pause/resume can toggle back.
    pub resume_index: usize,
}

impl Default for SimClock {
    fn default() -> Self {
        Self {
            time: 0.0,
            warp: 0.0,
            warp_levels: vec![
                0.0,
                1.0,
                5.0,
                10.0,
                50.0,
                100.0,
                1_000.0,
                10_000.0,
                100_000.0,
                1_000_000.0,
            ],
            warp_index: 0,
            resume_index: 1,
        }
    }
}

impl SimClock {
    pub fn paused(&self) -> bool {
        self.warp_index == 0
    }

    pub fn toggle_pause(&mut self) {
        if self.paused() {
            self.warp_index = self.resume_index;
        } else {
            self.resume_index = self.warp_index;
            self.warp_index = 0;
        }
        self.warp = self.warp_levels[self.warp_index];
    }
}

#[derive(Resource)]
pub struct ShipConfig {
    pub sim_index: usize,
    pub thrust_newtons: f64,
    pub dry_mass: f64,
    pub fuel_mass: f64,
}

/// Static body metadata indexed by sim body index. Built once at setup.
#[derive(Resource, Default, Clone)]
pub struct BodyData {
    pub radii: Vec<f64>,
    pub hill_radii: Vec<f64>,
}

/// The reference frame used for rendering trails and placing maneuver nodes.
/// Computed once per frame from camera focus and dominant body.
#[derive(Resource)]
pub struct TrailFrame {
    /// Sim body index whose position is subtracted from trail points.
    pub index: usize,
    /// Render-space offset from trail frame to camera frame.
    pub offset: Vec3,
}

#[derive(Resource)]
pub struct LiveDominantBody {
    pub index: usize,
    pub entity: Option<Entity>,
}

impl Default for LiveDominantBody {
    fn default() -> Self {
        Self {
            index: 1, // Earth
            entity: None,
        }
    }
}

// --- Plugin ---

pub struct SimPlugin;

impl Default for TrailFrame {
    fn default() -> Self {
        Self {
            index: 1,
            offset: Vec3::ZERO,
        }
    }
}

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimClock>()
            .add_systems(Startup, setup)
            .add_systems(
                Update,
                (
                    step_simulation,
                    update_live_dominant_body,
                    update_trail_frame.after(update_live_dominant_body),
                ),
            );
    }
}

// --- Systems ---

/// Set up the initial simulation state and spawn body/ship entities (without visuals).
pub fn setup(mut commands: Commands) {
    let ron_str = include_str!("../../../assets/solar_system.ron");
    let system = crate::solar_system::load(ron_str);

    let ship_idx = system.ship.sim_index;
    let homeworld_idx = system.ship.parent_body_index;

    // --- Spawn celestial body entities (data only, no meshes) ---
    for body in &system.bodies {
        commands.spawn((
            Transform::default(),
            Visibility::Inherited,
            SimBody(body.sim_index),
            CelestialBody {
                name: body.name.clone(),
                radius: body.radius_m,
                hill_radius: body.hill_radius_m,
                color: body.color,
                is_star: body.is_star,
            },
        ));
    }

    // --- Spawn ship entity (data only) ---
    let ship_entity = commands
        .spawn((
            Transform::default(),
            Visibility::Inherited,
            SimBody(ship_idx),
            Ship,
        ))
        .id();

    // --- Lighting ---
    commands.spawn(AmbientLight {
        color: Color::WHITE,
        brightness: 50.0,
        ..default()
    });

    // --- Build static body metadata indexed by sim body index ---
    let num_bodies = ship_idx + 1;
    let mut body_data = BodyData {
        radii: vec![0.0; num_bodies],
        hill_radii: vec![0.0; num_bodies],
    };
    for body in &system.bodies {
        body_data.radii[body.sim_index] = body.radius_m;
        body_data.hill_radii[body.sim_index] = body.hill_radius_m;
    }

    // --- Insert resources ---
    commands.insert_resource(PhysicsState {
        state: system.state,
        integrator: Box::new(RK4Integrator),
        force_model: Box::new(NBodyGravity::with_exclusions(vec![ship_idx])),
    });

    commands.insert_resource(crate::nav_map::camera::CameraFocus {
        entity: ship_entity,
        body_index: Some(ship_idx),
        active_frame: ship_idx,
    });

    commands.insert_resource(ShipConfig {
        sim_index: ship_idx,
        thrust_newtons: system.ship.thrust_newtons,
        dry_mass: system.ship.dry_mass_kg,
        fuel_mass: system.ship.fuel_mass_kg,
    });

    commands.insert_resource(LiveDominantBody {
        index: homeworld_idx,
        entity: None,
    });

    commands.insert_resource(TrailFrame {
        index: homeworld_idx,
        offset: Vec3::ZERO,
    });

    commands.insert_resource(body_data);
}

/// Helper: step the physics state forward by dt.
fn integrate_step(physics: &mut PhysicsState, dt: f64) {
    let PhysicsState {
        state,
        integrator,
        force_model,
    } = physics;
    let fm = force_model.as_ref();
    integrator.step(state, &|s| fm.compute_accelerations(s), dt);
}

/// Advance the simulation by frame_dt * warp, subdivided into substeps.
fn step_simulation(
    time: Res<Time>,
    mut clock: ResMut<SimClock>,
    mut physics: ResMut<PhysicsState>,
    mut plan: ResMut<crate::nav_map::maneuver::ManeuverPlan>,
    ship_config: Res<ShipConfig>,
    body_data: Res<BodyData>,
) {
    if clock.paused() {
        return;
    }

    // Clamp frame delta to prevent runaway catch-up after lag spikes
    let frame_dt = time.delta_secs_f64().min(0.1);
    let effective_dt = frame_dt * clock.warp;
    if effective_dt <= 0.0 {
        return;
    }

    let num_substeps = ((effective_dt / MAX_SUBSTEP).ceil() as usize)
        .max(1)
        .min(MAX_SUBSTEPS_PER_FRAME);
    let substep_dt = effective_dt / num_substeps as f64;
    let ship_idx = ship_config.sim_index;

    for _ in 0..num_substeps {
        let t = clock.time;
        let t_end = t + substep_dt;

        // Check for impulse burns within this substep
        let mut impulse_time_opt: Option<(f64, usize)> = None;
        for (i, node) in plan.nodes.iter().enumerate() {
            let (burn_start, burn_end) = node.burn.time_window();
            if (burn_end - burn_start).abs() < 1e-10 && burn_start >= t && burn_start < t_end {
                impulse_time_opt = Some((burn_start, i));
                break;
            }
        }

        if let Some((burn_start, node_idx)) = impulse_time_opt {
            let dt_before = burn_start - t;
            if dt_before > 1e-10 {
                integrate_step(&mut physics, dt_before);
            }

            // Apply impulse
            let dom = dominant_body(&physics.state, ship_idx, &body_data.hill_radii);
            if let Some(accel) = plan.nodes[node_idx].burn.acceleration(
                &physics.state,
                ship_idx,
                dom,
                burn_start,
                1.0,
            ) {
                physics.state.velocities[ship_idx] += accel;
            }

            let dt_after = t_end - burn_start;
            if dt_after > 1e-10 {
                integrate_step(&mut physics, dt_after);
            }
        } else {
            integrate_step(&mut physics, substep_dt);
        }

        clock.time += substep_dt;
    }

    plan.nodes.retain(|node| {
        let (_, end) = node.burn.time_window();
        end > clock.time
    });
}

/// Update which body exerts the strongest gravity on the ship.
fn update_live_dominant_body(
    physics: Res<PhysicsState>,
    mut live_dom: ResMut<LiveDominantBody>,
    ship_config: Res<ShipConfig>,
    body_data: Res<BodyData>,
    bodies: Query<(Entity, &SimBody)>,
) {
    let dom_idx = dominant_body(&physics.state, ship_config.sim_index, &body_data.hill_radii);
    live_dom.index = dom_idx;
    live_dom.entity = bodies
        .iter()
        .find(|(_, sb)| sb.0 == dom_idx)
        .map(|(e, _)| e);
}

/// Compute the trail reference frame: focused body's frame, or dominant body when focused on ship.
fn update_trail_frame(
    camera_focus: Res<crate::nav_map::camera::CameraFocus>,
    live_dom: Res<LiveDominantBody>,
    ship_config: Res<ShipConfig>,
    physics: Res<PhysicsState>,
    mut trail_frame: ResMut<TrailFrame>,
) {
    let frame_idx = if camera_focus.active_frame == ship_config.sim_index {
        live_dom.index
    } else {
        camera_focus.active_frame
    };
    trail_frame.index = frame_idx;

    // Offset from trail frame to camera frame in render space
    let cam_frame = camera_focus.active_frame;
    trail_frame.offset = if frame_idx == cam_frame {
        Vec3::ZERO
    } else {
        let trail_pos = physics
            .state
            .positions
            .get(frame_idx)
            .copied()
            .unwrap_or(DVec3::ZERO);
        let cam_pos = physics
            .state
            .positions
            .get(cam_frame)
            .copied()
            .unwrap_or(DVec3::ZERO);
        ((trail_pos - cam_pos) * crate::nav_map::RENDER_SCALE).as_vec3()
    };
}
