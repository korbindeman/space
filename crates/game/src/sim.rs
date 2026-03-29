use bevy::prelude::*;
use bevy::math::DVec3;

use space_prediction::phase::dominant_body;
use space_sim::*;

// Physical constants
const SUN_MASS: f64 = 1.989e30;
const EARTH_MASS: f64 = 5.972e24;
const MOON_MASS: f64 = 7.342e22;
const AU: f64 = 1.496e11;
const EARTH_MOON_DIST: f64 = 3.844e8;
const EARTH_RADIUS: f64 = 6.371e6;
const MOON_RADIUS: f64 = 1.737e6;
const SUN_RADIUS: f64 = 6.957e8;

/// Maximum substep size in seconds.
const MAX_SUBSTEP: f64 = 100.0;

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
            warp_levels: vec![0.0, 1.0, 5.0, 10.0, 50.0, 100.0, 1_000.0, 10_000.0, 100_000.0, 1_000_000.0],
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

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimClock>()
            .init_resource::<LiveDominantBody>()
            .add_systems(Startup, setup)
            .add_systems(Update, (
                step_simulation,
                update_live_dominant_body,
            ));
    }
}

// --- Systems ---

/// Set up the initial simulation state and spawn body/ship entities (without visuals).
pub fn setup(
    mut commands: Commands,
) {
    // --- Build initial SimState ---
    let earth_v = (G * SUN_MASS / AU).sqrt();
    let moon_v = (G * EARTH_MASS / EARTH_MOON_DIST).sqrt();
    let ship_alt = 400_000.0;
    let ship_r = EARTH_RADIUS + ship_alt;
    let ship_v = (G * EARTH_MASS / ship_r).sqrt();

    let earth_pos = DVec3::new(AU, 0.0, 0.0);
    let earth_vel = DVec3::new(0.0, 0.0, earth_v);
    let moon_pos = DVec3::new(AU + EARTH_MOON_DIST, 0.0, 0.0);
    let moon_vel = DVec3::new(0.0, 0.0, earth_v + moon_v);
    let ship_pos = earth_pos + DVec3::new(ship_r, 0.0, 0.0);
    let ship_vel = earth_vel + DVec3::new(0.0, 0.0, ship_v);

    let state = SimState {
        positions: vec![DVec3::ZERO, earth_pos, moon_pos, ship_pos],
        velocities: vec![DVec3::ZERO, earth_vel, moon_vel, ship_vel],
        masses: vec![SUN_MASS, EARTH_MASS, MOON_MASS, 1000.0],
    };

    let ship_idx = 3;

    // --- Spawn celestial body entities (data only, no meshes) ---
    let earth_hill = hill_radius(AU, EARTH_MASS, SUN_MASS);
    let moon_hill = hill_radius(EARTH_MOON_DIST, MOON_MASS, EARTH_MASS);

    struct BodyDef {
        name: &'static str,
        idx: usize,
        radius: f64,
        hill: f64,
        color: Color,
    }

    let body_defs = [
        BodyDef {
            name: "Sun",
            idx: 0,
            radius: SUN_RADIUS,
            hill: 0.0,
            color: Color::srgb(1.0, 0.95, 0.3),
        },
        BodyDef {
            name: "Earth",
            idx: 1,
            radius: EARTH_RADIUS,
            hill: earth_hill,
            color: Color::srgb(0.2, 0.4, 0.9),
        },
        BodyDef {
            name: "Moon",
            idx: 2,
            radius: MOON_RADIUS,
            hill: moon_hill,
            color: Color::srgb(0.6, 0.6, 0.6),
        },
    ];

    for def in &body_defs {
        commands.spawn((
            Transform::default(),
            Visibility::Inherited,
            SimBody(def.idx),
            CelestialBody {
                name: def.name.to_string(),
                radius: def.radius,
                hill_radius: def.hill,
                color: def.color,
            },
        ));
    }

    // --- Spawn ship entity (data only) ---
    let ship_entity = commands.spawn((
        Transform::default(),
        Visibility::Inherited,
        SimBody(ship_idx),
        Ship,
    )).id();

    // --- Lighting ---
    commands.spawn(AmbientLight {
        color: Color::WHITE,
        brightness: 50.0,
        ..default()
    });

    // --- Insert resources ---
    commands.insert_resource(PhysicsState {
        state,
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
        thrust_newtons: 10_000.0,
        dry_mass: 800.0,
        fuel_mass: 200.0,
    });
}

/// Helper: step the physics state forward by dt.
fn integrate_step(physics: &mut PhysicsState, dt: f64) {
    let PhysicsState { state, integrator, force_model } = physics;
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
) {
    if clock.paused() {
        return;
    }

    let frame_dt = time.delta_secs_f64();
    let effective_dt = frame_dt * clock.warp;
    if effective_dt <= 0.0 {
        return;
    }

    let num_substeps = (effective_dt / MAX_SUBSTEP).ceil().max(1.0) as usize;
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
            let dom = dominant_body(&physics.state, ship_idx);
            if let Some(accel) = plan.nodes[node_idx].burn.acceleration(
                &physics.state, ship_idx, dom, burn_start, 1.0,
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
    bodies: Query<(Entity, &SimBody)>,
) {
    let dom_idx = dominant_body(&physics.state, ship_config.sim_index);
    live_dom.index = dom_idx;
    live_dom.entity = bodies.iter().find(|(_, sb)| sb.0 == dom_idx).map(|(e, _)| e);
}
