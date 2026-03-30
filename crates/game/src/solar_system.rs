use bevy::math::DVec3;
use bevy::prelude::Color;
use serde::Deserialize;
use space_sim::{G, SimState};

// --- Declarative format ---

#[derive(Deserialize)]
pub struct SolarSystemDef {
    pub bodies: Vec<BodyDef>,
    pub ship: ShipDef,
}

#[derive(Deserialize)]
pub struct BodyDef {
    pub name: String,
    pub parent: Option<String>,
    pub mass: Mass,
    pub radius_km: f64,
    pub orbit: Option<OrbitDef>,
    pub color: (f32, f32, f32),
    #[serde(default)]
    pub is_star: bool,
}

#[derive(Deserialize, Clone, Copy)]
pub enum Mass {
    Solar(f64),
    Jupiter(f64),
    Earth(f64),
    Kg(f64),
}

#[derive(Deserialize, Clone, Copy)]
pub struct OrbitDef {
    /// Semi-major axis.
    pub radius: Distance,
    pub eccentricity: f64,
    /// True anomaly in degrees. Where the body starts on its orbit.
    #[serde(default)]
    pub start_angle: f64,
    /// Orbital inclination in degrees.
    #[serde(default)]
    pub inclination: f64,
    /// Argument of periapsis in degrees. Orients the ellipse within the orbital plane.
    #[serde(default)]
    pub arg_periapsis: f64,
    /// Longitude of ascending node in degrees. Orients the orbital plane in 3D.
    #[serde(default)]
    pub lon_ascending_node: f64,
}

#[derive(Deserialize, Clone, Copy)]
pub enum Distance {
    Au(f64),
    Km(f64),
}

#[derive(Deserialize)]
pub struct ShipDef {
    pub parent: String,
    pub altitude_km: f64,
    pub mass_kg: f64,
    pub thrust_newtons: f64,
    pub dry_mass_kg: f64,
    pub fuel_mass_kg: f64,
}

// --- Unit conversions ---

const SOLAR_MASS: f64 = 1.989e30;
const JUPITER_MASS: f64 = 1.898e27;
const EARTH_MASS: f64 = 5.972e24;
const AU_METERS: f64 = 1.496e11;

impl Mass {
    pub fn to_kg(self) -> f64 {
        match self {
            Mass::Solar(v) => v * SOLAR_MASS,
            Mass::Jupiter(v) => v * JUPITER_MASS,
            Mass::Earth(v) => v * EARTH_MASS,
            Mass::Kg(v) => v,
        }
    }
}

impl Distance {
    pub fn to_meters(self) -> f64 {
        match self {
            Distance::Au(v) => v * AU_METERS,
            Distance::Km(v) => v * 1_000.0,
        }
    }
}

// --- Resolved output ---

pub struct ResolvedBody {
    pub name: String,
    pub sim_index: usize,
    pub radius_m: f64,
    pub hill_radius_m: f64,
    pub color: Color,
    pub parent_index: Option<usize>,
    pub is_star: bool,
}

pub struct ResolvedShip {
    pub sim_index: usize,
    pub parent_body_index: usize,
    pub mass_kg: f64,
    pub thrust_newtons: f64,
    pub dry_mass_kg: f64,
    pub fuel_mass_kg: f64,
}

pub struct ResolvedSystem {
    pub state: SimState,
    pub bodies: Vec<ResolvedBody>,
    pub ship: ResolvedShip,
}

// --- Loader ---

pub fn load(ron_str: &str) -> ResolvedSystem {
    let def: SolarSystemDef = ron::from_str(ron_str).expect("failed to parse solar_system.ron");
    resolve(&def)
}

/// Apply Rodrigues' rotation: rotate `vec` around `axis` (must be unit) by `angle` radians.
fn rodrigues(vec: DVec3, axis: DVec3, angle: f64) -> DVec3 {
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    vec * cos_a + axis.cross(vec) * sin_a + axis * axis.dot(vec) * (1.0 - cos_a)
}

/// Compute position and velocity offset from parent for a Keplerian orbit.
///
/// Full orbital elements: semi-major axis (a), eccentricity (e), true anomaly (ν),
/// inclination (i), argument of periapsis (ω), longitude of ascending node (Ω).
///
/// Returns (position_offset, velocity_offset) in the parent's frame.
fn keplerian_state(
    a: f64,
    e: f64,
    true_anomaly: f64,
    incl: f64,
    arg_periapsis: f64,
    lon_ascending_node: f64,
    parent_mass: f64,
) -> (DVec3, DVec3) {
    let nu = true_anomaly;

    // Distance from focus: r = a(1 - e²) / (1 + e·cos(ν))
    let r = a * (1.0 - e * e) / (1.0 + e * nu.cos());

    // Speed from vis-viva: v = sqrt(GM * (2/r - 1/a))
    let mu = G * parent_mass;
    let speed = (mu * (2.0 / r - 1.0 / a)).sqrt();

    // Flight path angle: angle between velocity and local horizontal
    // tan(γ) = e·sin(ν) / (1 + e·cos(ν))
    let flight_path_angle = (e * nu.sin()).atan2(1.0 + e * nu.cos());

    // Position and velocity in the perifocal frame (orbit plane, periapsis along X)
    // Angle from periapsis = ν, so in the orbital plane:
    let angle_in_plane = arg_periapsis + nu;

    let pos_plane = DVec3::new(
        r * angle_in_plane.cos(),
        0.0,
        r * angle_in_plane.sin(),
    );

    // Velocity direction: perpendicular to radius rotated by flight path angle
    // In the orbital plane, the "horizontal" direction (perpendicular to radius, prograde) is
    // at angle (angle_in_plane + π/2). The flight path angle tilts toward radial.
    let vel_angle = angle_in_plane + std::f64::consts::FRAC_PI_2 - flight_path_angle;
    let vel_plane = DVec3::new(
        speed * vel_angle.cos(),
        0.0,
        speed * vel_angle.sin(),
    );

    // Rotate by inclination around the line of nodes (X axis rotated by Ω)
    // The line of nodes is at angle Ω in the reference XZ plane.
    let node_axis = DVec3::new(lon_ascending_node.cos(), 0.0, lon_ascending_node.sin());

    let pos = rodrigues(pos_plane, node_axis, incl);
    let vel = rodrigues(vel_plane, node_axis, incl);

    (pos, vel)
}

fn resolve(def: &SolarSystemDef) -> ResolvedSystem {
    let body_count = def.bodies.len();
    let ship_index = body_count; // ship gets the index after all bodies
    let total = body_count + 1;

    let mut positions = vec![DVec3::ZERO; total];
    let mut velocities = vec![DVec3::ZERO; total];
    let mut masses = vec![0.0_f64; total];
    let mut resolved_bodies = Vec::with_capacity(body_count);

    // Build name -> index map for parent lookups
    let name_to_index: std::collections::HashMap<&str, usize> = def
        .bodies
        .iter()
        .enumerate()
        .map(|(i, b)| (b.name.as_str(), i))
        .collect();

    for (i, body) in def.bodies.iter().enumerate() {
        let mass_kg = body.mass.to_kg();
        masses[i] = mass_kg;

        let parent_index = body.parent.as_ref().map(|name| {
            *name_to_index
                .get(name.as_str())
                .unwrap_or_else(|| panic!("unknown parent '{}' for body '{}'", name, body.name))
        });

        let mut hill_radius_m = 0.0;

        if let (Some(orbit), Some(pi)) = (&body.orbit, parent_index) {
            let sma = orbit.radius.to_meters();
            let parent_mass = masses[pi];
            let nu = orbit.start_angle.to_radians();
            let incl = orbit.inclination.to_radians();
            let omega = orbit.arg_periapsis.to_radians();
            let big_omega = orbit.lon_ascending_node.to_radians();

            let (pos_offset, vel_offset) =
                keplerian_state(sma, orbit.eccentricity, nu, incl, omega, big_omega, parent_mass);

            positions[i] = positions[pi] + pos_offset;
            velocities[i] = velocities[pi] + vel_offset;

            // Hill sphere: r_H = a * (m / 3M)^(1/3)
            hill_radius_m = space_sim::hill_radius(sma, mass_kg, parent_mass);
        }

        resolved_bodies.push(ResolvedBody {
            name: body.name.clone(),
            sim_index: i,
            radius_m: body.radius_km * 1_000.0,
            hill_radius_m,
            color: Color::srgb(body.color.0, body.color.1, body.color.2),
            parent_index,
            is_star: body.is_star,
        });
    }

    // Ship: circular orbit around its parent body at the given altitude
    let ship_parent_index = *name_to_index
        .get(def.ship.parent.as_str())
        .unwrap_or_else(|| panic!("unknown ship parent '{}'", def.ship.parent));

    let parent_radius = def.bodies[ship_parent_index].radius_km * 1_000.0;
    let ship_altitude_m = def.ship.altitude_km * 1_000.0;
    let ship_r = parent_radius + ship_altitude_m;
    let ship_v = (G * masses[ship_parent_index] / ship_r).sqrt();

    // Place ship at +X from parent (relative to parent's current position)
    positions[ship_index] = positions[ship_parent_index] + DVec3::new(ship_r, 0.0, 0.0);
    // Velocity perpendicular to position offset, matching parent's velocity
    velocities[ship_index] = velocities[ship_parent_index] + DVec3::new(0.0, 0.0, ship_v);
    masses[ship_index] = def.ship.mass_kg;

    ResolvedSystem {
        state: SimState {
            positions,
            velocities,
            masses,
        },
        bodies: resolved_bodies,
        ship: ResolvedShip {
            sim_index: ship_index,
            parent_body_index: ship_parent_index,
            mass_kg: def.ship.mass_kg,
            thrust_newtons: def.ship.thrust_newtons,
            dry_mass_kg: def.ship.dry_mass_kg,
            fuel_mass_kg: def.ship.fuel_mass_kg,
        },
    }
}
