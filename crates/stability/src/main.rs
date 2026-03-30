use glam::DVec3;
use serde::Deserialize;
use space_sim::*;

// --- Minimal .ron parser (no Bevy dependency) ---

#[derive(Deserialize)]
struct SolarSystemDef {
    bodies: Vec<BodyDef>,
    #[allow(dead_code)]
    ship: ShipDef,
}

#[derive(Deserialize)]
struct BodyDef {
    name: String,
    parent: Option<String>,
    mass: Mass,
    radius_km: f64,
    orbit: Option<OrbitDef>,
    #[allow(dead_code)]
    color: (f32, f32, f32),
    #[serde(default)]
    #[allow(dead_code)]
    is_star: bool,
}

#[derive(Deserialize, Clone, Copy)]
enum Mass {
    Solar(f64),
    Jupiter(f64),
    Earth(f64),
    Kg(f64),
}

#[derive(Deserialize, Clone, Copy)]
struct OrbitDef {
    radius: Distance,
    eccentricity: f64,
    #[serde(default)]
    start_angle: f64,
    #[serde(default)]
    inclination: f64,
    #[serde(default)]
    arg_periapsis: f64,
    #[serde(default)]
    lon_ascending_node: f64,
}

#[derive(Deserialize, Clone, Copy)]
enum Distance {
    Au(f64),
    Km(f64),
}

#[derive(Deserialize)]
struct ShipDef {
    #[allow(dead_code)]
    parent: String,
    #[allow(dead_code)]
    altitude_km: f64,
    #[allow(dead_code)]
    mass_kg: f64,
    #[allow(dead_code)]
    thrust_newtons: f64,
    #[allow(dead_code)]
    dry_mass_kg: f64,
    #[allow(dead_code)]
    fuel_mass_kg: f64,
}

const SOLAR_MASS: f64 = 1.989e30;
const JUPITER_MASS: f64 = 1.898e27;
const EARTH_MASS: f64 = 5.972e24;
const AU_METERS: f64 = 1.496e11;

impl Mass {
    fn to_kg(self) -> f64 {
        match self {
            Mass::Solar(v) => v * SOLAR_MASS,
            Mass::Jupiter(v) => v * JUPITER_MASS,
            Mass::Earth(v) => v * EARTH_MASS,
            Mass::Kg(v) => v,
        }
    }
}

impl Distance {
    fn to_meters(self) -> f64 {
        match self {
            Distance::Au(v) => v * AU_METERS,
            Distance::Km(v) => v * 1_000.0,
        }
    }
}

fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 { a } else { gcd(b, a % b) }
}

// --- Keplerian initialization (same as game) ---

fn rodrigues(vec: DVec3, axis: DVec3, angle: f64) -> DVec3 {
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    vec * cos_a + axis.cross(vec) * sin_a + axis * axis.dot(vec) * (1.0 - cos_a)
}

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
    let r = a * (1.0 - e * e) / (1.0 + e * nu.cos());
    let mu = G * parent_mass;
    let speed = (mu * (2.0 / r - 1.0 / a)).sqrt();
    let flight_path_angle = (e * nu.sin()).atan2(1.0 + e * nu.cos());
    let angle_in_plane = arg_periapsis + nu;

    let pos_plane = DVec3::new(r * angle_in_plane.cos(), 0.0, r * angle_in_plane.sin());
    let vel_angle = angle_in_plane + std::f64::consts::FRAC_PI_2 - flight_path_angle;
    let vel_plane = DVec3::new(speed * vel_angle.cos(), 0.0, speed * vel_angle.sin());

    let node_axis = DVec3::new(lon_ascending_node.cos(), 0.0, lon_ascending_node.sin());
    let pos = rodrigues(pos_plane, node_axis, incl);
    let vel = rodrigues(vel_plane, node_axis, incl);
    (pos, vel)
}

// --- Body metadata ---

struct BodyInfo {
    name: String,
    parent_index: Option<usize>,
    initial_sma: f64,
    initial_ecc: f64,
    radius_m: f64,
}

// --- Load ---

fn load(ron_str: &str) -> (SimState, Vec<BodyInfo>) {
    let def: SolarSystemDef = ron::from_str(ron_str).expect("failed to parse solar_system.ron");

    let body_count = def.bodies.len();
    // No ship — stability tool only cares about celestial bodies
    let mut positions = vec![DVec3::ZERO; body_count];
    let mut velocities = vec![DVec3::ZERO; body_count];
    let mut masses = vec![0.0_f64; body_count];
    let mut infos = Vec::with_capacity(body_count);

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
                .unwrap_or_else(|| panic!("unknown parent '{name}' for body '{}'", body.name))
        });

        let mut initial_sma = 0.0;
        let mut initial_ecc = 0.0;

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
            initial_sma = sma;
            initial_ecc = orbit.eccentricity;
        }

        infos.push(BodyInfo {
            name: body.name.clone(),
            parent_index,
            initial_sma,
            initial_ecc,
            radius_m: body.radius_km * 1_000.0,
        });
    }

    let state = SimState {
        positions,
        velocities,
        masses,
    };
    (state, infos)
}

// --- Stability analysis ---

/// Compute current orbital elements of body `idx` relative to its parent.
fn current_elements(state: &SimState, idx: usize, parent_idx: usize) -> OrbitalElements {
    let rel_pos = state.positions[idx] - state.positions[parent_idx];
    let rel_vel = state.velocities[idx] - state.velocities[parent_idx];
    let mu = G * state.masses[parent_idx];
    orbital_elements(rel_pos, rel_vel, mu)
}

struct StabilityResult {
    name: String,
    initial_sma: f64,
    initial_ecc: f64,
    final_sma: f64,
    final_ecc: f64,
    max_sma_drift: f64,
    max_ecc_drift: f64,
    collided: bool,
    escaped: bool,
}

fn run_stability(
    mut state: SimState,
    infos: &[BodyInfo],
    duration_years: f64,
    dt: f64,
) -> Vec<StabilityResult> {
    let duration_s = duration_years * 365.25 * 86400.0;
    let integrator = RK4Integrator;
    let force_model = NBodyGravity::new();
    let accel_fn = |s: &SimState| force_model.compute_accelerations(s);
    let n = state.positions.len();

    // Tracking per body
    let mut max_sma_drift = vec![0.0_f64; n];
    let mut max_ecc_drift = vec![0.0_f64; n];
    let mut collided = vec![false; n];
    let mut escaped = vec![false; n];

    let sample_interval = 86400.0 * 30.0; // sample every 30 days
    let mut next_sample = sample_interval;
    let mut t = 0.0;

    let total_steps = (duration_s / dt) as u64;
    let report_interval = total_steps / 20;

    let mut step = 0u64;

    while t < duration_s {
        integrator.step(&mut state, &accel_fn, dt);
        t += dt;
        step += 1;

        if report_interval > 0 && step % report_interval == 0 {
            let pct = (t / duration_s * 100.0) as u32;
            let years = t / (365.25 * 86400.0);
            eprint!("\r  simulating... {pct}% ({years:.0} years)");
        }

        if t >= next_sample {
            next_sample += sample_interval;

            for i in 0..n {
                let Some(pi) = infos[i].parent_index else {
                    continue;
                };
                if infos[i].initial_sma == 0.0 {
                    continue;
                }

                let elem = current_elements(&state, i, pi);

                // Check collision: distance < sum of radii
                let dist = (state.positions[i] - state.positions[pi]).length();
                if dist < infos[i].radius_m + infos[pi].radius_m {
                    collided[i] = true;
                }

                // Check escape: positive specific energy
                if elem.specific_energy > 0.0 {
                    escaped[i] = true;
                }

                let sma_drift = ((elem.semi_major_axis - infos[i].initial_sma) / infos[i].initial_sma).abs();
                let ecc_drift = (elem.eccentricity - infos[i].initial_ecc).abs();

                if sma_drift > max_sma_drift[i] {
                    max_sma_drift[i] = sma_drift;
                }
                if ecc_drift > max_ecc_drift[i] {
                    max_ecc_drift[i] = ecc_drift;
                }
            }
        }
    }
    eprintln!("\r  simulating... done                    ");

    // Final elements
    let mut results = Vec::new();
    for i in 0..n {
        let Some(pi) = infos[i].parent_index else {
            continue;
        };
        if infos[i].initial_sma == 0.0 {
            continue;
        }

        let final_elem = current_elements(&state, i, pi);

        results.push(StabilityResult {
            name: infos[i].name.clone(),
            initial_sma: infos[i].initial_sma,
            initial_ecc: infos[i].initial_ecc,
            final_sma: final_elem.semi_major_axis,
            final_ecc: final_elem.eccentricity,
            max_sma_drift: max_sma_drift[i],
            max_ecc_drift: max_ecc_drift[i],
            collided: collided[i],
            escaped: escaped[i],
        });
    }
    results
}

// --- Auto-fix: try small velocity adjustments to reduce drift ---

fn try_fix(
    base_state: &SimState,
    infos: &[BodyInfo],
    body_idx: usize,
    test_years: f64,
    dt: f64,
) -> Option<DVec3> {
    let parent_idx = infos[body_idx].parent_index?;

    // Try velocity perturbations along each axis
    let base_vel = base_state.velocities[body_idx];
    let speed = (base_vel - base_state.velocities[parent_idx]).length();
    let perturbation_scale = speed * 1e-4; // 0.01% of orbital velocity

    let mut best_correction = DVec3::ZERO;

    // First measure baseline drift
    let baseline = run_stability_single(base_state, infos, body_idx, test_years, dt);
    let mut best_drift = baseline.max_sma_drift + baseline.max_ecc_drift * 10.0;
    let original_best = best_drift;

    // Search along each axis direction, both positive and negative
    let directions = [
        DVec3::X,
        DVec3::NEG_X,
        DVec3::Y,
        DVec3::NEG_Y,
        DVec3::Z,
        DVec3::NEG_Z,
    ];

    for &dir in &directions {
        // Binary-ish search: try several magnitudes
        for mag_exp in -1..=3 {
            let magnitude = perturbation_scale * 10.0_f64.powi(mag_exp);
            let correction = dir * magnitude;

            let mut test_state = base_state.clone();
            test_state.velocities[body_idx] += correction;

            let result = run_stability_single(&test_state, infos, body_idx, test_years, dt);
            let drift = result.max_sma_drift + result.max_ecc_drift * 10.0;

            if drift < best_drift {
                best_drift = drift;
                best_correction = correction;
            }
        }
    }

    if best_drift < original_best * 0.8 {
        Some(best_correction)
    } else {
        None
    }
}

fn run_stability_single(
    state: &SimState,
    infos: &[BodyInfo],
    body_idx: usize,
    duration_years: f64,
    dt: f64,
) -> StabilityResult {
    let mut state = state.clone();
    let duration_s = duration_years * 365.25 * 86400.0;
    let integrator = RK4Integrator;
    let force_model = NBodyGravity::new();
    let accel_fn = |s: &SimState| force_model.compute_accelerations(s);
    let parent_idx = infos[body_idx].parent_index.unwrap();

    let mut max_sma_drift = 0.0_f64;
    let mut max_ecc_drift = 0.0_f64;
    let mut did_collide = false;
    let mut did_escape = false;

    let sample_interval = 86400.0 * 30.0;
    let mut next_sample = sample_interval;
    let mut t = 0.0;

    while t < duration_s {
        integrator.step(&mut state, &accel_fn, dt);
        t += dt;

        if t >= next_sample {
            next_sample += sample_interval;

            let elem = current_elements(&state, body_idx, parent_idx);

            let dist = (state.positions[body_idx] - state.positions[parent_idx]).length();
            if dist < infos[body_idx].radius_m + infos[parent_idx].radius_m {
                did_collide = true;
            }
            if elem.specific_energy > 0.0 {
                did_escape = true;
            }

            let sma_drift = ((elem.semi_major_axis - infos[body_idx].initial_sma) / infos[body_idx].initial_sma).abs();
            let ecc_drift = (elem.eccentricity - infos[body_idx].initial_ecc).abs();

            if sma_drift > max_sma_drift {
                max_sma_drift = sma_drift;
            }
            if ecc_drift > max_ecc_drift {
                max_ecc_drift = ecc_drift;
            }
        }
    }

    let final_elem = current_elements(&state, body_idx, parent_idx);

    StabilityResult {
        name: infos[body_idx].name.clone(),
        initial_sma: infos[body_idx].initial_sma,
        initial_ecc: infos[body_idx].initial_ecc,
        final_sma: final_elem.semi_major_axis,
        final_ecc: final_elem.eccentricity,
        max_sma_drift,
        max_ecc_drift,
        collided: did_collide,
        escaped: did_escape,
    }
}

fn format_distance(meters: f64) -> String {
    let au = meters / AU_METERS;
    if au > 0.1 {
        format!("{au:.3} AU")
    } else {
        format!("{:.0} km", meters / 1000.0)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let ron_path = args
        .get(1)
        .map(|s| s.as_str())
        .unwrap_or("assets/solar_system.ron");

    let duration_years: f64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(100.0);

    let dt: f64 = args
        .get(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(3600.0); // 1 hour default

    let auto_fix = args.iter().any(|a| a == "--fix");

    eprintln!("Stability Check");
    eprintln!("  file: {ron_path}");
    eprintln!("  duration: {duration_years} years");
    eprintln!("  timestep: {dt} seconds");
    if auto_fix {
        eprintln!("  auto-fix: enabled");
    }
    eprintln!();

    let ron_str = std::fs::read_to_string(ron_path).unwrap_or_else(|e| {
        eprintln!("Error reading {ron_path}: {e}");
        std::process::exit(1);
    });

    let (state, infos) = load(&ron_str);

    eprintln!("Loaded {} bodies", infos.len());

    // Check timestep adequacy: dt should be < 1/20 of the shortest orbital period
    let mut timestep_warnings = Vec::new();
    for info in &infos {
        if let Some(pi) = info.parent_index {
            if info.initial_sma > 0.0 {
                let period =
                    std::f64::consts::TAU * (info.initial_sma.powi(3) / (G * state.masses[pi])).sqrt();
                let recommended = period / 20.0;
                if dt > recommended {
                    timestep_warnings.push((
                        info.name.clone(),
                        period / 86400.0,
                        recommended,
                    ));
                }
            }
        }
    }
    if !timestep_warnings.is_empty() {
        eprintln!();
        eprintln!("WARNING: timestep too large for some bodies:");
        let mut min_recommended = f64::MAX;
        for (name, period_days, recommended) in &timestep_warnings {
            eprintln!(
                "  {name}: period = {period_days:.1} days, need dt < {recommended:.0}s (current: {dt:.0}s)"
            );
            if *recommended < min_recommended {
                min_recommended = *recommended;
            }
        }
        eprintln!("  Suggested: --dt {:.0}", min_recommended.floor());
        eprintln!("  (Results for these bodies will show numerical drift, not real instability)");
    }
    eprintln!();

    let results = run_stability(state.clone(), &infos, duration_years, dt);

    // Categorize results
    let sma_threshold = 0.01; // 1% drift = warning
    let sma_critical = 0.10;  // 10% drift = critical
    let ecc_threshold = 0.05;
    let ecc_critical = 0.20;

    println!();
    println!("=== Stability Report ({duration_years} years) ===");
    println!();
    println!(
        "{:<12} {:>12} {:>12} {:>10} {:>10}  {}",
        "Body", "SMA drift", "Ecc drift", "SMA (now)", "Ecc (now)", "Status"
    );
    println!("{}", "-".repeat(78));

    let mut unstable_indices = Vec::new();

    for r in &results {
        let status = if r.collided {
            "COLLISION"
        } else if r.escaped {
            "ESCAPED"
        } else if r.max_sma_drift > sma_critical || r.max_ecc_drift > ecc_critical {
            "UNSTABLE"
        } else if r.max_sma_drift > sma_threshold || r.max_ecc_drift > ecc_threshold {
            "WARNING"
        } else {
            "OK"
        };

        let sma_pct = format!("{:+.4}%", r.max_sma_drift * 100.0);
        let ecc_str = format!("{:+.6}", r.max_ecc_drift);
        let sma_now = format_distance(r.final_sma);
        let ecc_now = format!("{:.6}", r.final_ecc);

        println!(
            "{:<12} {:>12} {:>12} {:>10} {:>10}  {}",
            r.name, sma_pct, ecc_str, sma_now, ecc_now, status
        );

        if matches!(status, "UNSTABLE" | "ESCAPED" | "COLLISION" | "WARNING") {
            if let Some(idx) = infos.iter().position(|b| b.name == r.name) {
                unstable_indices.push(idx);
            }
        }
    }

    println!();

    // Diagnostics for unstable bodies
    for &idx in &unstable_indices {
        let info = &infos[idx];
        let r = results.iter().find(|r| r.name == info.name).unwrap();

        println!("--- {} ---", info.name);

        if r.collided {
            let parent_name = info
                .parent_index
                .map(|pi| infos[pi].name.as_str())
                .unwrap_or("?");
            println!("  Collided with parent ({parent_name}).");
            println!("  Suggestion: increase semi-major axis or reduce eccentricity so periapsis clears the parent.");
            let periapsis = info.initial_sma * (1.0 - info.initial_ecc);
            let parent_radius = info.parent_index.map(|pi| infos[pi].radius_m).unwrap_or(0.0);
            println!(
                "  Current periapsis: {} (parent radius: {})",
                format_distance(periapsis),
                format_distance(parent_radius)
            );
        } else if r.escaped {
            println!("  Escaped from parent's gravity.");
            println!("  Suggestion: reduce orbital velocity or semi-major axis.");
        } else {
            println!(
                "  SMA drifted {:.2}% (from {} to {})",
                r.max_sma_drift * 100.0,
                format_distance(r.initial_sma),
                format_distance(r.final_sma)
            );
            println!(
                "  Eccentricity drifted {:.6} (from {:.6} to {:.6})",
                r.max_ecc_drift, r.initial_ecc, r.final_ecc
            );

            // Check if there's a nearby resonance causing trouble
            if let Some(pi) = info.parent_index {
                let parent_parent = infos[pi].parent_index;
                if let Some(ppi) = parent_parent {
                    // Moon orbiting a planet — check if planet's siblings interact
                    let _ = ppi; // future: resonance analysis
                } else {
                    // Planet orbiting star — check resonances with siblings
                    let my_period = std::f64::consts::TAU
                        * (info.initial_sma.powi(3) / (G * state.masses[pi])).sqrt();
                    for j in 0..infos.len() {
                        if j == idx || infos[j].parent_index != Some(pi) {
                            continue;
                        }
                        let their_period = std::f64::consts::TAU
                            * (infos[j].initial_sma.powi(3) / (G * state.masses[pi])).sqrt();
                        if their_period == 0.0 || infos[j].initial_sma == 0.0 {
                            continue;
                        }
                        let ratio = my_period / their_period;
                        // Check for simple integer ratios (reduced fractions only)
                        for p in 1..=7_u32 {
                            for q in 1..=7_u32 {
                                if gcd(p, q) != 1 {
                                    continue; // skip non-reduced fractions
                                }
                                let resonance = p as f64 / q as f64;
                                if (ratio - resonance).abs() < 0.02 {
                                    println!(
                                        "  Near {p}:{q} resonance with {} (period ratio: {ratio:.4})",
                                        infos[j].name
                                    );
                                }
                            }
                        }
                    }
                }
            }

            println!("  Suggestions:");
            println!("    - Adjust semi-major axis to avoid resonances with massive neighbors");
            println!("    - Reduce eccentricity to limit perturbation sensitivity");
        }

        // Auto-fix
        if auto_fix {
            let fix_years = (duration_years / 5.0).max(20.0).min(duration_years);
            eprint!("  Searching for velocity correction ({fix_years:.0}yr test)...");
            if let Some(correction) = try_fix(&state, &infos, idx, fix_years, dt) {
                let mag = correction.length();
                eprintln!(" found!");
                println!(
                    "  Fix: apply delta-v of {:.3} m/s [{:+.4}, {:+.4}, {:+.4}] m/s",
                    mag, correction.x, correction.y, correction.z
                );
            } else {
                eprintln!(" no improvement found with simple corrections.");
                println!("  No simple velocity correction improves stability.");
            }
        }

        println!();
    }

    if unstable_indices.is_empty() {
        println!("All orbits stable over {duration_years} years.");
    }
}
