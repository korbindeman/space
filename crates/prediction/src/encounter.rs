use glam::DVec3;

use space_sim::{gravity::G, orbital_elements};

use super::types::{
    CaptureStatus, ClosestApproach, Encounter, RawEncounter, TrailSegment,
};

/// Enrich raw encounter data with orbital elements.
pub fn detect_encounters(
    raw_encounters: &[RawEncounter],
    segments: &[TrailSegment],
    body_radii: &[f64],
    body_masses: &[f64],
) -> Vec<Encounter> {
    raw_encounters
        .iter()
        .filter_map(|raw| {
            let seg = segments.get(raw.closest_segment_idx)?;
            let pt_idx = raw.closest_point_idx;
            if pt_idx >= seg.points.len() || raw.body_idx >= seg.body_positions.get(pt_idx)?.len() {
                return None;
            }

            let ship_pos = seg.points[pt_idx];
            let body_pos = seg.body_positions[pt_idx][raw.body_idx];
            let rel_pos = ship_pos - body_pos;

            // Compute relative velocity at closest approach from finite differences
            let rel_vel = if pt_idx > 0 && pt_idx + 1 < seg.points.len() {
                let dt = seg.times[pt_idx + 1] - seg.times[pt_idx - 1];
                if dt > 0.0 {
                    let prev_rel = seg.points[pt_idx - 1] - seg.body_positions[pt_idx - 1][raw.body_idx];
                    let next_rel = seg.points[pt_idx + 1] - seg.body_positions[pt_idx + 1][raw.body_idx];
                    (next_rel - prev_rel) / dt
                } else {
                    DVec3::ZERO
                }
            } else {
                DVec3::ZERO
            };

            let body_radius = if raw.body_idx < body_radii.len() {
                body_radii[raw.body_idx]
            } else {
                0.0
            };

            let mu = if raw.body_idx < body_masses.len() {
                G * body_masses[raw.body_idx]
            } else {
                return None;
            };

            let elements = orbital_elements(rel_pos, rel_vel, mu);

            let periapsis_altitude = raw.closest_distance - body_radius;

            let capture = if raw.collided || periapsis_altitude < 0.0 {
                CaptureStatus::Impact
            } else if raw.captured || elements.eccentricity < 1.0 {
                CaptureStatus::Captured
            } else if periapsis_altitude < body_radius * 0.1 {
                CaptureStatus::Graze {
                    altitude: periapsis_altitude,
                }
            } else {
                CaptureStatus::Flyby
            };

            Some(Encounter {
                body_idx: raw.body_idx,
                entry_time: raw.entry_time,
                exit_time: raw.exit_time,
                closest_approach: raw.closest_distance,
                closest_time: raw.closest_time,
                relative_velocity: rel_vel.length(),
                capture,
                periapsis_altitude,
                eccentricity: elements.eccentricity,
                inclination: elements.inclination,
            })
        })
        .collect()
}

/// Find the closest approach to each body across all trail segments.
pub fn find_closest_approaches(
    segments: &[TrailSegment],
    ship_idx: usize,
    body_radii: &[f64],
) -> Vec<ClosestApproach> {
    let body_count = segments
        .iter()
        .flat_map(|s| s.body_positions.iter())
        .map(|bp| bp.len())
        .max()
        .unwrap_or(0);

    let mut best: Vec<Option<ClosestApproach>> = vec![None; body_count];

    for seg in segments {
        for (i, (ship_pos, body_positions)) in
            seg.points.iter().zip(seg.body_positions.iter()).enumerate()
        {
            for (body_idx, body_pos) in body_positions.iter().enumerate() {
                if body_idx == ship_idx {
                    continue;
                }
                let dist = (*ship_pos - *body_pos).length();
                let body_radius = if body_idx < body_radii.len() {
                    body_radii[body_idx]
                } else {
                    0.0
                };

                let is_closer = match &best[body_idx] {
                    None => true,
                    Some(prev) => dist < prev.distance,
                };

                if is_closer {
                    best[body_idx] = Some(ClosestApproach {
                        body_idx,
                        position: *ship_pos,
                        body_position: *body_pos,
                        distance: dist,
                        time: seg.times[i],
                        is_collision: dist < body_radius,
                    });
                }
            }

            // Interpolation check
            if i > 0 {
                let prev_ship = seg.points[i - 1];
                let prev_bodies = &seg.body_positions[i - 1];

                for (body_idx, body_pos) in body_positions.iter().enumerate() {
                    if body_idx == ship_idx || body_idx >= prev_bodies.len() {
                        continue;
                    }
                    let prev_body = prev_bodies[body_idx];

                    let ship_delta = *ship_pos - prev_ship;
                    let body_delta = *body_pos - prev_body;
                    let rel_delta = ship_delta - body_delta;
                    let rel_start = prev_ship - prev_body;

                    let a = rel_delta.length_squared();
                    if a < 1e-30 {
                        continue;
                    }
                    let b = 2.0 * rel_start.dot(rel_delta);
                    let t_min = (-b / (2.0 * a)).clamp(0.0, 1.0);

                    if t_min > 0.01 && t_min < 0.99 {
                        let interp_ship = prev_ship + ship_delta * t_min;
                        let interp_body = prev_body + body_delta * t_min;
                        let interp_dist = (interp_ship - interp_body).length();

                        let body_radius = if body_idx < body_radii.len() {
                            body_radii[body_idx]
                        } else {
                            0.0
                        };

                        let is_closer = match &best[body_idx] {
                            None => true,
                            Some(prev) => interp_dist < prev.distance,
                        };

                        if is_closer {
                            let time = seg.times[i - 1]
                                + (seg.times[i] - seg.times[i - 1]) * t_min;
                            best[body_idx] = Some(ClosestApproach {
                                body_idx,
                                position: interp_ship,
                                body_position: interp_body,
                                distance: interp_dist,
                                time,
                                is_collision: interp_dist < body_radius,
                            });
                        }
                    }
                }
            }
        }
    }

    best.into_iter().flatten().collect()
}
