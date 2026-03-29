use glam::{DMat3, DVec3};

/// Prograde/normal/radial basis for the ship's orbit around `center_pos`.
///
/// Returns a matrix whose columns are:
///   col 0 = prograde  (along velocity relative to center)
///   col 1 = normal    (along angular momentum)
///   col 2 = radial    (outward from center)
///
/// If velocity is zero or parallel to radial, returns an arbitrary valid frame.
pub fn orbital_frame(pos: DVec3, vel: DVec3, center_pos: DVec3, center_vel: DVec3) -> DMat3 {
    let r = pos - center_pos;
    let radial = if r.length_squared() > 0.0 {
        r.normalize()
    } else {
        DVec3::X
    };

    let rel_vel = vel - center_vel;
    let angular_momentum = r.cross(rel_vel);

    let normal = if angular_momentum.length_squared() > 1e-30 {
        angular_momentum.normalize()
    } else {
        // Velocity is zero or parallel to radial — pick arbitrary normal
        let candidate = if radial.dot(DVec3::Y).abs() < 0.9 {
            DVec3::Y
        } else {
            DVec3::Z
        };
        radial.cross(candidate).normalize()
    };

    let prograde = normal.cross(radial);

    DMat3::from_cols(prograde, normal, radial)
}

#[derive(Clone, Debug)]
pub struct OrbitalElements {
    pub semi_major_axis: f64,
    pub eccentricity: f64,
    pub inclination: f64,
    pub longitude_of_ascending_node: f64,
    pub argument_of_periapsis: f64,
    pub true_anomaly: f64,
    pub period: f64,
    pub periapsis: f64,
    pub apoapsis: f64,
    pub specific_energy: f64,
}

/// Classical Keplerian orbital elements from position and velocity vectors.
/// `mu` = G * M_central (gravitational parameter of the central body).
pub fn orbital_elements(pos: DVec3, vel: DVec3, mu: f64) -> OrbitalElements {
    let r = pos;
    let v = vel;
    let r_mag = r.length();
    let v_mag = v.length();

    // Specific orbital energy
    let specific_energy = v_mag * v_mag / 2.0 - mu / r_mag;

    // Semi-major axis
    let a = if specific_energy.abs() < 1e-30 {
        f64::INFINITY // parabolic
    } else {
        -mu / (2.0 * specific_energy)
    };

    // Angular momentum vector
    let h = r.cross(v);
    let h_mag = h.length();

    // Node vector (points toward ascending node)
    let n = DVec3::new(-h.y, h.x, 0.0);
    let n_mag = n.length();

    // Eccentricity vector
    let e_vec = (v.cross(h) / mu) - r.normalize();
    let e = e_vec.length();

    // Inclination
    let inclination = if h_mag > 1e-30 {
        (h.z / h_mag).clamp(-1.0, 1.0).acos()
    } else {
        0.0
    };

    // Longitude of ascending node
    let longitude_of_ascending_node = if n_mag > 1e-30 {
        let mut omega = (n.x / n_mag).clamp(-1.0, 1.0).acos();
        if n.y < 0.0 {
            omega = std::f64::consts::TAU - omega;
        }
        omega
    } else {
        0.0 // equatorial orbit — undefined, use 0
    };

    // Argument of periapsis
    let argument_of_periapsis = if e > 1e-10 && n_mag > 1e-30 {
        let cos_w = n.dot(e_vec) / (n_mag * e);
        let mut w = cos_w.clamp(-1.0, 1.0).acos();
        if e_vec.z < 0.0 {
            w = std::f64::consts::TAU - w;
        }
        w
    } else {
        0.0 // circular or equatorial — undefined, use 0
    };

    // True anomaly
    let true_anomaly = if e > 1e-10 {
        let cos_nu = e_vec.dot(r) / (e * r_mag);
        let mut nu = cos_nu.clamp(-1.0, 1.0).acos();
        if r.dot(v) < 0.0 {
            nu = std::f64::consts::TAU - nu;
        }
        nu
    } else {
        // Circular orbit — measure from ascending node or reference direction
        if n_mag > 1e-30 {
            let cos_nu = n.dot(r) / (n_mag * r_mag);
            let mut nu = cos_nu.clamp(-1.0, 1.0).acos();
            if r.z < 0.0 {
                nu = std::f64::consts::TAU - nu;
            }
            nu
        } else {
            // Circular equatorial — measure from X axis
            let mut nu = (r.x / r_mag).clamp(-1.0, 1.0).acos();
            if r.y < 0.0 {
                nu = std::f64::consts::TAU - nu;
            }
            nu
        }
    };

    // Period (only meaningful for elliptical orbits)
    let period = if a > 0.0 {
        std::f64::consts::TAU * (a.powi(3) / mu).sqrt()
    } else {
        f64::INFINITY
    };

    // Periapsis and apoapsis distances
    let periapsis = a * (1.0 - e);
    let apoapsis = if e < 1.0 { a * (1.0 + e) } else { f64::INFINITY };

    OrbitalElements {
        semi_major_axis: a,
        eccentricity: e,
        inclination,
        longitude_of_ascending_node,
        argument_of_periapsis,
        true_anomaly,
        period,
        periapsis,
        apoapsis,
        specific_energy,
    }
}

/// Hill sphere radius: a * (m_body / (3 * m_parent))^(1/3)
pub fn hill_radius(semi_major_axis: f64, body_mass: f64, parent_mass: f64) -> f64 {
    semi_major_axis * (body_mass / (3.0 * parent_mass)).cbrt()
}
