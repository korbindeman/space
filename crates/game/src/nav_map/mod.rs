use bevy::prelude::*;

pub mod body;
pub mod camera;
pub mod hud;
pub mod maneuver;
pub mod prediction;
pub mod target;
pub mod trajectory;

// --- Shared constants ---

/// Render scale: sim meters to render units.
pub const RENDER_SCALE: f64 = 1e-6;

/// Radius of screen-stable markers (body icons, ship, maneuver nodes) as a fraction of camera distance.
pub const MARKER_RADIUS: f32 = 0.006;

/// Body colors for trail segments and icons.
pub const BODY_COLORS: &[Color] = &[
    Color::srgb(1.0, 0.95, 0.3), // Sun - yellow
    Color::srgb(0.3, 0.5, 1.0),  // Earth - blue
    Color::srgb(0.6, 0.6, 0.6),  // Moon - grey
    Color::srgb(0.0, 1.0, 0.5),  // Ship - green
];

// --- Shared helpers ---

pub fn format_distance(meters: f64) -> String {
    let abs = meters.abs();
    if abs < 1_000.0 {
        format!("{:.0} m", meters)
    } else if abs < 1_000_000.0 {
        format!("{:.1} km", meters / 1_000.0)
    } else if abs < 1e9 {
        format!("{:.0} km", meters / 1_000.0)
    } else {
        format!("{:.3} AU", meters / 1.496e11)
    }
}

pub fn format_duration(seconds: f64) -> String {
    if seconds.is_infinite() || seconds.is_nan() {
        return "\u{221e}".to_string();
    }
    let s = seconds.abs();
    if s < 60.0 {
        format!("{:.0}s", s)
    } else if s < 3600.0 {
        format!("{:.0}m {:.0}s", s / 60.0, s % 60.0)
    } else if s < 86400.0 {
        format!("{:.0}h {:.0}m", s / 3600.0, (s % 3600.0) / 60.0)
    } else {
        format!("{:.1}d", s / 86400.0)
    }
}

// --- Plugin ---

pub struct NavMapPlugin;

impl Plugin for NavMapPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            body::BodyPlugin,
            camera::CameraPlugin,
            prediction::PredictionPlugin,
            maneuver::ManeuverPlugin,
            trajectory::TrajectoryPlugin,
            target::TargetPlugin,
            hud::HudPlugin,
        ));
    }
}
