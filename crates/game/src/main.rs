use bevy::prelude::*;

pub mod sim;
pub mod camera;
pub mod prediction;
pub mod maneuver;
pub mod trail;
pub mod target;
pub mod hud;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Sol".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(bevy::prelude::MeshPickingPlugin)
        .add_plugins(bevy_egui::EguiPlugin::default())
        .add_plugins((
            sim::SimPlugin,
            camera::CameraPlugin,
            prediction::PredictionPlugin,
            maneuver::ManeuverPlugin,
            trail::TrailPlugin,
            target::TargetPlugin,
            hud::HudPlugin,
        ))
        .run();
}
