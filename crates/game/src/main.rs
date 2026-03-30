use bevy::prelude::*;
use bevy::asset::AssetPlugin;
use bevy::window::{PresentMode, WindowMode};

pub mod sim;
pub mod nav_map;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.02, 0.01, 0.04)))
        .add_plugins(DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Sol".into(),
                    mode: WindowMode::BorderlessFullscreen(MonitorSelection::Current),
                    present_mode: PresentMode::AutoNoVsync,
                    ..default()
                }),
                ..default()
            })
            .set(AssetPlugin {
                file_path: "../../assets".to_string(),
                ..default()
            })
        )
        .add_plugins(bevy::prelude::MeshPickingPlugin)
        .add_plugins(bevy_egui::EguiPlugin::default())
        .add_plugins((
            sim::SimPlugin,
            nav_map::NavMapPlugin,
        ))
        .run();
}
