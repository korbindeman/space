use bevy::prelude::*;
use bevy::core_pipeline::Skybox;
use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::picking::prelude::{Click, Pointer};
use bevy::render::render_resource::{TextureViewDescriptor, TextureViewDimension};
use bevy::camera::visibility::RenderLayers;
use bevy_egui::PrimaryEguiContext;

use crate::sim::*;
use super::body::EncounterGhost;
use super::RENDER_SCALE;

/// Render layer for maneuver UI elements (arrows, markers, snap indicator).
/// Rendered by an overlay camera that clears depth but not color, so these
/// elements always appear on top of the world while depth-testing against each other.
pub const OVERLAY_LAYER: usize = 1;

// --- Resources ---

#[derive(Resource)]
pub struct CameraFocus {
    pub entity: Entity,
    pub body_index: Option<usize>,
    pub active_frame: usize,
}

#[derive(Resource, Default)]
struct LastClick {
    entity: Option<Entity>,
    time: f64,
}

/// Camera orbit state.
#[derive(Resource)]
pub struct OrbitCamera {
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f64,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            yaw: 0.0,
            pitch: 0.3,
            distance: 2e7, // 20,000 km
        }
    }
}

// --- Plugin ---

#[derive(Component)]
pub struct OverlayCamera;

#[derive(Resource)]
struct SkyboxHandle(Handle<Image>);

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<OrbitCamera>()
            .init_resource::<LastClick>()
            .add_systems(Startup, setup_camera)
            .add_systems(Update, (
                keyboard_input,
                camera_input,
                update_camera,
                sync_overlay_camera.after(update_camera),
                setup_skybox_cubemap,
            ))
            .add_observer(on_body_clicked);
    }
}

// --- Systems ---

fn setup_camera(
    mut commands: Commands,
    mut egui_settings: ResMut<bevy_egui::EguiGlobalSettings>,
    asset_server: Res<AssetServer>,
) {
    egui_settings.auto_create_primary_context = false;

    let skybox_handle = asset_server.load("skybox.png");
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
        Skybox {
            image: skybox_handle.clone(),
            brightness: 500.0,
            ..default()
        },
        PrimaryEguiContext,
    ));
    commands.insert_resource(SkyboxHandle(skybox_handle));

    // Overlay camera: renders maneuver UI on top of the world.
    // Clears depth (so maneuver elements depth-test against each other, not the world)
    // but not color (world renders underneath).
    commands.spawn((
        Camera3d::default(),
        Camera {
            order: 1,
            clear_color: ClearColorConfig::None,
            ..default()
        },
        Transform::from_xyz(0.0, 5.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
        RenderLayers::layer(OVERLAY_LAYER),
        OverlayCamera,
    ));
}

/// Once the skybox PNG is loaded, reinterpret it as a cubemap (6 faces stacked vertically).
fn setup_skybox_cubemap(
    skybox_handle: Option<Res<SkyboxHandle>>,
    mut images: ResMut<Assets<Image>>,
    mut done: Local<bool>,
) {
    if *done {
        return;
    }
    let Some(skybox_handle) = skybox_handle else { return };
    let Some(image) = images.get_mut(&skybox_handle.0) else { return };

    if image.reinterpret_stacked_2d_as_array(6).is_err() {
        return;
    }
    image.texture_view_descriptor = Some(TextureViewDescriptor {
        dimension: Some(TextureViewDimension::Cube),
        ..default()
    });
    *done = true;
}

/// Handle keyboard input for time control and camera focus.
fn keyboard_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut clock: ResMut<SimClock>,
    mut camera_focus: ResMut<CameraFocus>,
    bodies: Query<(Entity, &SimBody, &CelestialBody)>,
    ship: Query<(Entity, &SimBody), With<Ship>>,
) {
    // Pause/unpause
    if keys.just_pressed(KeyCode::Space) {
        clock.toggle_pause();
    }

    // Warp controls
    if keys.just_pressed(KeyCode::Period) {
        if clock.warp_index + 1 < clock.warp_levels.len() {
            clock.warp_index += 1;
            clock.warp = clock.warp_levels[clock.warp_index];
        }
    }
    if keys.just_pressed(KeyCode::Comma) {
        if clock.warp_index > 0 {
            clock.warp_index -= 1;
            clock.warp = clock.warp_levels[clock.warp_index];
        }
    }

    // Tab: cycle focus
    if keys.just_pressed(KeyCode::Tab) {
        let mut all_entities: Vec<(Entity, usize)> = bodies
            .iter()
            .map(|(e, sb, _)| (e, sb.0))
            .collect();
        if let Ok((e, sb)) = ship.single() {
            all_entities.push((e, sb.0));
        }
        all_entities.sort_by_key(|(_, idx)| *idx);

        if let Some(current_pos) = all_entities.iter().position(|(e, _)| *e == camera_focus.entity) {
            let next = (current_pos + 1) % all_entities.len();
            camera_focus.entity = all_entities[next].0;
            camera_focus.body_index = Some(all_entities[next].1);
            camera_focus.active_frame = all_entities[next].1;
        }
    }
}

/// Handle mouse input for orbit camera.
fn camera_input(
    mut orbit_cam: ResMut<OrbitCamera>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mut mouse_motion: bevy::ecs::message::MessageReader<MouseMotion>,
    mut scroll: bevy::ecs::message::MessageReader<MouseWheel>,
) {
    if mouse_button.pressed(MouseButton::Middle) || mouse_button.pressed(MouseButton::Right) {
        for ev in mouse_motion.read() {
            orbit_cam.yaw -= ev.delta.x * 0.005;
            orbit_cam.pitch += ev.delta.y * 0.005;
            orbit_cam.pitch = orbit_cam.pitch.clamp(-1.4, 1.4);
        }
    } else {
        mouse_motion.clear();
    }

    for ev in scroll.read() {
        let amount = match ev.unit {
            MouseScrollUnit::Line => ev.y * 0.1,
            MouseScrollUnit::Pixel => ev.y * 0.001,
        };
        orbit_cam.distance *= (-amount as f64).exp();
        orbit_cam.distance = orbit_cam.distance.clamp(1e3, 1e13);
    }
}

/// Update camera transform from orbit camera state.
fn update_camera(
    orbit_cam: Res<OrbitCamera>,
    mut camera_q: Query<&mut Transform, (With<Camera3d>, Without<OverlayCamera>)>,
) {
    let Ok(mut transform) = camera_q.single_mut() else {
        return;
    };

    let render_dist = (orbit_cam.distance * RENDER_SCALE) as f32;
    let direction = Vec3::new(
        orbit_cam.yaw.cos() * orbit_cam.pitch.cos(),
        orbit_cam.pitch.sin(),
        orbit_cam.yaw.sin() * orbit_cam.pitch.cos(),
    )
    .normalize();

    transform.translation = direction * render_dist;
    transform.look_at(Vec3::ZERO, Vec3::Y);
}

/// Keep the overlay camera in sync with the main camera.
fn sync_overlay_camera(
    main: Query<&Transform, (With<Camera3d>, Without<OverlayCamera>)>,
    mut overlay: Query<&mut Transform, With<OverlayCamera>>,
) {
    let Ok(main_tf) = main.single() else { return };
    let Ok(mut overlay_tf) = overlay.single_mut() else { return };
    *overlay_tf = *main_tf;
}

/// Double-click on a body, ship, or encounter ghost to focus the camera on it.
fn on_body_clicked(
    trigger: On<Pointer<Click>>,
    time: Res<Time>,
    mut last_click: ResMut<LastClick>,
    mut camera_focus: ResMut<CameraFocus>,
    bodies: Query<&SimBody, Or<(With<CelestialBody>, With<Ship>)>>,
    ghosts: Query<&EncounterGhost>,
) {
    let entity = trigger.event_target();

    // Resolve the focus target: real body/ship, or encounter ghost mapping to its real body
    let (focus_entity, body_idx) = if let Ok(sim_body) = bodies.get(entity) {
        (entity, sim_body.0)
    } else if let Ok(ghost) = ghosts.get(entity) {
        (ghost.body_entity, ghost.body_idx)
    } else {
        return;
    };

    let now = time.elapsed_secs_f64();
    let is_double_click = last_click.entity == Some(entity) && (now - last_click.time) < 0.4;

    last_click.entity = Some(entity);
    last_click.time = now;

    if is_double_click {
        camera_focus.entity = focus_entity;
        camera_focus.body_index = Some(body_idx);
        camera_focus.active_frame = body_idx;
    }
}
