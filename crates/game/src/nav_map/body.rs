use bevy::prelude::*;
use bevy::math::DVec3;

use crate::sim::*;
use super::camera::{CameraFocus, OrbitCamera};
use super::{RENDER_SCALE, ICON_THRESHOLD, MARKER_RADIUS};

// --- Components ---

/// Marker for the 3D sphere mesh child of a celestial body.
#[derive(Component)]
struct BodyMesh;

/// Marker for the flat circle icon child entity of a celestial body.
#[derive(Component)]
struct BodyIcon;

/// Marker for the body name label child entity.
#[derive(Component)]
struct BodyLabel;

// --- Plugin ---

pub struct BodyPlugin;

impl Plugin for BodyPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_visuals.after(crate::sim::setup))
            .add_systems(Update, sync_transforms);
    }
}

// --- Systems ---

/// Attach visual children (meshes, icons, labels) to entities already spawned by sim::setup.
pub fn setup_visuals(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    bodies: Query<(Entity, &CelestialBody)>,
    ships: Query<Entity, With<Ship>>,
) {
    for (entity, body) in &bodies {
        let render_radius = (body.radius * RENDER_SCALE) as f32;
        let visual_radius = render_radius.max(0.01);

        let mesh = meshes.add(Sphere::new(visual_radius));
        let material = if body.name == "Sun" {
            materials.add(StandardMaterial {
                base_color: body.color,
                emissive: {
                    let lin: LinearRgba = body.color.into();
                    (lin * 8.0).into()
                },
                ..default()
            })
        } else {
            materials.add(StandardMaterial {
                base_color: body.color,
                ..default()
            })
        };

        let icon_mesh = meshes.add(Circle::new(1.0));
        let icon_material = materials.add(StandardMaterial {
            base_color: body.color,
            emissive: {
                let lin: LinearRgba = body.color.into();
                (lin * 2.0).into()
            },
            unlit: true,
            cull_mode: None,
            ..default()
        });

        let mut entity_commands = commands.entity(entity);

        entity_commands.with_child((
            Mesh3d(mesh),
            MeshMaterial3d(material),
            BodyMesh,
        ));

        entity_commands.with_child((
            Mesh3d(icon_mesh),
            MeshMaterial3d(icon_material),
            Transform::default(),
            Visibility::Hidden,
            BodyIcon,
        ));

        entity_commands.with_child((
            Text2d::new(&body.name),
            TextFont {
                font_size: 14.0,
                ..default()
            },
            TextColor(body.color),
            Transform::from_translation(Vec3::new(0.0, 1.5, 0.0)),
            Visibility::Hidden,
            BodyLabel,
        ));

        if body.name == "Sun" {
            entity_commands.with_child(PointLight {
                intensity: 10_000_000.0,
                range: 100_000.0,
                shadows_enabled: true,
                ..default()
            });
        }
    }

    // Ship gets a circle icon + label (same as body icons)
    for entity in &ships {
        let ship_color = Color::srgb(0.0, 1.0, 0.5);
        let ship_mesh = meshes.add(Circle::new(1.0));
        let ship_material = materials.add(StandardMaterial {
            base_color: ship_color,
            emissive: ship_color.into(),
            unlit: true,
            cull_mode: None,
            ..default()
        });

        let mut entity_commands = commands.entity(entity);
        // Ship's main mesh is the icon circle (always visible as flat marker)
        entity_commands.insert((
            Mesh3d(ship_mesh),
            MeshMaterial3d(ship_material),
        ));

        entity_commands.with_child((
            Text2d::new("Ship"),
            TextFont {
                font_size: 14.0,
                ..default()
            },
            TextColor(ship_color),
            Transform::from_translation(Vec3::new(0.0, 1.5, 0.0)),
            Visibility::Inherited,
            BodyLabel,
        ));
    }
}

/// Sync entity Transforms to camera-relative positions. Toggle between 3D mesh and flat icon.
fn sync_transforms(
    physics: Res<PhysicsState>,
    camera_focus: Res<CameraFocus>,
    orbit_cam: Res<OrbitCamera>,
    camera_q: Query<&Transform, (With<Camera3d>, Without<SimBody>, Without<BodyIcon>, Without<BodyMesh>, Without<BodyLabel>)>,
    mut bodies: Query<(&SimBody, &mut Transform, Option<&CelestialBody>, Option<&Children>, Option<&Ship>), (Without<BodyIcon>, Without<BodyMesh>, Without<BodyLabel>)>,
    mut icons: Query<(&mut Transform, &mut Visibility), (With<BodyIcon>, Without<BodyMesh>, Without<SimBody>, Without<BodyLabel>)>,
    mut meshes: Query<&mut Visibility, (With<BodyMesh>, Without<BodyIcon>, Without<SimBody>, Without<BodyLabel>)>,
    mut labels: Query<(&mut Transform, &mut Visibility), (With<BodyLabel>, Without<BodyIcon>, Without<BodyMesh>, Without<SimBody>)>,
) {
    let focus_idx = camera_focus.active_frame;
    let camera_sim_pos = if focus_idx < physics.state.positions.len() {
        physics.state.positions[focus_idx]
    } else {
        DVec3::ZERO
    };

    let cam_render_dist = (orbit_cam.distance * RENDER_SCALE) as f32;
    let threshold = cam_render_dist * ICON_THRESHOLD;
    let icon_radius = cam_render_dist * MARKER_RADIUS;

    let cam_rotation = camera_q
        .single()
        .map(|t| t.rotation)
        .unwrap_or(Quat::IDENTITY);

    for (sim_body, mut transform, celestial, children, is_ship) in &mut bodies {
        let sim_pos = physics.state.positions[sim_body.0];
        let relative = (sim_pos - camera_sim_pos) * RENDER_SCALE;
        transform.translation = relative.as_vec3();

        if is_ship.is_some() {
            transform.rotation = cam_rotation;
            transform.scale = Vec3::splat(icon_radius);
        }

        if let Some(body) = celestial {
            let render_radius = (body.radius * RENDER_SCALE) as f32;
            let use_icon = render_radius < threshold;

            if let Some(children) = children {
                for child in children.iter() {
                    if let Ok((mut icon_tf, mut icon_vis)) = icons.get_mut(child) {
                        if use_icon {
                            *icon_vis = Visibility::Inherited;
                            icon_tf.rotation = cam_rotation;
                            icon_tf.scale = Vec3::splat(icon_radius);
                        } else {
                            *icon_vis = Visibility::Hidden;
                        }
                    }
                    if let Ok(mut mesh_vis) = meshes.get_mut(child) {
                        if use_icon {
                            *mesh_vis = Visibility::Hidden;
                        } else {
                            *mesh_vis = Visibility::Inherited;
                        }
                    }
                    if let Ok((mut label_tf, mut label_vis)) = labels.get_mut(child) {
                        if use_icon || is_ship.is_some() {
                            *label_vis = Visibility::Inherited;
                            label_tf.rotation = cam_rotation;
                            label_tf.scale = Vec3::splat(icon_radius);
                        } else {
                            *label_vis = Visibility::Hidden;
                        }
                    }
                }
            }
        }
    }
}
