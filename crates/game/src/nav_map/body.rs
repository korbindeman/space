use bevy::prelude::*;
use bevy::math::DVec3;

use crate::sim::*;
use super::camera::{CameraFocus, OrbitCamera};
use super::prediction::PredictionCache;
use super::{RENDER_SCALE, MARKER_RADIUS};

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

/// Ghost entity shown at an encounter body's predicted future position.
/// Uses the real body's mesh (shared handle) so visuals stay in sync.
#[derive(Component)]
pub struct EncounterGhost {
    pub body_idx: usize,
    pub body_entity: Entity,
    /// Which encounter this ghost represents (index into PredictionResult.encounters).
    pub encounter_index: usize,
}

/// Marker for the 3D sphere mesh child of a ghost.
#[derive(Component)]
struct GhostMesh;

/// Marker for the flat circle icon child of a ghost.
#[derive(Component)]
struct GhostIcon;

/// Marker for the label child of a ghost.
#[derive(Component)]
struct GhostLabel;

// --- Resources ---

/// Per-body visual assets for ghost spawning. Mesh handles are shared with real bodies.
struct GhostBodyAssets {
    sphere_mesh: Handle<Mesh>,
    ghost_material: Handle<StandardMaterial>,
    icon_mesh: Handle<Mesh>,
    ghost_icon_material: Handle<StandardMaterial>,
    name: String,
    color: Color,
    radius: f64,
}

/// Stores ghost visual assets indexed by sim body index. Built once at setup.
#[derive(Resource, Default)]
pub(crate) struct GhostAssets {
    bodies: Vec<Option<GhostBodyAssets>>,
}

/// Tracks dynamically spawned ghost entities for cleanup.
#[derive(Resource, Default)]
struct ActiveGhosts {
    entities: Vec<Entity>,
    /// Prediction generation when ghosts were last rebuilt.
    last_generation: u64,
}

// --- Plugin ---

pub struct BodyPlugin;

impl Plugin for BodyPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GhostAssets>()
            .init_resource::<ActiveGhosts>()
            .add_systems(Startup, setup_visuals.after(crate::sim::setup))
            .add_systems(Update, (
                sync_transforms,
                update_encounter_ghosts,
                sync_encounter_ghost_transforms.after(update_encounter_ghosts),
            ));
    }
}

// --- Systems ---

/// Attach visual children (meshes, icons, labels) to entities already spawned by sim::setup.
/// Also builds GhostAssets with shared mesh handles for ghost spawning.
pub(crate) fn setup_visuals(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut ghost_assets: ResMut<GhostAssets>,
    bodies: Query<(Entity, &SimBody, &CelestialBody)>,
    ships: Query<Entity, With<Ship>>,
) {
    for (entity, sim_body, body) in &bodies {
        let render_radius = (body.radius * RENDER_SCALE) as f32;
        let visual_radius = render_radius.max(0.01);

        // Sphere mesh — shared between real body and future ghosts
        let sphere_mesh = meshes.add(Sphere::new(visual_radius));
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

        // Icon mesh — shared between real body and future ghosts
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
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(material),
            BodyMesh,
        ));

        entity_commands.with_child((
            Mesh3d(icon_mesh.clone()),
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

        // Build ghost materials (semi-transparent versions using the same meshes)
        let ghost_color = body.color.with_alpha(0.35);
        let ghost_material = materials.add(StandardMaterial {
            base_color: ghost_color,
            alpha_mode: AlphaMode::Blend,
            ..default()
        });
        let ghost_icon_material = materials.add(StandardMaterial {
            base_color: ghost_color,
            emissive: {
                let lin: LinearRgba = ghost_color.into();
                (lin * 1.5).into()
            },
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            cull_mode: None,
            ..default()
        });

        // Store shared mesh handles + ghost materials for dynamic ghost spawning
        let idx = sim_body.0;
        while ghost_assets.bodies.len() <= idx {
            ghost_assets.bodies.push(None);
        }
        ghost_assets.bodies[idx] = Some(GhostBodyAssets {
            sphere_mesh,
            ghost_material,
            icon_mesh,
            ghost_icon_material,
            name: body.name.clone(),
            color: ghost_color,
            radius: body.radius,
        });
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
    camera_q: Query<&Transform, (With<Camera3d>, Without<SimBody>, Without<BodyIcon>, Without<BodyMesh>, Without<BodyLabel>, Without<super::camera::OverlayCamera>)>,
    mut bodies: Query<(&SimBody, &mut Transform, Option<&CelestialBody>, Option<&Children>, Option<&Ship>), (Without<BodyIcon>, Without<BodyMesh>, Without<BodyLabel>, Without<EncounterGhost>)>,
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
            let use_icon = render_radius < icon_radius;

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

/// Spawn/despawn ghost entities when the prediction changes.
fn update_encounter_ghosts(
    mut commands: Commands,
    cache: Res<PredictionCache>,
    ghost_assets: Res<GhostAssets>,
    mut active_ghosts: ResMut<ActiveGhosts>,
    body_query: Query<(Entity, &SimBody), With<CelestialBody>>,
) {
    // Only rebuild when prediction changes
    if cache.generation == active_ghosts.last_generation {
        return;
    }
    active_ghosts.last_generation = cache.generation;

    // Despawn old ghosts
    for entity in active_ghosts.entities.drain(..) {
        commands.entity(entity).despawn();
    }

    let Some(ref result) = cache.result else { return };

    // Spawn a ghost for each encounter
    for (enc_idx, enc) in result.encounters.iter().enumerate() {
        let Some(Some(assets)) = ghost_assets.bodies.get(enc.body_idx) else {
            continue;
        };
        let Some((body_entity, _)) = body_query.iter().find(|(_, sb)| sb.0 == enc.body_idx) else {
            continue;
        };

        let ghost = commands.spawn((
            EncounterGhost {
                body_idx: enc.body_idx,
                body_entity,
                encounter_index: enc_idx,
            },
            Transform::default(),
            Visibility::Hidden,
        )).with_children(|parent| {
            // Sphere mesh (same handle as real body)
            parent.spawn((
                Mesh3d(assets.sphere_mesh.clone()),
                MeshMaterial3d(assets.ghost_material.clone()),
                GhostMesh,
            ));
            // Icon circle (same handle as real body)
            parent.spawn((
                Mesh3d(assets.icon_mesh.clone()),
                MeshMaterial3d(assets.ghost_icon_material.clone()),
                Transform::default(),
                Visibility::Hidden,
                GhostIcon,
            ));
            // Label
            parent.spawn((
                Text2d::new(&assets.name),
                TextFont {
                    font_size: 14.0,
                    ..default()
                },
                TextColor(assets.color),
                Transform::from_translation(Vec3::new(0.0, 1.5, 0.0)),
                Visibility::Hidden,
                GhostLabel,
            ));
        }).id();

        active_ghosts.entities.push(ghost);
    }
}

/// Position ghost entities and toggle icon/mesh visibility based on camera distance.
fn sync_encounter_ghost_transforms(
    cache: Res<PredictionCache>,
    trail_frame: Res<TrailFrame>,
    orbit_cam: Res<OrbitCamera>,
    ghost_assets: Res<GhostAssets>,
    camera_q: Query<&Transform, (With<Camera3d>, Without<EncounterGhost>, Without<GhostMesh>, Without<GhostIcon>, Without<GhostLabel>, Without<super::camera::OverlayCamera>)>,
    mut ghosts: Query<(&EncounterGhost, &mut Transform, &mut Visibility, Option<&Children>), (Without<GhostMesh>, Without<GhostIcon>, Without<GhostLabel>)>,
    mut ghost_meshes: Query<&mut Visibility, (With<GhostMesh>, Without<EncounterGhost>, Without<GhostIcon>, Without<GhostLabel>)>,
    mut ghost_icons: Query<(&mut Transform, &mut Visibility), (With<GhostIcon>, Without<EncounterGhost>, Without<GhostMesh>, Without<GhostLabel>)>,
    mut ghost_labels: Query<(&mut Transform, &mut Visibility), (With<GhostLabel>, Without<EncounterGhost>, Without<GhostMesh>, Without<GhostIcon>)>,
) {
    let Some(ref result) = cache.result else { return };

    let cam_render_dist = (orbit_cam.distance * RENDER_SCALE) as f32;
    let icon_radius = cam_render_dist * MARKER_RADIUS;
    let cam_rotation = camera_q
        .single()
        .map(|t| t.rotation)
        .unwrap_or(Quat::IDENTITY);

    for (ghost, mut transform, mut visibility, children) in &mut ghosts {
        let Some(enc) = result.encounters.get(ghost.encounter_index) else {
            *visibility = Visibility::Hidden;
            continue;
        };
        if enc.body_idx != ghost.body_idx {
            *visibility = Visibility::Hidden;
            continue;
        }

        // Look up body position at encounter time from trail segment data
        let Some(seg) = result.segments.get(enc.closest_segment_idx) else {
            *visibility = Visibility::Hidden;
            continue;
        };
        let pt_idx = enc.closest_point_idx;
        if pt_idx >= seg.body_positions.len()
            || enc.body_idx >= seg.body_positions[pt_idx].len()
            || trail_frame.index >= seg.body_positions[pt_idx].len()
        {
            *visibility = Visibility::Hidden;
            continue;
        }

        let body_pos = seg.body_positions[pt_idx][enc.body_idx];
        let frame_pos = seg.body_positions[pt_idx][trail_frame.index];
        let render_pos = ((body_pos - frame_pos) * RENDER_SCALE).as_vec3() + trail_frame.offset;

        *visibility = Visibility::Inherited;
        transform.translation = render_pos;

        // Toggle between sphere mesh and icon based on camera distance (same as real bodies)
        let render_radius = ghost_assets.bodies.get(enc.body_idx)
            .and_then(|a| a.as_ref())
            .map(|a| (a.radius * RENDER_SCALE) as f32)
            .unwrap_or(0.0);
        let use_icon = render_radius < icon_radius;

        if let Some(children) = children {
            for child in children.iter() {
                if let Ok(mut mesh_vis) = ghost_meshes.get_mut(child) {
                    *mesh_vis = if use_icon { Visibility::Hidden } else { Visibility::Inherited };
                }
                if let Ok((mut icon_tf, mut icon_vis)) = ghost_icons.get_mut(child) {
                    if use_icon {
                        *icon_vis = Visibility::Inherited;
                        icon_tf.rotation = cam_rotation;
                        icon_tf.scale = Vec3::splat(icon_radius);
                    } else {
                        *icon_vis = Visibility::Hidden;
                    }
                }
                if let Ok((mut label_tf, mut label_vis)) = ghost_labels.get_mut(child) {
                    if use_icon {
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
