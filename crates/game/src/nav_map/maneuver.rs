use bevy::math::DVec3;
use bevy::picking::hover::HoverMap;
use bevy::picking::prelude::{DragEnd, DragStart, Pickable, Pointer};
use bevy::prelude::*;
use bevy::camera::visibility::RenderLayers;
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiContexts, egui};

use space_sim::*;

use super::camera::*;
use super::prediction::*;
use super::trajectory;
use super::{RENDER_SCALE, MARKER_RADIUS};
use crate::sim::*;

// --- Resources ---

#[derive(Resource, Default)]
pub struct ManeuverPlan {
    pub nodes: Vec<ManeuverNode>,
    pub dirty: bool,
    next_id: u64,
}

impl ManeuverPlan {
    pub fn next_node_id(&mut self) -> NodeId {
        let id = NodeId(self.next_id);
        self.next_id += 1;
        id
    }
}

/// Cached orbital-frame delta-v for the currently selected node.
#[derive(Resource, Default)]
pub struct NodeDeltaV {
    pub prograde: f64,
    pub normal: f64,
    pub radial: f64,
}

/// Currently selected maneuver node.
#[derive(Resource, Default)]
pub struct SelectedNode {
    pub id: Option<NodeId>,
}

/// Node placement mode — toggled with N key.
#[derive(Resource, Default)]
pub struct TrailHover {
    pub active: bool,
    pub snap_time: Option<f64>,
    pub snap_world_pos: Option<Vec3>,
}

/// Node slide state — dragging a node along the trajectory to change its time.
#[derive(Resource, Default)]
struct NodeSlideState {
    active: bool,
}

/// Arrow drag state — written by picking observers, read by the input system.
#[derive(Resource, Default)]
pub struct ArrowDragState {
    /// Which axis is being dragged: 0=prograde, 1=normal, 2=radial. None if not dragging.
    pub active_axis: Option<usize>,
    /// Which polarity arrow is being dragged (true=positive, false=negative).
    pub active_positive: Option<bool>,
    /// Screen-space direction of the axis (for projecting mouse delta to thrust rate).
    pub axis_screen_dir: Vec2,
    /// Screen position at drag start (used to compute displacement for throttle).
    pub drag_origin: Vec2,
    /// Sign of current drag rate: +1 = adding dv in positive direction, -1 = removing.
    pub rate_sign: f32,
}

// --- Events ---

#[derive(Clone)]
pub enum ManeuverEvent {
    PlaceNode { trail_time: f64 },
    AdjustNode { id: NodeId, delta_v: DVec3 },
    SlideNode { id: NodeId, new_time: f64 },
    DeleteNode { id: NodeId },
    SelectNode { id: NodeId },
    DeselectNode,
    WarpToNode { id: NodeId },
}

impl bevy::ecs::message::Message for ManeuverEvent {}

// --- Components ---

/// Marker component for arrow handle entities (visual + hitbox).
#[derive(Component)]
pub struct ArrowHandle {
    pub axis: usize,
    pub positive: bool,
}

#[derive(Component)]
struct ArrowShaft;

#[derive(Component)]
struct ArrowCone;

/// Invisible hitbox for picking — a large sphere around each arrow.
#[derive(Component)]
struct ArrowHitbox;

/// Per-arrow material handle and base color for dynamic opacity adjustment.
#[derive(Component)]
struct ArrowVisual {
    material: Handle<StandardMaterial>,
    base_color: LinearRgba,
}

/// Material handle for the slide sphere (for hover highlight).
#[derive(Component)]
struct SphereVisual {
    material: Handle<StandardMaterial>,
}

/// Animated stretch state per arrow: [axis][positive=0/negative=1].
/// Smoothly lerps toward the target stretch each frame.
#[derive(Resource)]
struct ArrowStretchState {
    current: [[f32; 2]; 3],
}

impl Default for ArrowStretchState {
    fn default() -> Self {
        Self { current: [[0.0; 2]; 3] }
    }
}

/// Draggable sphere at the node center for sliding along the trajectory.
#[derive(Component)]
struct NodeSlideSphere;

/// Flat circle marker for an unselected maneuver node.
#[derive(Component)]
struct NodeMarkerDisc {
    node_id: NodeId,
}

/// Flat circle for the snap indicator when placing a node (N key).
#[derive(Component)]
struct SnapIndicator;

// --- Constants ---

const SNAP_THRESHOLD_PX: f32 = 30.0;
const SELECT_THRESHOLD_PX: f32 = 20.0;

/// Arrow dimensions in "screen-stable" units — scaled by camera distance each frame.
const SHAFT_RADIUS: f32 = 0.002;
const CONE_RADIUS: f32 = 0.005;
const CONE_HEIGHT: f32 = 0.008;
const BASE_ARROW_LEN: f32 = 0.04;
/// Hitbox capsule radius — fat enough to grab easily.
const HITBOX_CAPSULE_RADIUS: f32 = 0.008;
/// Slide sphere radius.
const SLIDE_SPHERE_RADIUS: f32 = 0.012;

const ARROW_COLORS: [Color; 3] = [
    Color::srgb(0.0, 1.0, 0.0), // prograde - green
    Color::srgb(0.7, 0.0, 1.0), // normal - purple
    Color::srgb(0.0, 1.0, 1.0), // radial - cyan
];

// --- Plugin ---

pub struct ManeuverPlugin;

impl Plugin for ManeuverPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<ManeuverEvent>()
            .init_resource::<ManeuverPlan>()
            .init_resource::<NodeDeltaV>()
            .init_resource::<SelectedNode>()
            .init_resource::<TrailHover>()
            .init_resource::<ArrowDragState>()
            .init_resource::<NodeSlideState>()
            .init_resource::<ArrowStretchState>()
            .add_systems(Startup, spawn_snap_indicator)
            .add_systems(
                Update,
                (
                    sync_node_delta_v,
                    maneuver_input,
                    manage_arrow_handles,
                    update_arrow_transforms,
                    manage_node_markers,
                    update_snap_indicator,
                    handle_maneuver_events,
                ),
            )
            .add_observer(arrow_drag_start)
            .add_observer(arrow_drag_end)
            .add_observer(slide_sphere_drag_start)
            .add_observer(slide_sphere_drag_end)
            .add_systems(bevy_egui::EguiPrimaryContextPass, node_editor_panel);
    }
}

// --- Systems ---

/// Sync NodeDeltaV when selection changes.
fn sync_node_delta_v(
    selected: Res<SelectedNode>,
    plan: Res<ManeuverPlan>,
    mut node_dv: ResMut<NodeDeltaV>,
) {
    if !selected.is_changed() {
        return;
    }
    if let Some(id) = selected.id {
        if let Some(node) = plan.nodes.iter().find(|n| n.id == id) {
            let dv = node.burn.delta_v_orbital();
            node_dv.prograde = dv.x;
            node_dv.normal = dv.y;
            node_dv.radial = dv.z;
        }
    } else {
        *node_dv = NodeDeltaV::default();
    }
}

/// Main input system for maneuver nodes.
fn maneuver_input(
    input: (
        Res<ButtonInput<KeyCode>>,
        Res<ButtonInput<MouseButton>>,
        Res<Time>,
    ),
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_q: Query<(&Camera, &GlobalTransform), (With<Camera3d>, Without<OverlayCamera>)>,
    cache: Res<PredictionCache>,
    trail_frame: Res<TrailFrame>,
    plan: ResMut<ManeuverPlan>,
    mut hover: ResMut<TrailHover>,
    mut selected: ResMut<SelectedNode>,
    mut node_dv: ResMut<NodeDeltaV>,
    mut drag: ResMut<ArrowDragState>,
    mut slide: ResMut<NodeSlideState>,
    picking: (
        Res<HoverMap>,
        Query<Entity, Or<(With<ArrowHitbox>, With<NodeSlideSphere>)>>,
    ),
    mut writer: bevy::ecs::message::MessageWriter<ManeuverEvent>,
) {
    let (keys, mouse, time) = input;
    let (hover_map, hitboxes) = picking;

    // Check if pointer is currently hovering over any arrow hitbox
    let pointer_on_arrow = hover_map
        .0
        .values()
        .any(|hovers| hovers.keys().any(|e| hitboxes.get(*e).is_ok()));

    // N key: toggle placement mode
    if keys.just_pressed(KeyCode::KeyN) {
        hover.active = !hover.active;
        if !hover.active {
            hover.snap_time = None;
            hover.snap_world_pos = None;
        }
    }

    // Delete selected node
    if keys.just_pressed(KeyCode::Delete) || keys.just_pressed(KeyCode::Backspace) {
        if let Some(id) = selected.id {
            writer.write(ManeuverEvent::DeleteNode { id });
            selected.id = None;
        }
    }

    let Ok(window) = windows.single() else { return };
    let Some(cursor_pos) = window.cursor_position() else {
        return;
    };
    let Ok((camera, cam_transform)) = camera_q.single() else {
        return;
    };
    let Some(ref result) = cache.result else {
        return;
    };
    let frame = trail_frame.index;
    let offset = trail_frame.offset;

    // --- Handle active arrow drag (throttle-style) ---
    if let Some(axis) = drag.active_axis {
        if mouse.pressed(MouseButton::Left) {
            let screen_delta = cursor_pos - drag.drag_origin;
            let displacement = screen_delta.dot(drag.axis_screen_dir);
            let rate = displacement as f64 * 10.0;
            let dt = time.delta_secs_f64();

            // Track rate sign for arrow stretch visual feedback
            drag.rate_sign = if rate.abs() < 0.01 { 0.0 } else { rate.signum() as f32 };

            match axis {
                0 => node_dv.prograde += rate * dt,
                1 => node_dv.normal += rate * dt,
                2 => node_dv.radial += rate * dt,
                _ => {}
            }

            if let Some(id) = selected.id {
                writer.write(ManeuverEvent::AdjustNode {
                    id,
                    delta_v: DVec3::new(node_dv.prograde, node_dv.normal, node_dv.radial),
                });
            }
            return;
        } else {
            drag.active_axis = None;
            drag.active_positive = None;
        }
    }

    // --- Handle node sliding along trajectory ---
    if slide.active {
        if mouse.pressed(MouseButton::Left) {
            if let Some(sel_id) = selected.id {
                let mut best_dist = f32::MAX;
                let mut best_time = None;

                for seg in &result.segments {
                    for i in 0..seg.points.len() {
                        let world = trajectory::frame_relative_pos(
                            seg.points[i], &seg.body_positions[i], frame,
                        ) + offset;
                        let Some(screen) = camera.world_to_viewport(cam_transform, world).ok()
                        else {
                            continue;
                        };
                        let dist = (screen - cursor_pos).length();
                        if dist < best_dist {
                            best_dist = dist;
                            best_time = Some(seg.times[i]);
                        }
                    }
                }

                if let Some(new_time) = best_time {
                    writer.write(ManeuverEvent::SlideNode {
                        id: sel_id,
                        new_time,
                    });
                }
            }
            return;
        } else {
            slide.active = false;
        }
    }

    // --- Placement mode: snap to trail ---
    if hover.active {
        let mut best_dist = f32::MAX;
        let mut best_time = None;
        let mut best_world = None;

        for seg in &result.segments {
            for i in 0..seg.points.len() {
                let world = trajectory::frame_relative_pos(
                    seg.points[i], &seg.body_positions[i], frame,
                ) + offset;
                let Some(screen) = camera.world_to_viewport(cam_transform, world).ok() else {
                    continue;
                };
                let dist = (screen - cursor_pos).length();
                if dist < best_dist {
                    best_dist = dist;
                    best_time = Some(seg.times[i]);
                    best_world = Some(world);
                }
            }
        }

        if best_dist < SNAP_THRESHOLD_PX {
            hover.snap_time = best_time;
            hover.snap_world_pos = best_world;
        } else {
            hover.snap_time = best_time;
            hover.snap_world_pos = best_world;
        }

        if mouse.just_pressed(MouseButton::Left) {
            if let Some(time) = best_time {
                writer.write(ManeuverEvent::PlaceNode { trail_time: time });
                hover.active = false;
                hover.snap_time = None;
                hover.snap_world_pos = None;
            }
        }
        return;
    } else {
        hover.snap_time = None;
        hover.snap_world_pos = None;
    }

    // --- Left click: select/deselect nodes, or start sliding (skip if pointer is on an arrow) ---
    if mouse.just_pressed(MouseButton::Left) && !pointer_on_arrow {
        let mut closest_node_id = None;
        let mut closest_node_dist = SELECT_THRESHOLD_PX;
        for node in &plan.nodes {
            let (burn_start, _) = node.burn.time_window();
            if let Some(world_pos) =
                find_node_position_on_trail(result, burn_start, frame, offset)
            {
                let Some(screen) = camera.world_to_viewport(cam_transform, world_pos).ok() else {
                    continue;
                };
                let dist = (screen - cursor_pos).length();
                if dist < closest_node_dist {
                    closest_node_dist = dist;
                    closest_node_id = Some(node.id);
                }
            }
        }

        if let Some(id) = closest_node_id {
            if selected.id == Some(id) {
                // Already selected — start sliding along trajectory
                slide.active = true;
            } else {
                selected.id = Some(id);
            }
        } else {
            selected.id = None;
        }
    }
}

/// Spawn/despawn solid mesh arrow handles when selection changes.
fn manage_arrow_handles(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    selected: Res<SelectedNode>,
    plan: Res<ManeuverPlan>,
    cache: Res<PredictionCache>,
    trail_frame: Res<TrailFrame>,
    existing_handles: Query<Entity, With<ArrowHandle>>,
    existing_spheres: Query<Entity, With<NodeSlideSphere>>,
) {
    let has_node = selected
        .id
        .and_then(|id| {
            let node = plan.nodes.iter().find(|n| n.id == id)?;
            let result = cache.result.as_ref()?;
            node_world_pos_and_frame(result, node.burn.time_window().0, trail_frame.index)
        })
        .is_some();

    if !has_node {
        for entity in &existing_handles {
            commands.entity(entity).despawn();
        }
        for entity in &existing_spheres {
            commands.entity(entity).despawn();
        }
        return;
    }

    if !existing_handles.is_empty() {
        return; // already spawned
    }

    let hitbox_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.0, 0.0, 0.0, 0.0),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });
    let hitbox_mesh = meshes.add(Capsule3d::new(HITBOX_CAPSULE_RADIUS, BASE_ARROW_LEN));

    for axis in 0..3 {
        let color = ARROW_COLORS[axis];
        let base_bright: LinearRgba = color.into();
        let base_dim = LinearRgba::new(base_bright.red * 0.3, base_bright.green * 0.3, base_bright.blue * 0.3, 1.0);

        let shaft_mesh = meshes.add(Cylinder::new(SHAFT_RADIUS, 1.0));
        let cone_mesh = meshes.add(Cone {
            radius: CONE_RADIUS,
            height: CONE_HEIGHT,
        });

        for positive in [true, false] {
            let base_color = if positive { base_bright } else { base_dim };
            // Each arrow gets its own material so opacity can be adjusted per-frame.
            // AlphaMode::Blend ensures arrows render in the transparent pass, AFTER
            // the opaque slide sphere, so the sphere correctly occludes arrows behind it.
            let arrow_material = materials.add(StandardMaterial {
                base_color: base_color.into(),
                emissive: base_color.into(),
                unlit: true,
                alpha_mode: AlphaMode::Blend,
                ..default()
            });

            let overlay = RenderLayers::layer(OVERLAY_LAYER);
            commands
                .spawn((
                    Mesh3d(hitbox_mesh.clone()),
                    MeshMaterial3d(hitbox_mat.clone()),
                    Transform::default(),
                    ArrowHandle { axis, positive },
                    ArrowHitbox,
                    ArrowVisual { material: arrow_material.clone(), base_color },
                    Pickable::default(),
                    overlay.clone(),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        Mesh3d(shaft_mesh.clone()),
                        MeshMaterial3d(arrow_material.clone()),
                        Transform::from_scale(Vec3::new(1.0, BASE_ARROW_LEN, 1.0)),
                        Pickable::IGNORE,
                        ArrowShaft,
                        overlay.clone(),
                    ));
                    parent.spawn((
                        Mesh3d(cone_mesh.clone()),
                        MeshMaterial3d(arrow_material),
                        Transform::from_translation(Vec3::new(0.0, BASE_ARROW_LEN / 2.0, 0.0)),
                        Pickable::IGNORE,
                        ArrowCone,
                        overlay.clone(),
                    ));
                });
        }
    }

    // Spawn slide sphere at node center
    let sphere_mesh = meshes.add(Sphere::new(SLIDE_SPHERE_RADIUS));
    let sphere_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.8, 0.0),
        emissive: Color::srgb(0.5, 0.4, 0.0).into(),
        unlit: true,
        ..default()
    });
    commands.spawn((
        Mesh3d(sphere_mesh),
        MeshMaterial3d(sphere_mat.clone()),
        Transform::default(),
        NodeSlideSphere,
        SphereVisual { material: sphere_mat },
        RenderLayers::layer(OVERLAY_LAYER),
    ));
}

/// Fixed stretch amount for arrows indicating delta-v direction.
const ARROW_STRETCH: f32 = 0.0075;
/// Lerp speed for arrow stretch animation (per second).
const STRETCH_LERP_SPEED: f32 = 12.0;
/// Hover brightness boost multiplier.
const HOVER_BRIGHTNESS: f32 = 1.8;

/// Update arrow handle transforms each frame. Arrows maintain fixed screen size.
/// Arrows that face the camera fade out and become non-interactable.
/// Hovered arrows brighten. Dragged arrows stretch.
fn update_arrow_transforms(
    cache: Res<PredictionCache>,
    selected: Res<SelectedNode>,
    plan: Res<ManeuverPlan>,
    trail_frame: Res<TrailFrame>,
    orbit_cam: Res<OrbitCamera>,
    drag: Res<ArrowDragState>,
    hover_map: Res<HoverMap>,
    time: Res<Time>,
    mut stretch_state: ResMut<ArrowStretchState>,
    mut material_assets: ResMut<Assets<StandardMaterial>>,
    camera_q: Query<&Transform, (With<Camera3d>, Without<ArrowHitbox>, Without<NodeSlideSphere>, Without<OverlayCamera>, Without<ArrowShaft>, Without<ArrowCone>)>,
    mut hitboxes: Query<
        (Entity, &ArrowHandle, &mut Transform, &ArrowVisual, &mut Pickable, Option<&Children>),
        (With<ArrowHitbox>, Without<NodeSlideSphere>, Without<ArrowShaft>, Without<ArrowCone>),
    >,
    mut children_set: ParamSet<(
        Query<&mut Transform, (With<ArrowShaft>, Without<ArrowHitbox>, Without<ArrowCone>, Without<NodeSlideSphere>)>,
        Query<&mut Transform, (With<ArrowCone>, Without<ArrowHitbox>, Without<ArrowShaft>, Without<NodeSlideSphere>)>,
        Query<(Entity, &mut Transform, &SphereVisual), (With<NodeSlideSphere>, Without<ArrowHitbox>, Without<ArrowShaft>, Without<ArrowCone>)>,
    )>,
) {
    let node_info = selected.id.and_then(|id| {
        let node = plan.nodes.iter().find(|n| n.id == id)?;
        let result = cache.result.as_ref()?;
        node_world_pos_and_frame(result, node.burn.time_window().0, trail_frame.index)
    });

    let Some((base_pos, frame)) = node_info else {
        return;
    };
    let world_pos = base_pos + trail_frame.offset;

    // Camera forward direction (from camera position toward origin)
    let view_dir = camera_q
        .single()
        .map(|t| -t.translation.normalize())
        .unwrap_or(-Vec3::Z);

    // Which entities are hovered?
    let hovered_entities: Vec<Entity> = hover_map.0.values()
        .flat_map(|hovers| hovers.keys().copied())
        .collect();

    let s = (orbit_cam.distance * RENDER_SCALE) as f32;
    let arrow_len = BASE_ARROW_LEN * s;
    let sphere_gap = (SLIDE_SPHERE_RADIUS + HITBOX_CAPSULE_RADIUS) * s;

    let mut shaft_updates: Vec<(Entity, f32)> = Vec::new();
    let mut cone_updates: Vec<(Entity, f32)> = Vec::new();

    for (entity, handle, mut transform, visual, mut pickable, children) in &mut hitboxes {
        let dir = frame.col(handle.axis).normalize();
        let sign = if handle.positive { 1.0 } else { -1.0 };

        // Fade arrows based on camera alignment
        let alignment = dir.dot(view_dir).abs();
        // Picking disabled at 45°, but opacity drops fast right at the threshold
        // so it's clearly non-interactive by the time you can't click it.
        let fade_start = 0.707; // cos(45°)
        let opacity = if alignment < fade_start {
            1.0
        } else {
            // Fast drop to 0.15 within a narrow band, then hold
            let t = ((alignment - fade_start) / 0.2).min(1.0); // full fade in ~12° past threshold
            1.0 - t * 0.85
        };

        // Hover highlight
        let is_hovered = hovered_entities.contains(&entity);
        let is_dragging = drag.active_axis == Some(handle.axis);
        let brightness = if is_hovered || is_dragging { HOVER_BRIGHTNESS } else { 1.0 };

        // Update material
        if let Some(mat) = material_assets.get_mut(&visual.material) {
            let c = visual.base_color * opacity * brightness;
            mat.base_color = LinearRgba::new(c.red, c.green, c.blue, opacity).into();
            mat.emissive = LinearRgba::new(c.red, c.green, c.blue, opacity).into();
        }

        // Disable picking when mostly face-on
        // Disable picking when opacity has dropped to ~30%
        pickable.is_hoverable = opacity > 0.3;

        // Stretch only the exact arrow being dragged, based on immediate drag direction.
        // Positive rate_sign = adding dv in positive axis direction.
        // Positive arrow stretches with positive rate, shrinks with negative.
        // Negative arrow is the opposite.
        let is_this_arrow_dragged = drag.active_axis == Some(handle.axis)
            && drag.active_positive == Some(handle.positive);
        let target_stretch = if is_this_arrow_dragged {
            let dir_sign = if handle.positive { 1.0 } else { -1.0 };
            ARROW_STRETCH * drag.rate_sign * dir_sign
        } else {
            0.0
        };

        // Animate toward target
        let polarity_idx = if handle.positive { 0 } else { 1 };
        let current = &mut stretch_state.current[handle.axis][polarity_idx];
        let dt = time.delta_secs();
        *current += (target_stretch - *current) * (STRETCH_LERP_SPEED * dt).min(1.0);
        let stretch = *current;

        let base = world_pos + dir * sign * sphere_gap;
        let midpoint = base + dir * sign * (arrow_len * 0.5 + stretch * s * 0.5);
        let rotation = Quat::from_rotation_arc(Vec3::Y, dir * sign);
        *transform = Transform {
            translation: midpoint,
            rotation,
            scale: Vec3::splat(s),
        };

        // Collect child updates for stretch
        if let Some(children) = children {
            let shaft_len = BASE_ARROW_LEN + stretch;
            for child in children.iter() {
                shaft_updates.push((child, shaft_len));
                cone_updates.push((child, shaft_len));
            }
        }
    }

    // Apply shaft stretches via ParamSet
    {
        let mut shafts = children_set.p0();
        for (entity, shaft_len) in &shaft_updates {
            if let Ok(mut tf) = shafts.get_mut(*entity) {
                tf.scale = Vec3::new(1.0, *shaft_len, 1.0);
            }
        }
    }
    {
        let mut cones = children_set.p1();
        for (entity, shaft_len) in &cone_updates {
            if let Ok(mut tf) = cones.get_mut(*entity) {
                tf.translation.y = *shaft_len / 2.0;
            }
        }
    }

    // Slide sphere: position + hover highlight
    {
        let mut spheres = children_set.p2();
        for (entity, mut transform, sphere_visual) in &mut spheres {
            let is_hovered = hovered_entities.contains(&entity);
            if let Some(mat) = material_assets.get_mut(&sphere_visual.material) {
                if is_hovered {
                    mat.base_color = Color::srgb(1.0, 1.0, 0.5);
                    mat.emissive = LinearRgba::from(Color::srgb(0.8, 0.7, 0.2)).into();
                } else {
                    mat.base_color = Color::srgb(1.0, 0.8, 0.0);
                    mat.emissive = LinearRgba::from(Color::srgb(0.5, 0.4, 0.0)).into();
                }
            }

            *transform = Transform {
                translation: world_pos,
                scale: Vec3::splat(s),
                ..default()
            };
        }
    }
}

/// System that handles picking-based drag start on arrow hitboxes.
fn arrow_drag_start(
    trigger: On<Pointer<DragStart>>,
    handles: Query<&ArrowHandle, With<ArrowHitbox>>,
    mut drag: ResMut<ArrowDragState>,
    cache: Res<PredictionCache>,
    selected: Res<SelectedNode>,
    plan: Res<ManeuverPlan>,
    trail_frame: Res<TrailFrame>,
    orbit_cam: Res<OrbitCamera>,
    camera_q: Query<(&Camera, &GlobalTransform), (With<Camera3d>, Without<OverlayCamera>)>,
) {
    let event = trigger.event();
    let entity = event.entity;
    let Ok(handle) = handles.get(entity) else {
        return;
    };
    let Ok((camera, cam_transform)) = camera_q.single() else {
        return;
    };

    let node_info = selected.id.and_then(|id| {
        let node = plan.nodes.iter().find(|n| n.id == id)?;
        let result = cache.result.as_ref()?;
        node_world_pos_and_frame(result, node.burn.time_window().0, trail_frame.index)
    });

    let Some((base_pos, frame)) = node_info else {
        return;
    };
    let world_pos = base_pos + trail_frame.offset;
    let s = (orbit_cam.distance * RENDER_SCALE) as f32;
    let arrow_len = BASE_ARROW_LEN * s;
    let sphere_gap = (SLIDE_SPHERE_RADIUS + HITBOX_CAPSULE_RADIUS) * s;
    let dir = frame.col(handle.axis).normalize();
    let positive_tip = world_pos + dir * (sphere_gap + arrow_len);

    let Some(node_screen) = camera.world_to_viewport(cam_transform, world_pos).ok() else {
        return;
    };
    let Some(positive_tip_screen) = camera.world_to_viewport(cam_transform, positive_tip).ok()
    else {
        return;
    };

    drag.active_axis = Some(handle.axis);
    drag.active_positive = Some(handle.positive);
    drag.axis_screen_dir = (positive_tip_screen - node_screen).normalize_or_zero();
    drag.drag_origin = event.pointer_location.position;
}

/// System that handles picking-based drag end.
fn arrow_drag_end(
    trigger: On<Pointer<DragEnd>>,
    handles: Query<&ArrowHandle, With<ArrowHitbox>>,
    mut drag: ResMut<ArrowDragState>,
) {
    if handles.get(trigger.event().entity).is_ok() {
        drag.active_axis = None;
        drag.active_positive = None;
    }
}

/// Start sliding node when dragging the slide sphere.
fn slide_sphere_drag_start(
    trigger: On<Pointer<DragStart>>,
    spheres: Query<(), With<NodeSlideSphere>>,
    mut slide: ResMut<NodeSlideState>,
) {
    if spheres.get(trigger.event().entity).is_ok() {
        slide.active = true;
    }
}

/// Stop sliding node when releasing the slide sphere.
fn slide_sphere_drag_end(
    trigger: On<Pointer<DragEnd>>,
    spheres: Query<(), With<NodeSlideSphere>>,
    mut slide: ResMut<NodeSlideState>,
) {
    if spheres.get(trigger.event().entity).is_ok() {
        slide.active = false;
    }
}

/// Handle maneuver events.
fn handle_maneuver_events(
    mut events: bevy::ecs::message::MessageReader<ManeuverEvent>,
    mut plan: ResMut<ManeuverPlan>,
    mut clock: ResMut<SimClock>,
) {
    for event in events.read() {
        match event.clone() {
            ManeuverEvent::PlaceNode { trail_time } => {
                let id = plan.next_node_id();
                plan.nodes.push(ManeuverNode {
                    id,
                    burn: Box::new(ImpulseBurn {
                        time: trail_time,
                        delta_v: DVec3::ZERO,
                    }),
                });
                plan.dirty = true;
            }
            ManeuverEvent::AdjustNode { id, delta_v } => {
                if let Some(node) = plan.nodes.iter_mut().find(|n| n.id == id) {
                    let time = node.burn.time_window().0;
                    node.burn = Box::new(ImpulseBurn { time, delta_v });
                    plan.dirty = true;
                }
            }
            ManeuverEvent::SlideNode { id, new_time } => {
                if let Some(node) = plan.nodes.iter_mut().find(|n| n.id == id) {
                    let dv = node.burn.delta_v_orbital();
                    node.burn = Box::new(ImpulseBurn {
                        time: new_time,
                        delta_v: dv,
                    });
                    plan.dirty = true;
                }
            }
            ManeuverEvent::DeleteNode { id } => {
                plan.nodes.retain(|n| n.id != id);
                plan.dirty = true;
            }
            ManeuverEvent::WarpToNode { id } => {
                if let Some(node) = plan.nodes.iter().find(|n| n.id == id) {
                    let target_time = node.burn.time_window().0 - 10.0;
                    if target_time > clock.time {
                        clock.time = target_time;
                    }
                    // Resume at 1x
                    clock.warp_index = 1;
                    clock.warp = clock.warp_levels[1];
                }
            }
            _ => {}
        }
    }
}

/// Spawn the snap indicator (hidden by default).
fn spawn_snap_indicator(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mesh = meshes.add(Circle::new(1.0));
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 1.0, 0.0),
        emissive: {
            let lin: LinearRgba = Color::srgb(1.0, 1.0, 0.0).into();
            (lin * 2.0).into()
        },
        unlit: true,
        cull_mode: None,
        ..default()
    });
    commands.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(mat),
        Transform::default(),
        Visibility::Hidden,
        SnapIndicator,
        RenderLayers::layer(OVERLAY_LAYER),
    ));
}

/// Update snap indicator position/visibility.
fn update_snap_indicator(
    hover: Res<TrailHover>,
    orbit_cam: Res<OrbitCamera>,
    camera_q: Query<&Transform, (With<Camera3d>, Without<SnapIndicator>, Without<OverlayCamera>)>,
    mut indicators: Query<(&mut Transform, &mut Visibility), With<SnapIndicator>>,
) {
    let Ok((mut tf, mut vis)) = indicators.single_mut() else {
        return;
    };

    if hover.active {
        if let Some(pos) = hover.snap_world_pos {
            let cam_dist = (orbit_cam.distance * RENDER_SCALE) as f32;
            let cam_rot = camera_q
                .single()
                .map(|t| t.rotation)
                .unwrap_or(Quat::IDENTITY);
            *vis = Visibility::Inherited;
            *tf = Transform {
                translation: pos,
                rotation: cam_rot,
                scale: Vec3::splat(cam_dist * MARKER_RADIUS),
            };
            return;
        }
    }
    *vis = Visibility::Hidden;
}

/// Manage flat circle markers for unselected maneuver nodes.
fn manage_node_markers(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    plan: Res<ManeuverPlan>,
    selected: Res<SelectedNode>,
    cache: Res<PredictionCache>,
    trail_frame: Res<TrailFrame>,
    orbit_cam: Res<OrbitCamera>,
    camera_q: Query<&Transform, (With<Camera3d>, Without<NodeMarkerDisc>, Without<OverlayCamera>)>,
    mut markers: Query<(Entity, &NodeMarkerDisc, &mut Transform, &mut Visibility)>,
) {
    let frame = trail_frame.index;
    let offset = trail_frame.offset;
    let cam_dist = (orbit_cam.distance * RENDER_SCALE) as f32;
    let cam_rot = camera_q
        .single()
        .map(|t| t.rotation)
        .unwrap_or(Quat::IDENTITY);
    let result = cache.result.as_ref();

    // Update existing markers or hide stale ones
    for (entity, marker, mut tf, mut vis) in &mut markers {
        let dominated = selected.id == Some(marker.node_id);
        let node_exists = plan.nodes.iter().any(|n| n.id == marker.node_id);

        if !node_exists || dominated {
            commands.entity(entity).despawn();
            continue;
        }

        let node = plan.nodes.iter().find(|n| n.id == marker.node_id).unwrap();
        let burn_time = node.burn.time_window().0;
        if let Some(Some(world_pos)) =
            result.map(|r| find_node_position_on_trail(r, burn_time, frame, offset))
        {
            *vis = Visibility::Inherited;
            *tf = Transform {
                translation: world_pos,
                rotation: cam_rot,
                scale: Vec3::splat(cam_dist * MARKER_RADIUS),
            };
        } else {
            *vis = Visibility::Hidden;
        }
    }

    // Spawn markers for nodes that don't have one yet
    for node in &plan.nodes {
        if selected.id == Some(node.id) {
            continue;
        }
        let already_exists = markers.iter().any(|(_, m, _, _)| m.node_id == node.id);
        if already_exists {
            continue;
        }

        let mesh = meshes.add(Circle::new(1.0));
        let mat = materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.6, 0.0),
            emissive: {
                let lin: LinearRgba = Color::srgb(0.8, 0.6, 0.0).into();
                (lin * 2.0).into()
            },
            unlit: true,
            cull_mode: None,
            ..default()
        });

        let burn_time = node.burn.time_window().0;
        let world_pos = result
            .and_then(|r| find_node_position_on_trail(r, burn_time, frame, offset))
            .unwrap_or(Vec3::ZERO);

        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(mat),
            Transform {
                translation: world_pos,
                rotation: cam_rot,
                scale: Vec3::splat(cam_dist * MARKER_RADIUS),
            },
            NodeMarkerDisc { node_id: node.id },
            RenderLayers::layer(OVERLAY_LAYER),
        ));
    }
}

/// Bottom panel: Node editor.
fn node_editor_panel(
    mut contexts: EguiContexts,
    mut plan: ResMut<ManeuverPlan>,
    mut selected: ResMut<SelectedNode>,
    mut clock: ResMut<SimClock>,
    ship_config: Res<ShipConfig>,
    mut node_dv: ResMut<NodeDeltaV>,
) {
    let Some(sel_id) = selected.id else { return };
    let Some(node_idx) = plan.nodes.iter().position(|n| n.id == sel_id) else {
        selected.id = None;
        return;
    };

    let burn_time = plan.nodes[node_idx].burn.time_window().0;
    let total_dv = plan.nodes[node_idx].burn.total_delta_v();
    let time_until = burn_time - clock.time;

    let Ok(ctx) = contexts.ctx_mut() else { return };
    egui::TopBottomPanel::bottom("node_editor").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.heading("Maneuver Node");
            ui.separator();
            ui.label(format!("Node #{}", sel_id.0));
            ui.separator();
            ui.label(format!("T{:+.0}s", time_until));
            ui.separator();
            ui.label(format!("Total \u{0394}v: {:.1} m/s", total_dv));

            let burn_duration = if ship_config.thrust_newtons > 0.0 {
                total_dv * (ship_config.dry_mass + ship_config.fuel_mass)
                    / ship_config.thrust_newtons
            } else {
                0.0
            };
            ui.label(format!("Est. burn: {:.1}s", burn_duration));
            ui.separator();

            if ui.button("Delete").clicked() {
                plan.nodes.retain(|n| n.id != sel_id);
                plan.dirty = true;
                selected.id = None;
            }
            if ui.button("Warp to node").clicked() {
                if let Some(node) = plan.nodes.iter().find(|n| n.id == sel_id) {
                    let target_time = node.burn.time_window().0 - 10.0;
                    if target_time > clock.time {
                        clock.time = target_time;
                    }
                    clock.warp_index = 1;
                    clock.warp = clock.warp_levels[1];
                }
            }
        });

        ui.horizontal(|ui| {
            let mut pg = node_dv.prograde as f32;
            let mut nm = node_dv.normal as f32;
            let mut rd = node_dv.radial as f32;

            let changed_pg = ui
                .add(
                    egui::DragValue::new(&mut pg)
                        .speed(1.0)
                        .prefix("P: ")
                        .suffix(" m/s"),
                )
                .changed();
            let changed_nm = ui
                .add(
                    egui::DragValue::new(&mut nm)
                        .speed(1.0)
                        .prefix("N: ")
                        .suffix(" m/s"),
                )
                .changed();
            let changed_rd = ui
                .add(
                    egui::DragValue::new(&mut rd)
                        .speed(1.0)
                        .prefix("R: ")
                        .suffix(" m/s"),
                )
                .changed();

            if changed_pg || changed_nm || changed_rd {
                node_dv.prograde = pg as f64;
                node_dv.normal = nm as f64;
                node_dv.radial = rd as f64;
                if let Some(node) = plan.nodes.iter_mut().find(|n| n.id == sel_id) {
                    let time = node.burn.time_window().0;
                    node.burn = Box::new(ImpulseBurn {
                        time,
                        delta_v: DVec3::new(pg as f64, nm as f64, rd as f64),
                    });
                    plan.dirty = true;
                }
            }
        });
    });
}

// --- Helpers ---

/// Compute node world position and orbital frame from trail data.
fn node_world_pos_and_frame(
    result: &space_prediction::PredictionResult,
    time: f64,
    frame_body: usize,
) -> Option<(Vec3, Mat3)> {
    for seg in &result.segments {
        if seg.times.is_empty() {
            continue;
        }
        let first_time = seg.times[0];
        let last_time = *seg.times.last().unwrap();
        if time < first_time || time > last_time {
            continue;
        }

        let idx = seg
            .times
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                ((**a - time).abs())
                    .partial_cmp(&((**b - time).abs()))
                    .unwrap()
            })
            .map(|(i, _)| i)?;

        let frame_pos = if frame_body < seg.body_positions[idx].len() {
            seg.body_positions[idx][frame_body]
        } else {
            DVec3::ZERO
        };
        let world_pos = ((seg.points[idx] - frame_pos) * RENDER_SCALE).as_vec3();

        // Use exact stored velocities — matches ImpulseBurn.acceleration exactly
        let vel = seg.velocities[idx];
        let dom = seg.dominant_body[idx];
        let center_pos = if dom < seg.body_positions[idx].len() {
            seg.body_positions[idx][dom]
        } else {
            DVec3::ZERO
        };
        let center_vel = if dom < seg.body_velocities[idx].len() {
            seg.body_velocities[idx][dom]
        } else {
            DVec3::ZERO
        };

        let orbital = orbital_frame(seg.points[idx], vel, center_pos, center_vel);
        let render_frame = Mat3::from_cols(
            orbital.col(0).as_vec3(),
            orbital.col(1).as_vec3(),
            orbital.col(2).as_vec3(),
        );

        return Some((world_pos, render_frame));
    }
    None
}

fn find_node_position_on_trail(
    result: &space_prediction::PredictionResult,
    time: f64,
    frame_body: usize,
    offset: Vec3,
) -> Option<Vec3> {
    node_world_pos_and_frame(result, time, frame_body).map(|(pos, _)| pos + offset)
}
