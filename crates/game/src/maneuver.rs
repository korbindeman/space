use bevy::prelude::*;
use bevy::math::DVec3;
use bevy::picking::prelude::{Pickable, Pointer, DragStart, DragEnd};
use bevy::picking::hover::HoverMap;
use bevy::window::PrimaryWindow;
use bevy_egui::{egui, EguiContexts};

use space_sim::*;

use crate::sim::*;
use crate::camera::*;
use crate::prediction::*;
use crate::trail;

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

/// Arrow drag state — written by picking observers, read by the input system.
#[derive(Resource, Default)]
pub struct ArrowDragState {
    /// Which axis is being dragged: 0=prograde, 1=normal, 2=radial. None if not dragging.
    pub active_axis: Option<usize>,
    /// Screen-space direction of the axis (for projecting mouse delta to thrust rate).
    pub axis_screen_dir: Vec2,
    /// Screen position at drag start (used to compute displacement for throttle).
    pub drag_origin: Vec2,
}

// --- Events ---

#[derive(Clone)]
pub enum ManeuverEvent {
    PlaceNode { trail_time: f64 },
    AdjustNode { id: NodeId, delta_v: DVec3 },
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

// --- Constants ---

const SNAP_THRESHOLD_PX: f32 = 30.0;
const SELECT_THRESHOLD_PX: f32 = 20.0;

/// Arrow dimensions in "screen-stable" units — scaled by camera distance each frame.
const SHAFT_RADIUS: f32 = 0.004;
const CONE_RADIUS: f32 = 0.008;
const CONE_HEIGHT: f32 = 0.012;
const BASE_ARROW_LEN: f32 = 0.06;
/// Hitbox capsule radius — fat enough to grab easily.
const HITBOX_CAPSULE_RADIUS: f32 = 0.015;

const ARROW_COLORS: [Color; 3] = [
    Color::srgb(0.0, 1.0, 0.0),   // prograde - green
    Color::srgb(0.7, 0.0, 1.0),   // normal - purple
    Color::srgb(0.0, 1.0, 1.0),   // radial - cyan
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
            .add_systems(Update, (
                sync_node_delta_v,
                maneuver_input,
                manage_arrow_handles,
                update_arrow_transforms,
                render_maneuver_nodes,
                handle_maneuver_events,
            ))
            .add_observer(arrow_drag_start)
            .add_observer(arrow_drag_end)
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
    input: (Res<ButtonInput<KeyCode>>, Res<ButtonInput<MouseButton>>, Res<Time>),
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_q: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    cache: Res<PredictionCache>,
    sim: (Res<CameraFocus>, Res<LiveDominantBody>, Res<PhysicsState>),
    plan: ResMut<ManeuverPlan>,
    mut hover: ResMut<TrailHover>,
    mut selected: ResMut<SelectedNode>,
    mut node_dv: ResMut<NodeDeltaV>,
    mut drag: ResMut<ArrowDragState>,
    picking: (Res<HoverMap>, Query<Entity, With<ArrowHitbox>>),
    mut writer: bevy::ecs::message::MessageWriter<ManeuverEvent>,
) {
    let (keys, mouse, time) = input;
    let (camera_focus, live_dom, physics) = sim;
    let (hover_map, hitboxes) = picking;

    // Check if pointer is currently hovering over any arrow hitbox
    let pointer_on_arrow = hover_map.0.values().any(|hovers| {
        hovers.keys().any(|e| hitboxes.get(*e).is_ok())
    });

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
    let Some(cursor_pos) = window.cursor_position() else { return };
    let Ok((camera, cam_transform)) = camera_q.single() else { return };
    let Some(ref result) = cache.result else { return };

    let trail_frame = live_dom.index;
    let offset = trail::trail_frame_offset(&physics, trail_frame, camera_focus.active_frame);

    // --- Handle active arrow drag (throttle-style) ---
    if let Some(axis) = drag.active_axis {
        if mouse.pressed(MouseButton::Left) {
            let screen_delta = cursor_pos - drag.drag_origin;
            let displacement = screen_delta.dot(drag.axis_screen_dir);
            let rate = displacement as f64 * 10.0;
            let dt = time.delta_secs_f64();

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
        }
    }

    // --- Placement mode: snap to trail ---
    if hover.active {
        let mut best_dist = f32::MAX;
        let mut best_time = None;
        let mut best_world = None;

        for seg in &result.segments {
            for i in 0..seg.points.len() {
                let frame_pos = if trail_frame < seg.body_positions[i].len() {
                    seg.body_positions[i][trail_frame]
                } else {
                    DVec3::ZERO
                };
                let world = ((seg.points[i] - frame_pos) * RENDER_SCALE).as_vec3() + offset;
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

    // --- Left click: select/deselect nodes (skip if pointer is on an arrow) ---
    if mouse.just_pressed(MouseButton::Left) && !pointer_on_arrow {
        let mut closest_node_id = None;
        let mut closest_node_dist = SELECT_THRESHOLD_PX;
        for node in &plan.nodes {
            let (burn_start, _) = node.burn.time_window();
            if let Some(world_pos) = find_node_position_on_trail(result, burn_start, trail_frame, offset) {
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
            selected.id = Some(id);
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
    live_dom: Res<LiveDominantBody>,
    existing_handles: Query<Entity, With<ArrowHandle>>,
) {
    let trail_frame = live_dom.index;

    let has_node = selected.id.and_then(|id| {
        let node = plan.nodes.iter().find(|n| n.id == id)?;
        let result = cache.result.as_ref()?;
        node_world_pos_and_frame(result, node.burn.time_window().0, trail_frame)
    }).is_some();

    if !has_node {
        for entity in &existing_handles {
            commands.entity(entity).despawn();
        }
        return;
    }

    if !existing_handles.is_empty() {
        return; // already spawned
    }

    let hitbox_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.0, 0.0, 0.3),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });
    let hitbox_mesh = meshes.add(Capsule3d::new(HITBOX_CAPSULE_RADIUS, BASE_ARROW_LEN));

    for axis in 0..3 {
        let color = ARROW_COLORS[axis];
        let mat = materials.add(StandardMaterial {
            base_color: color,
            emissive: color.into(),
            unlit: true,
            ..default()
        });
        let mat_dim = materials.add(StandardMaterial {
            base_color: color.with_alpha(0.5),
            emissive: color.with_alpha(0.5).into(),
            unlit: true,
            ..default()
        });

        let shaft_mesh = meshes.add(Cylinder::new(SHAFT_RADIUS, 1.0));
        let cone_mesh = meshes.add(Cone { radius: CONE_RADIUS, height: CONE_HEIGHT });

        for positive in [true, false] {
            let material = if positive { mat.clone() } else { mat_dim.clone() };

            commands.spawn((
                Mesh3d(shaft_mesh.clone()),
                MeshMaterial3d(material.clone()),
                Transform::default(),
                Pickable::IGNORE,
                ArrowHandle { axis, positive },
                ArrowShaft,
            ));

            commands.spawn((
                Mesh3d(cone_mesh.clone()),
                MeshMaterial3d(material),
                Transform::default(),
                Pickable::IGNORE,
                ArrowHandle { axis, positive },
                ArrowCone,
            ));

            commands.spawn((
                Mesh3d(hitbox_mesh.clone()),
                MeshMaterial3d(hitbox_mat.clone()),
                Transform::default(),
                ArrowHandle { axis, positive },
                ArrowHitbox,
            ));
        }
    }
}

/// Update arrow handle transforms each frame. Arrows maintain fixed screen size.
fn update_arrow_transforms(
    cache: Res<PredictionCache>,
    selected: Res<SelectedNode>,
    plan: Res<ManeuverPlan>,
    live_dom: Res<LiveDominantBody>,
    physics: Res<PhysicsState>,
    camera_focus: Res<CameraFocus>,
    orbit_cam: Res<OrbitCamera>,
    mut shafts: Query<(&ArrowHandle, &mut Transform), (With<ArrowShaft>, Without<ArrowCone>, Without<ArrowHitbox>)>,
    mut cones: Query<(&ArrowHandle, &mut Transform), (With<ArrowCone>, Without<ArrowShaft>, Without<ArrowHitbox>)>,
    mut hitboxes: Query<(&ArrowHandle, &mut Transform), (With<ArrowHitbox>, Without<ArrowShaft>, Without<ArrowCone>)>,
) {
    let trail_frame = live_dom.index;
    let offset = trail::trail_frame_offset(&physics, trail_frame, camera_focus.active_frame);

    let node_info = selected.id.and_then(|id| {
        let node = plan.nodes.iter().find(|n| n.id == id)?;
        let result = cache.result.as_ref()?;
        node_world_pos_and_frame(result, node.burn.time_window().0, trail_frame)
    });

    let Some((base_pos, frame)) = node_info else { return };
    let world_pos = base_pos + offset;

    let s = (orbit_cam.distance * RENDER_SCALE) as f32;
    let arrow_len = BASE_ARROW_LEN * s;

    for (handle, mut transform) in &mut shafts {
        let dir = frame.col(handle.axis).normalize();
        let sign = if handle.positive { 1.0 } else { -1.0 };
        let midpoint = world_pos + dir * sign * arrow_len * 0.5;
        let rotation = Quat::from_rotation_arc(Vec3::Y, dir * sign);
        *transform = Transform {
            translation: midpoint,
            rotation,
            scale: Vec3::new(s, arrow_len, s),
        };
    }

    for (handle, mut transform) in &mut cones {
        let dir = frame.col(handle.axis).normalize();
        let sign = if handle.positive { 1.0 } else { -1.0 };
        let tip = world_pos + dir * sign * arrow_len;
        let rotation = Quat::from_rotation_arc(Vec3::Y, dir * sign);
        *transform = Transform {
            translation: tip,
            rotation,
            scale: Vec3::splat(s),
        };
    }

    for (handle, mut transform) in &mut hitboxes {
        let dir = frame.col(handle.axis).normalize();
        let sign = if handle.positive { 1.0 } else { -1.0 };
        let midpoint = world_pos + dir * sign * arrow_len * 0.5;
        let rotation = Quat::from_rotation_arc(Vec3::Y, dir * sign);
        *transform = Transform {
            translation: midpoint,
            rotation,
            scale: Vec3::splat(s),
        };
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
    live_dom: Res<LiveDominantBody>,
    physics: Res<PhysicsState>,
    camera_focus: Res<CameraFocus>,
    orbit_cam: Res<OrbitCamera>,
    camera_q: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
) {
    let event = trigger.event();
    let entity = event.entity;
    let Ok(handle) = handles.get(entity) else { return };
    let Ok((camera, cam_transform)) = camera_q.single() else { return };

    let trail_frame = live_dom.index;
    let offset = trail::trail_frame_offset(&physics, trail_frame, camera_focus.active_frame);

    let node_info = selected.id.and_then(|id| {
        let node = plan.nodes.iter().find(|n| n.id == id)?;
        let result = cache.result.as_ref()?;
        node_world_pos_and_frame(result, node.burn.time_window().0, trail_frame)
    });

    let Some((base_pos, frame)) = node_info else { return };
    let world_pos = base_pos + offset;
    let s = (orbit_cam.distance * RENDER_SCALE) as f32;
    let arrow_len = BASE_ARROW_LEN * s;
    let dir = frame.col(handle.axis).normalize();
    let sign = if handle.positive { 1.0 } else { -1.0 };
    let tip = world_pos + dir * sign * arrow_len;

    let Some(node_screen) = camera.world_to_viewport(cam_transform, world_pos).ok() else { return };
    let Some(tip_screen) = camera.world_to_viewport(cam_transform, tip).ok() else { return };

    drag.active_axis = Some(handle.axis);
    drag.axis_screen_dir = (tip_screen - node_screen).normalize_or_zero();
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
                    node.burn = Box::new(ImpulseBurn {
                        time,
                        delta_v,
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
                    clock.warp_index = 0;
                    clock.warp = clock.warp_levels[0];
                    clock.paused = false;
                }
            }
            _ => {}
        }
    }
}

/// Draw snap indicator and node markers.
fn render_maneuver_nodes(
    cache: Res<PredictionCache>,
    live_dom: Res<LiveDominantBody>,
    physics: Res<PhysicsState>,
    camera_focus: Res<CameraFocus>,
    plan: Res<ManeuverPlan>,
    selected: Res<SelectedNode>,
    hover: Res<TrailHover>,
    mut gizmos: Gizmos,
) {
    let Some(ref result) = cache.result else { return };
    let trail_frame = live_dom.index;
    let offset = trail::trail_frame_offset(&physics, trail_frame, camera_focus.active_frame);

    if hover.active {
        if let Some(pos) = hover.snap_world_pos {
            gizmos.sphere(Isometry3d::from_translation(pos), 0.15, Color::srgb(1.0, 1.0, 0.0));
        }
    }

    for node in &plan.nodes {
        let (burn_start, _) = node.burn.time_window();
        let Some(world_pos) = find_node_position_on_trail(result, burn_start, trail_frame, offset) else {
            continue;
        };

        let is_selected = selected.id == Some(node.id);
        let color = if is_selected {
            Color::srgb(1.0, 0.8, 0.0)
        } else {
            Color::srgb(0.8, 0.6, 0.0)
        };
        let size = if is_selected { 0.15 } else { 0.1 };
        gizmos.sphere(Isometry3d::from_translation(world_pos), size, color);
    }
}

/// Bottom panel: Node editor.
fn node_editor_panel(
    mut contexts: EguiContexts,
    plan: ResMut<ManeuverPlan>,
    mut selected: ResMut<SelectedNode>,
    clock: Res<SimClock>,
    ship_config: Res<ShipConfig>,
    mut node_dv: ResMut<NodeDeltaV>,
    mut writer: bevy::ecs::message::MessageWriter<ManeuverEvent>,
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
            ui.label(format!("Total Δv: {:.1} m/s", total_dv));

            let burn_duration = if ship_config.thrust_newtons > 0.0 {
                total_dv * (ship_config.dry_mass + ship_config.fuel_mass) / ship_config.thrust_newtons
            } else {
                0.0
            };
            ui.label(format!("Est. burn: {:.1}s", burn_duration));
            ui.separator();

            if ui.button("Delete").clicked() {
                writer.write(ManeuverEvent::DeleteNode { id: sel_id });
                selected.id = None;
            }
            if ui.button("Warp to node").clicked() {
                writer.write(ManeuverEvent::WarpToNode { id: sel_id });
            }
        });

        ui.horizontal(|ui| {
            let mut pg = node_dv.prograde as f32;
            let mut nm = node_dv.normal as f32;
            let mut rd = node_dv.radial as f32;

            let changed_pg = ui.add(egui::DragValue::new(&mut pg).speed(1.0).prefix("P: ").suffix(" m/s")).changed();
            let changed_nm = ui.add(egui::DragValue::new(&mut nm).speed(1.0).prefix("N: ").suffix(" m/s")).changed();
            let changed_rd = ui.add(egui::DragValue::new(&mut rd).speed(1.0).prefix("R: ").suffix(" m/s")).changed();

            if changed_pg || changed_nm || changed_rd {
                node_dv.prograde = pg as f64;
                node_dv.normal = nm as f64;
                node_dv.radial = rd as f64;
                writer.write(ManeuverEvent::AdjustNode {
                    id: sel_id,
                    delta_v: DVec3::new(pg as f64, nm as f64, rd as f64),
                });
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

        let idx = seg.times
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| ((**a - time).abs()).partial_cmp(&((**b - time).abs())).unwrap())
            .map(|(i, _)| i)?;

        let frame_pos = if frame_body < seg.body_positions[idx].len() {
            seg.body_positions[idx][frame_body]
        } else {
            DVec3::ZERO
        };
        let world_pos = ((seg.points[idx] - frame_pos) * RENDER_SCALE).as_vec3();

        let vel = if idx + 1 < seg.points.len() {
            let dt = seg.times[idx + 1] - seg.times[idx];
            if dt > 1e-10 { (seg.points[idx + 1] - seg.points[idx]) / dt } else { DVec3::X }
        } else if idx > 0 {
            let dt = seg.times[idx] - seg.times[idx - 1];
            if dt > 1e-10 { (seg.points[idx] - seg.points[idx - 1]) / dt } else { DVec3::X }
        } else {
            DVec3::X
        };

        let dom = seg.dominant_body[idx];
        let center_pos = if dom < seg.body_positions[idx].len() {
            seg.body_positions[idx][dom]
        } else {
            DVec3::ZERO
        };

        let orbital = orbital_frame(seg.points[idx], vel, center_pos);
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
