# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Sol — a spaceflight simulator with N-body orbital map view, maneuver planner, and encounter visualization. Built with Rust + Bevy 0.18 + egui.

## Build & Run Commands

```bash
cargo run -p space_game       # Run the game
cargo build                   # Build all crates
cargo test                    # Run all tests
cargo test -p space_sim       # Run tests for a single crate
cargo test -p space_prediction
cargo clippy                  # Lint
```

First build is slow due to Bevy. Dev builds use `opt-level = 1` with fully optimized deps (`opt-level = 3`) for acceptable runtime performance.

## Architecture

Three-layer stack where each layer depends only on the one below:

```
Game Plugins            → crates/game/src/          (Bevy plugins: sim, camera, prediction, maneuver, trail, target, hud)
Prediction Pipeline     → crates/prediction/        (ghost propagation, phase machine, encounters)
Simulation Core         → crates/sim/               (integrator, gravity, burns, orbital math)
```

**Key boundary:** `space_sim` and `space_prediction` have zero Bevy dependency — pure Rust with only `glam` for math. They can be tested headless and are WASM-compatible.

### Workspace Crates

- **`space_sim`** (`crates/sim/`) — N-body simulation core. `SimState` (positions/velocities/masses as parallel vecs), RK4 integrator via `Integrator` trait, `ForceModel` trait (`NBodyGravity`), `BurnModel` trait (`ImpulseBurn`), orbital math helpers. All units are SI (meters, seconds, kilograms).
- **`space_prediction`** (`crates/prediction/`) — Trajectory prediction pipeline. Takes a `SimState` + maneuver nodes, propagates a ghost copy forward, produces `PredictionResult` with trail segments, encounters, and closest approaches. Uses a phase machine (`PredictionPhase`: Orbiting/Encounter/Transit/Done) to control adaptive timestep and termination.
- **`space_game`** (`crates/game/`) — Bevy app organized as feature plugins:
  - **`sim`** — `SimPlugin`: `PhysicsState`, `SimClock`, `ShipConfig`, `LiveDominantBody`, `BodyData` (static body metadata), `TrailFrame` (per-frame trail reference frame + offset), components (`SimBody`, `CelestialBody`, `Ship`), scene setup, physics stepping, transform sync
  - **`camera`** — `CameraPlugin`: `OrbitCamera`, `CameraFocus`, mouse orbit, keyboard focus cycling and time warp controls
  - **`prediction`** — `PredictionPlugin`: `PredictionCache`, `PredictFurther`, prediction pipeline runner
  - **`maneuver`** — `ManeuverPlugin`: `ManeuverPlan`, `ManeuverEvent`, `NodeDeltaV`, `SelectedNode`, arrow handles, node placement/editing, node editor panel
  - **`trail`** — `TrailPlugin`: predicted trail rendering via gizmos
  - **`target`** — `TargetPlugin`: `TargetBody`, targeting input, encounter info panel, ghost trail rendering
  - **`hud`** — `HudPlugin`: time control panel, orbital info panel

### Simulation Flow

1. `step_simulation` advances `PhysicsState` using RK4 + N-body gravity each frame
2. `run_prediction` propagates a ghost state forward from current time, applying maneuver nodes, producing trail segments split at each node
3. Trail segments carry per-point dominant body index and prediction phase for rendering
4. Maneuver nodes use `ImpulseBurn` with delta-v in orbital frame (prograde, normal, radial)
5. When sim time reaches a node, `handle_maneuver_events` executes the burn

### Coordinate System

- Sim runs in absolute coordinates (Sun at origin). Render scale: `1e-6` (sim meters to render units).
- Orbital frame is computed per-point relative to the dominant body (determined by Hill sphere membership).
- Camera orbits the focused entity with yaw/pitch/distance controls.

## Design Principles

**Derived values get their own resource.** If multiple systems need the same value computed from the same inputs (e.g., trail reference frame from camera focus + dominant body), compute it once in a dedicated system and store it in a resource. Don't scatter the same conditional logic across every consumer.

**Static data is a resource, not a query.** Data that doesn't change after setup (body radii, Hill radii) belongs in a resource built at setup time, not reconstructed each frame from ECS queries.

**One source of truth per concept.** When a value like "which body's frame should trails render in" needs to be consistent across rendering, input handling, and UI, it must come from a single authoritative source — not be independently derived in each place.

## Bevy 0.18 Patterns

Full reference: `internal/bevy_guide.md`. Key points for coding in this project:

**Spawn component tuples, never bundles.** Bundles (`Camera3dBundle`, `PbrBundle`) are removed. Spawn `(Camera3d::default(), Transform::from_xyz(...))` — Required Components auto-add `Camera`, `Projection`, `Frustum`, `Visibility`, etc.

**Mesh/material newtypes.** `Mesh3d(Handle<Mesh>)` and `MeshMaterial3d(Handle<M>)` are newtypes. `StandardMaterial` implements `From<Color>` and `From<Handle<Image>>`, so `materials.add(Color::WHITE)` works.

**Coordinate system.** Right-handed Y-up (X-right, Y-up, Z-toward-viewer). Camera forward is -Z.

**Crate paths after 0.17 render split.** `bevy::prelude::*` covers common types. For less-common: `bevy::light` (lights, shadows, `CascadeShadowConfigBuilder`, `light_consts`), `bevy::camera` (camera, visibility, culling), `bevy::mesh`, `bevy::image`, `bevy::post_process` (Bloom, etc.), `bevy::anti_alias`, `bevy::pbr`.

**HDR is a marker component.** `Camera.hdr` field removed — use `commands.spawn((Camera3d::default(), Hdr))`. Bloom/AutoExposure auto-require Hdr.

**Event/Message split (0.17+).** `Event` is for observers (entity-targeted, immediate). Buffered cross-system communication uses `Message` / `MessageWriter` / `MessageReader`.

**System ordering.** `Transform` is local/writable, `GlobalTransform` is computed in `PostUpdate` by `TransformSystems::TransformPropagate`. Write to `Transform`, read `GlobalTransform` for world-space. Physics in `FixedUpdate`, visuals in `Update`. Systems with overlapping `&mut` access serialize unless separated by `Without<T>` filters.

**Hierarchy.** Use `children![]` macro or `ChildOf(parent)`. Container entities need `Visibility::default()` even if they don't render. Despawning parent despawns children — use `.detach_all_children()` to preserve them.

**Plugin structure.** Plugins register systems/resources/observers. Component/resource type definitions belong in shared modules, not plugins.

**Common pitfalls:**
- `StandardMaterial` renders black if mesh lacks normals. Normal maps need tangents.
- SSAO and TAA require `Msaa::Off`.
- Default far plane is 1000 units — override for large scenes.
- `GltfAssetLabel` sub-asset loads parse the entire file each time — load `Handle<Gltf>` once and access sub-assets through it.
- `compute_matrix()` renamed to `to_matrix()` in 0.17.
- System set renames: `TransformSystem` → `TransformSystems`, `CameraUpdateSystem` → `CameraUpdateSystems`.

## Bevy 0.18 Examples Reference

Official examples are an excellent source of implementation patterns. Fetch the source from GitHub when implementing a feature that matches a topic below.

**Base URL:** `https://github.com/bevyengine/bevy/blob/v0.18.1/examples/`

### 3D Rendering & Scene (`3d/`)
- `3d_scene.rs` — minimal scene setup (camera, light, mesh)
- `3d_shapes.rs` — all built-in 3D primitives
- `bloom_3d.rs` — HDR bloom setup
- `lighting.rs` — point, directional, spot lights with shadows
- `pbr.rs` — full PBR material showcase (metallic, roughness, emissive)
- `transparency_3d.rs` — alpha blending modes
- `lines.rs` — line rendering
- `generate_custom_mesh.rs` — procedural mesh creation with vertex attributes
- `post_processing.rs` — fullscreen post-process effects
- `tonemapping.rs` — tonemapping operator comparison
- `anti_aliasing.rs` — MSAA, FXAA, SMAA, TAA comparison
- `skybox.rs` — environment map / skybox
- `atmosphere.rs` — atmospheric scattering
- `fog.rs`, `atmospheric_fog.rs`, `volumetric_fog.rs` — fog techniques
- `render_to_texture.rs` — offscreen rendering
- `split_screen.rs` — multiple viewports
- `3d_viewport_to_world.rs` — screen-to-world ray casting
- `mesh_ray_cast.rs` — ray-mesh intersection
- `parenting.rs` — transform hierarchy
- `visibility_range.rs` — LOD via distance
- `orthographic.rs` — orthographic 3D camera

### Camera (`camera/`)
- `camera_orbit.rs` — orbit camera (yaw/pitch/distance) **← directly relevant to this project**
- `free_camera_controller.rs` — WASD fly camera
- `pan_camera_controller.rs` — pan/zoom camera
- `projection_zoom.rs` — perspective vs ortho zoom
- `first_person_view_model.rs` — FPS view model rendering
- `custom_projection.rs` — custom projection matrix

### Gizmos (`gizmos/`)
- `3d_gizmos.rs` — lines, circles, arcs, arrows, spheres **← trail/orbit rendering patterns**
- `axes.rs` — axis indicator gizmo
- `light_gizmos.rs` — debug light visualization

### Input (`input/`)
- `mouse_input.rs` — mouse button detection
- `mouse_input_events.rs` — mouse move/scroll events
- `keyboard_input.rs` — key press detection
- `keyboard_input_events.rs` — keyboard events
- `mouse_grab.rs` — cursor lock/grab
- `gamepad_input.rs` — controller support

### ECS Patterns (`ecs/`)
- `observers.rs` — observer pattern basics
- `observer_propagation.rs` — event bubbling through hierarchy
- `message.rs`, `send_and_receive_messages.rs` — buffered cross-system messaging
- `component_hooks.rs` — on-add/remove hooks
- `change_detection.rs` — `Changed<T>`, `Added<T>` filters
- `fixed_timestep.rs` — `FixedUpdate` usage
- `hierarchy.rs` — parent/child relationships
- `run_conditions.rs` — conditional system execution
- `relationships.rs` — entity relationships
- `custom_schedule.rs` — custom schedules
- `removal_detection.rs` — detecting component removal
- `system_param.rs` — custom system parameters
- `one_shot_systems.rs` — on-demand system execution
- `entity_disabling.rs` — disabling entities

### Movement & Physics (`movement/`)
- `physics_in_fixed_timestep.rs` — FixedUpdate physics with interpolation **← directly relevant**
- `smooth_follow.rs` — smooth camera/entity following

### Transforms (`transforms/`)
- `3d_rotation.rs` — rotation methods (Quat, Euler)
- `transform.rs` — Transform basics
- `translation.rs` — movement via translation

### Time (`time/`)
- `virtual_time.rs` — time scaling (pause, slow-mo, fast-forward) **← relevant to time warp**
- `time.rs` — time resource basics
- `timers.rs` — recurring/one-shot timers

### Shaders (`shader/`)
- `shader_material.rs` — custom material with WGSL
- `extended_material.rs` — extending StandardMaterial
- `animate_shader.rs` — time-based shader animation
- `shader_prepass.rs` — depth/normal prepass access
- `shader_material_screenspace_texture.rs` — screen-space UVs

### UI (`ui/`)
- `button.rs` — clickable UI button
- `text.rs` — text rendering
- `flex_layout.rs` — flexbox layout
- `scroll.rs` — scrollable containers
- `ui_material.rs` — custom UI shaders
- `relative_cursor_position.rs` — cursor position within UI nodes

### Picking (`picking/`)
- `mesh_picking.rs` — clicking on 3D meshes
- `simple_picking.rs` — basic entity picking
- `dragdrop_picking.rs` — drag and drop

### App Structure (`app/`)
- `plugin.rs` — plugin definition
- `plugin_group.rs` — plugin groups
- `headless.rs` — headless app (no rendering)

### Assets (`asset/`)
- `asset_loading.rs` — loading assets
- `alter_mesh.rs` — runtime mesh modification
- `hot_asset_reloading.rs` — live reload

### Animation (`animation/`)
- `animated_transform.rs` — keyframe transform animation
- `eased_motion.rs` — easing/tweening
- `easing_functions.rs` — easing curve catalog

### State Management (`state/`)
- `states.rs` — basic state machine
- `computed_states.rs` — derived states
- `sub_states.rs` — hierarchical states

## Design Reference

`internal/DESIGN.md` contains the full design document with detailed specs for each layer. Consult it for phase machine logic, encounter detection rules, adaptive timestep strategy, and UI behavior specs.
