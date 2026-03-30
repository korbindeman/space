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
cargo run -p stability_check -- assets/solar_system.ron 100 60  # Validate orbital stability (years, dt)
cargo run -p stability_check -- assets/solar_system.ron 100 60 --fix  # Auto-fix unstable orbits
```

First build is slow due to Bevy. Dev builds use `opt-level = 1` with fully optimized deps (`opt-level = 3`) for acceptable runtime performance.

The game runs borderless fullscreen with vsync off. Assets are loaded from `../../assets` relative to the game crate (set via `AssetPlugin` in `main.rs`).

## Architecture

Three-layer stack where each layer depends only on the one below:

```
Game (Bevy App)         → crates/game/src/          (SimPlugin + NavMapPlugin)
  sim.rs                                            (physics stepping, scene setup, dominant body tracking)
  solar_system.rs                                   (RON scene loader, Keplerian element conversion)
  nav_map/                                          (camera, prediction, maneuver, trajectory, body, target, hud)
Prediction Pipeline     → crates/prediction/        (ghost propagation, phase machine, encounters)
Simulation Core         → crates/sim/               (integrator, gravity, burns, orbital math)
Stability Tool          → crates/stability/          (CLI: orbital stability validation + auto-fix)
```

**Key boundary:** `space_sim`, `space_prediction`, and `stability_check` have zero Bevy dependency — pure Rust with only `glam` for math. They can be tested headless and are WASM-compatible.

### Workspace Crates

- **`space_sim`** (`crates/sim/`) — N-body simulation core. `SimState` (positions/velocities/masses as parallel vecs), RK4 integrator via `Integrator` trait, `ForceModel` trait (`NBodyGravity`), `BurnModel` trait (`ImpulseBurn`), orbital math helpers. All units are SI (meters, seconds, kilograms).
- **`space_prediction`** (`crates/prediction/`) — Trajectory prediction pipeline. Takes a `SimState` + maneuver nodes, propagates a ghost copy forward, produces `PredictionResult` with trail segments, encounters, and closest approaches. Uses a phase machine (`PredictionPhase`: Orbiting/Encounter/Transit/Done) to control adaptive timestep and termination.
- **`stability_check`** (`crates/stability/`) — CLI tool for validating orbital stability of `solar_system.ron` definitions. Tracks SMA/eccentricity drift, detects collisions and escapes, identifies mean motion resonances. Has `--fix` mode that searches for velocity corrections. No Bevy dependency.
- **`space_game`** (`crates/game/`) — Bevy app with two top-level plugins:
  - **`sim`** — `SimPlugin`: `PhysicsState`, `SimClock`, `ShipConfig`, `LiveDominantBody`, `BodyData` (static body metadata), `TrailFrame` (per-frame trail reference frame + offset), components (`SimBody`, `CelestialBody`, `Ship`), scene setup from `solar_system.ron`, physics stepping, transform sync
  - **`nav_map`** — `NavMapPlugin`: unified navigation map UI, contains all sub-plugins:
    - **`camera`** — `CameraPlugin`: `OrbitCamera`, `CameraFocus`, mouse orbit, keyboard focus cycling and time warp controls
    - **`prediction`** — `PredictionPlugin`: `PredictionCache`, `PredictFurther`, prediction pipeline runner
    - **`maneuver`** — `ManeuverPlugin`: `ManeuverPlan`, `ManeuverEvent`, `NodeDeltaV`, `SelectedNode`, arrow handles, node placement/editing, node editor panel
    - **`trajectory`** — `TrajectoryPlugin`: predicted trail rendering via gizmos, `BodyOrbitCache` for cached celestial orbit lines
    - **`body`** — `BodyPlugin`: celestial body spawning/rendering, `EncounterGhost` entities for predicted encounter positions
    - **`target`** — `TargetPlugin`: `TargetBody`, targeting input, encounter info panel, ghost trail rendering
    - **`hud`** — `HudPlugin`: time control panel

### Scene Definition

The solar system is defined declaratively in `assets/solar_system.ron` (RON format) and loaded by `solar_system.rs`. Bodies specify mass (in solar/Jupiter/Earth/kg units), radius, color, and full Keplerian orbital elements (SMA, eccentricity, true anomaly, inclination, argument of periapsis, longitude of ascending node). The loader converts Keplerian elements to Cartesian state vectors at startup via `keplerian_state()`.

### Shared Constants (`nav_map/mod.rs`)

- `RENDER_SCALE: f64 = 1e-6` — sim meters to render units
- `MARKER_RADIUS: f32 = 0.006` — body/node icon size as fraction of camera distance
- `BODY_COLORS` — color array indexed by sim body index (matches `solar_system.ron` order)
- `format_distance()` / `format_duration()` — smart unit formatting helpers

### Simulation Flow

1. `step_simulation` advances `PhysicsState` using RK4 + N-body gravity each frame
2. `run_prediction` propagates a ghost state forward from current time, applying maneuver nodes, producing trail segments split at each node
3. Trail segments carry per-point dominant body index and prediction phase for rendering
4. Maneuver nodes use `ImpulseBurn` with delta-v in orbital frame (prograde, normal, radial)
5. When sim time reaches a node, `handle_maneuver_events` executes the burn

### Coordinate System

- Sim runs in absolute coordinates (Sun at origin). Render scale defined by `RENDER_SCALE` in `nav_map/mod.rs`.
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

**egui integration.** UI panels use `bevy_egui` — systems take `EguiContexts` parameter and draw immediate-mode UI with `egui::Window` / `egui::SidePanel`. Used in hud, camera, maneuver, and target modules.

**Common pitfalls:**
- `StandardMaterial` renders black if mesh lacks normals. Normal maps need tangents.
- SSAO and TAA require `Msaa::Off`.
- Default far plane is 1000 units — override for large scenes.
- `GltfAssetLabel` sub-asset loads parse the entire file each time — load `Handle<Gltf>` once and access sub-assets through it.
- `compute_matrix()` renamed to `to_matrix()` in 0.17.
- System set renames: `TransformSystem` → `TransformSystems`, `CameraUpdateSystem` → `CameraUpdateSystems`.

## Bevy References

- Full Bevy 0.18 API patterns: `internal/bevy_guide.md`
- Official examples: `https://github.com/bevyengine/bevy/blob/v0.18.1/examples/` — fetch source when implementing a new feature. Key examples for this project: `camera/camera_orbit.rs`, `gizmos/3d_gizmos.rs`, `movement/physics_in_fixed_timestep.rs`, `time/virtual_time.rs`.

## Design Reference

`internal/DESIGN.md` contains the full design document with detailed specs for each layer. Consult it for phase machine logic, encounter detection rules, adaptive timestep strategy, and UI behavior specs.
