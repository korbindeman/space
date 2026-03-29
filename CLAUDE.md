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
  - **`sim`** — `SimPlugin`: `PhysicsState`, `SimClock`, `ShipConfig`, `LiveDominantBody`, components (`SimBody`, `CelestialBody`, `Ship`), scene setup, physics stepping, transform sync, auto-warp
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

## Design Reference

`internal/DESIGN.md` contains the full design document with detailed specs for each layer. Consult it for phase machine logic, encounter detection rules, adaptive timestep strategy, and UI behavior specs.
