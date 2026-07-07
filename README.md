# Vehicle Dynamics Simulator v9.3

Real-time 2-D vehicle dynamics simulator with component-level design and physics.
PyQt6 desktop app — custom engine, transmission, brakes, chassis, and body designer with a live top-down viewport.

---

## Quick Start

```bash
python -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

> **macOS note:** `torch==2.7.1` may fail on some Python / arch combos. `requirements.txt` already uses `torch>=2.2.0` floor pins.

---

## Project Structure

```
v9.3/
├── main.py                           # Entry point — QApplication + MainWindow
├── requirements.txt
│
├── simulation/                       # Pure-Python physics models (no Qt)
│   ├── engine.py                     # EngineModel — 4-stroke IC engine
│   ├── transmission.py               # TransmissionModel — manual gearbox
│   ├── automatic.py                  # AutomaticTransmissionModel — torque converter + auto shift
│   ├── brakes.py                     # BrakeModel — friction brakes
│   └── mass.py                       # Component mass estimators
│
├── gui/
│   ├── app.py                        # MainWindow — menu bar, tab host, status bar
│   ├── widgets.py                    # Reusable widgets (CollapsibleSection, SteeringWheel, etc.)
│   ├── logo.png
│   └── tabs/
│       └── vehicle_design/
│           ├── viewport.py           # ViewportWidget — physics loop + top-down renderer
│           ├── component_designer.py # ComponentDesignerWidget — right-panel config
│           ├── direct_control.py     # DirectControlWidget — sliders / gear / clutch / brakes
│           ├── wheel_frame_section.py# WheelFrameBody + resolve_frame() + mass/inertia model
│           └── vehicle_info.py       # VehicleInfoWidget — live telemetry readout
│
└── assets/                           # JSON template directories (user-saveable)
    ├── engines/
    ├── transmissions/
    ├── brakes/
    ├── wheelframes/
    ├── frame_and_body/
    ├── vehicle_bodies/
    └── vehicles/
```

---

## Architecture

### Layout (gui/app.py → MainWindow)

```
┌──────────────────────────────────────────────────────────────────┐
│ Custom Menu Bar  (File · View · Help)                           │
├──────────┬───────────────────────────────────┬──────────────────┤
│ Direct   │                                   │ Component        │
│ Control  │        ViewportWidget              │ Designer         │
│ Panel    │      (800×800, top-down,           │ (scrollable,     │
│ (left)   │       60 fps physics)              │  accordion)      │
│          │                                   │ (right)          │
├──────────┴───────────────────────────────────┴──────────────────┤
│ Vehicle Info (mass · speed · heading)                           │
│ Status Bar                                                     │
└──────────────────────────────────────────────────────────────────┘
```

- `MainWindow` holds a `QTabWidget`; first tab is `VehicleDesignTab`.
- Panels connect via **Qt signals** — no model holds a reference to any widget.

### Signal Wiring (VehicleDesignTab)

```
WheelFrameBody ──frame_changed──►  ViewportWidget.set_wheel_frame()
                                   DirectControlWidget.update_axle_controls()

EngineConfigBody ──engine_changed──► ViewportWidget.set_engine_cfg()

TransmissionConfigBody ──transmission_changed──► ViewportWidget.set_transmission_cfg()
                                                  DirectControlWidget.update_gear_controls()

BrakeConfigBody ──brakes_changed──► ViewportWidget.set_brakes_cfg()

VehicleBodyBody ──floor_changed──► ViewportWidget.set_chassis_floor()

DirectControlWidget ──steering_changed──►  ViewportWidget.set_steer()
                    ──torque_changed──►    ViewportWidget.set_torque()
                    ──accelerator_changed──► ViewportWidget.set_accelerator()
                    ──clutch_changed──►    ViewportWidget.set_clutch()
                    ──gear_changed──►      ViewportWidget.set_gear()
                    ──brake_changed──►     ViewportWidget.set_brake()
                    ──range_changed──►     ViewportWidget.set_drive_range()
                    ──drive_mode_changed──► ViewportWidget.set_drive_mode()
                    ──mode_changed──►      ViewportWidget.set_control_mode()

ViewportWidget ──state_updated──► VehicleInfoWidget.update_state()
               ──engine_torque_changed──► DirectControlWidget.update_engine_torque()
```

---

## Simulation Models

### Engine — `simulation/engine.py` → `EngineModel`

4-stroke naturally-aspirated IC engine.

| Parameter | Default | Unit |
|-----------|---------|------|
| `capacity_l` | 2.0 | L |
| `max_rpm` | 6000 | RPM |
| `idle_rpm` | 800 | RPM |
| `k` | 1,232,000 | J·s/kg |
| `c_drag` | 0.05 | Nm/RPM |
| `afr_target` | 14.7 | — |

**Core torque equation:**

```
ṁ_air = (V_d / 2) × (N / 60) × ρ_air × VE(N) × α
τ     = ṁ_air × k / ω
```

N cancels → torque curve shape is entirely determined by the VE map.

- **VE map**: piecewise-linear, draggable in the GUI. Points at 1000 RPM intervals, 0–100 %.
- **Engine braking** (α = 0): `τ = −c_drag × (RPM − idle_rpm)` above idle; below idle, a tiny `α = 0.05` holds idle speed.
- **Derived calcs**: `peak_torque_nm()`, `peak_power_kw()`, `k_from_peak_torque()`, `k_from_peak_power()` — bidirectional linking so editing any one of k/τ/kW/hp updates the others.

### Transmission — `simulation/transmission.py` → `TransmissionModel`

Manual gearbox with clutch.

| Parameter | Default | Notes |
|-----------|---------|-------|
| `gear_ratios` | {-1: −3.32, 0: 0, 1: 3.54 … 5: 0.85} | Signed; 0 = neutral |
| `final_drive` | 3.90 | |
| `eta` | 0.95 | Mechanical efficiency |
| `I_engine` | 0.15 | kg·m² — engine-side inertia |
| `clutch_torque_max` | 350 | Nm — clutch capacity |
| `launch_inertia_ratio` | 20 | Stall protection threshold |

**Physics per tick:**

1. `GR_tot = gear_ratio × final_drive`
2. Clutch torque transfer clamped to `min(cap, engine_torque)` where `cap = e × clutch_torque_max`
3. Engine RPM integrated: `ω += (τ_net / I_engine) × dt`
4. Locked clutch (`e ≥ 1`): engine RPM = `|wheel_rpm| × |GR_tot|`; stalls if reflected inertia exceeds threshold.
5. `wheel_torque = τ_clutch × GR_tot × η`

**Stall model**: If the clutch is locked (`e > 0.1`), engine RPM < idle, and engine torque ≤ 0 → `is_stalled = True`, outputs 0. Shifting to Neutral restarts.

### Automatic Transmission — `simulation/automatic.py` → `AutomaticTransmissionModel`

Torque converter coupling + automatic shift controller.

**Torque converter:**
- Pump load: `τ_pump = C × N_e² × max(0, 1 − SR²)` where `SR = N_turbine / N_engine`
- Torque ratio: linear from `stall_torque_ratio` (default 2.0) at SR=0 to 1.0 at `coupling_sr` (0.86)
- Lock-up clutch engages above `lockup_sr` (0.90), disengages below `unlock_sr` (0.80); only in gear ≥ 2

**Shift logic** — throttle-dependent shift points:
```
up_rpm   = idle + span × (up_base + up_span × throttle)
down_rpm = idle + span × (dn_base + dn_span × throttle)
```

**Drive modes** (configurable per template):

| Mode | Up Base | Up Span | Dn Base | Dn Span | Shift Time |
|------|---------|---------|---------|---------|------------|
| ECO | 0.20 | 0.35 | 0.06 | 0.18 | 0.60 s |
| CITY | 0.30 | 0.55 | 0.10 | 0.25 | 0.40 s |
| SPORT | 0.55 | 0.42 | 0.22 | 0.30 | 0.25 s |

**Drive range selector**: P / R / N / D — maps to fixed gears; D enables auto-shift.

### Brakes — `simulation/brakes.py` → `BrakeModel`

Friction brake opposing motion.

```
torque = brake × max_torque
force  = torque / tyre_radius
force  = min(force, μ × mass × 9.81)    # grip-limited
```

- `front_bias` (0.65 default) splits torque across axles (rendering only; physics uses total).
- Independent of clutch/gear — works in neutral and when stalled.

### Mass — `simulation/mass.py`

Single source of truth for component mass. Used by both UI labels and viewport physics.

| Component | Formula |
|-----------|---------|
| Engine | `40 + 55 × capacity_l` |
| Transmission | `35 + 9 × n_gears` (+30 for automatic) |
| Brakes | `6 + 0.009 × max_torque` |
| Body | `55 × (L + front_oh + rear_oh) × (W + 2 × side_oh)` |
| Frame | Steel C-section rail model (see `wheel_frame_section.py`) |

---

## Viewport Physics — `viewport.py` → `ViewportWidget._tick()`

60 fps QTimer-driven loop (`dt = 0.016 s`). Coordinates: world pixels, `_PX_PER_M = 40`.

### Tick sequence

1. **Engine mode**: compute engine torque → run transmission `.update()` → get wheel torque
2. **Direct RPM mode**: override speed kinematically (bypasses torque physics)
3. **Longitudinal dynamics** (`_apply_drive_torque()`):
   - Per-wheel torque → force at contact patch
   - Friction ceiling: `μ × N_w × tyre_width_factor` (wider tyre → up to 1.5× grip)
   - Within grip: kinematic rolling; excess torque → wheelspin via rotational inertia
   - Rolling resistance drag (`0.05` coasting, `0.50` in gear with no throttle)
   - Friction braking via `BrakeModel.decel_force()`, clamped to stop
4. **Lateral dynamics** (bicycle model):
   - Kinematic path: `ω_kin = v × tan_net / L`
   - Grip check: `F_centripetal > μ × m × g` → enter slip mode
   - Slip mode: tyre slip angles → cornering forces → integrate `v_y` and `ω`
   - Cornering stiffness scales with mass for Euler stability
   - Slip exit: `|v_y| < 0.08` and `|ω| < 0.04`
5. **Position integration**: heading-rotated velocity → world pixel displacement
6. **Differential**: open (per-side ω from Ackermann radii) or locked (equal ω)
7. **Fifth-wheel trailer**: kinematic hitch model, articulation angle clamped

### Steering modes

| Mode | Front δ | Rear δ |
|------|---------|--------|
| front | steer_rad | 0 |
| rear | 0 | steer_rad |
| both | steer_rad | −steer_rad |

### Dynamic mass/inertia

Total mass = sum of all component masses. Yaw inertia = frame base inertia × `(total_mass / frame_mass)`.

---

## Wheel Frame — `wheel_frame_section.py`

### `resolve_frame(cfg) → dict`

Expands raw UI config into canonical form consumed by viewport + controls.

- **Axle groups**: `front`, `middle`, `rear` — each with `axle_count`, `position_pct`, `wheels_per_axle` (1/2/4), `separation_cm`
- Middle group is always passive (not steerable, not drivable)
- Axle positions: fractional along frame length `[0.01, 0.99]`
- Multi-axle groups fan out from the centre position by `(tyre_diameter + separation) / frame_length`

### Mass/Inertia model (`_compute_mass_inertia`)

| Component | Model |
|-----------|-------|
| Rails | Two C-section steel (120×8 mm web + 2×60 mm flanges) × length |
| Cross members | 80×6 mm steel, one per 0.8 m |
| Axle beams | `20 + 8 × n_wheels` kg each |
| Tyres | `12 × (R/0.33)^1.5 × (w/0.20)^0.8` kg each |
| Hitch | 90 kg fifth-wheel turntable (when enabled) |

Yaw inertia: uniform rectangle for frame + parallel-axis offsets for axles and wheels.

### Fifth-wheel trailer config

- `hitch_pct`: hitch position along tractor frame (% from front)
- `trailer_length_m`, `max_angle_deg`
- Trailer axle group: `axle_count`, `wheels_per_axle`, `axle_position_pct`, `axle_separation_cm`

---

## Component Designer — `component_designer.py`

Right-panel accordion with collapsible sections:

| Section | Config body | Emits |
|---------|-------------|-------|
| Frame and Body | `WheelFrameBody` + `VehicleBodyBody` | `wheel_frame_changed`, `chassis_floor_changed` |
| Engine | `EngineConfigBody` | `engine_cfg_changed` |
| Transmission | `TransmissionConfigBody` | `transmission_cfg_changed` |
| Brakes | `BrakeConfigBody` | `brakes_cfg_changed` |

Each section has:
- **Template bar**: Load / Save / Save As / Delete from `assets/<component>/` JSON files
- **Checkable header**: toggles component visibility in viewport
- **Mass readout**: live-updated from mass formulas

### VE Map Editor (`_VeMapWidget`)

Custom `QWidget` paintEvent: grid, curve, draggable control points.
- X = RPM (0 → max_rpm), Y = VE fraction (0–1)
- Points at 1000 RPM intervals, drag to adjust
- Rebuilds on max_rpm change (interpolates new points from old)

### Transmission config

- Manual: gear count (1–8), per-gear ratio spinners, final drive, reverse ratio, clutch capacity, efficiency, engine inertia
- Automatic: same ratios + converter capacity, stall torque ratio, configurable drive modes (name, shift schedule params)
- Switching type rebuilds the direct-control panel (gear buttons ↔ P/R/N/D selector)

### Vehicle Body config

- Visibility toggles: chassis floor, fuel tank, body, windshields, lights
- Per-face overhangs: front, side, rear (separate for tractor / trailer)
- Corner styles: Angular / Bevelled (depth + angle) / Rounded (radius + eccentricity) — independent front/rear

---

## Direct Control — `direct_control.py`

### Two drive modes (toggle buttons)

| Mode | Torque slider | Accelerator slider |
|------|---------------|-------------------|
| Direct | Active (user sets Nm) | Disabled |
| Engine | Read-only (mirrors engine output) | Active (0–100 %) |

### Control outputs

| Signal | Type | Range |
|--------|------|-------|
| `steering_changed` | float | ±max_angle degrees |
| `torque_changed` | float | ±max_torque Nm |
| `accelerator_changed` | float | 0–1 |
| `brake_changed` | float | 0–1 |
| `clutch_changed` | float | 0–1 (1 = locked) |
| `gear_changed` | int | -1=R, 0=N, 1…N |
| `range_changed` | str | P/R/N/D |
| `drive_mode_changed` | str | ECO/CITY/SPORT/… |

### Shift animation

Shift Up / Shift Down triggers a timed animation:
1. Disengage clutch (14 steps × 18 ms)
2. Change gear at midpoint
3. Re-engage clutch (14 steps × 18 ms)

---

## Viewport Renderer

Layered rendering order per frame:

1. **Terrain** — two-octave value noise, cached per cell, green-brown gradient
2. **Grid** — world-space minor (40 px) + major (5×) lines
3. **Chassis rails** — floor outline stroke with wheel-arch notches
4. **Support rails** — two central beams (below floor panel)
5. **Chassis floor** — filled notched polygon (`_C_FLOOR`)
6. **Axles + wheels** — colour-coded by role (steer/drive/both/passive), Ackermann steering angles, animated tread sectors
7. **Brakes** — red pads on inboard tyre face (rounded for single, rectangular for dual)
8. **Transmission block** — green rounded rect with gear-array symbols
9. **Driveshaft** — line from transmission output to each driven axle
10. **Engine block** — yellow rounded rect
11. **Fuel tank** — light-blue rounded rect (rear-anchored)
12. **Body silhouette** — expanded floor outline, solid fill (no wheel cutouts)
13. **Trailer** — V-drawbar + full bed with its own rail/floor/axle/body layers
14. **Compass HUD** — screen-space north indicator

Camera: scroll-wheel zoom (0.1×–20×), right-drag pan, double-click resets vehicle + camera.

---

## Reusable Widgets — `gui/widgets.py`

| Widget | Purpose |
|--------|---------|
| `CollapsibleSection` | Accordion panel with expand/collapse, optional checkbox, subtitle |
| `SectionHeader` | Styled uppercase header bar |
| `ResetSlider` | QSlider that resets to default on double-click |
| `AssetCombo` | Editable QComboBox backed by a JSON asset directory |
| `SteeringWheelWidget` | Rotatable steering wheel graphic (unused in current tab layout) |
| `make_accordion(sections)` | Mutual-exclusion wiring: expanding one collapses others |

---

## Template / Asset System

All templates are JSON files saved to `assets/<category>/`.

- **Frame and Body** (`assets/frame_and_body/`): wheel-frame config + embedded `body` key
- **Engines** (`assets/engines/`): capacity, RPM range, k, VE map, drag, AFR
- **Transmissions** (`assets/transmissions/`): ratios, final drive, eta, clutch, type, drive modes
- **Brakes** (`assets/brakes/`): max torque, front bias, μ

`_TemplateBar` provides Load / Save / Save As / Delete UI for each category. Templates round-trip through JSON without data loss.

---

## Key Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| PyQt6 | 6.7.1 | GUI framework |
| torch | ≥ 2.2.0 | (reserved for RL/training pipelines) |
| stable-baselines3 | latest | (reserved for RL agents) |
| gymnasium | latest | (reserved for RL environments) |
| numpy, scipy, scikit-learn | latest | Numerical / ML utilities |
| matplotlib | latest | Plotting |
| panda3d | 1.10.16 | (reserved for 3-D rendering) |
| pygame | latest | (reserved for alternative rendering) |

> Core simulation + GUI only requires **PyQt6**. Remaining deps are for planned RL / 3D features.

---

## Known Constraints / Notes

- **Circular dependency handling**: Engine and transmission use one-frame lag — engine torque is computed with the *previous* frame's RPM; transmission integrates RPM forward.
- **Physics timestep**: fixed 16 ms. No sub-stepping — large torques at low frame rates may cause instability.
- **No ABS / TCS**: Brake lock-up and traction control are out of scope.
- **2-D only**: All physics are planar (yaw + longitudinal + lateral). No suspension, pitch, or roll.
- **Terrain is cosmetic**: The noise-based terrain has no effect on physics (no slope, no friction variation).
