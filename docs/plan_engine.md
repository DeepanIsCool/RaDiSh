# Engine Model — Design Plan

## 1. Overview

The engine model translates driver accelerator input into wheel torque via a physically
grounded thermodynamic pipeline. It runs alongside a direct torque bypass mode for
isolated dynamics testing.

---

## 2. Physics Model

### 2.1 Corrected Torque Formula

Torque is power divided by angular velocity, not power itself. The key fix from the
naive model is the `1/ω` term:

```
τ = (ṁ_air × k) / ω
```

where `ω = 2π N / 60` (rad/s).

Substituting the airflow equation, N cancels out and torque becomes:

```
τ = (V_d × ρ_air × f_VE(N) × α × k) / (4π)
```

This means the torque curve shape is entirely determined by the VE map.
Peak torque occurs where VE peaks at full throttle.

### 2.2 Full Pipeline (per simulation frame)

| Step | Equation | Notes |
|------|----------|-------|
| 1. Lookup VE | `VE = f_VE(N)` | Interpolate from VE map at current RPM |
| 2. Airflow | `ṁ_air = (V_d/2) × (N/60) × ρ_air × VE × α` | V_d in m³ (convert from L × 0.001) |
| 3. Fuel rate | `ṁ_fuel = ṁ_air / AFR_target` | Used for fuel consumption tracking |
| 4. Torque | `τ = ṁ_air × k / ω` | ω = 2πN/60 |
| 5. Engine braking | `τ = −C_drag × N` | Applied when α = 0 |

### 2.3 Engine Braking

When throttle is fully closed (α = 0), combustion torque is replaced by braking torque:

```
τ_braking = −C_drag × N
```

`C_drag` captures internal friction and manifold vacuum resistance.
A smooth transition (partial throttle) is a future improvement.

### 2.4 Constants

| Symbol | Value | Note |
|--------|-------|------|
| `ρ_air` | 1.2 kg/m³ | Hardcoded — sea level standard |
| `AFR_target` | 14.7 (default) | Stoichiometric for gasoline; user-configurable |

---

## 3. Parameters

### 3.1 Engine Configuration (right panel)

| Parameter | UI | Range | Default | Notes |
|-----------|----|-------|---------|-------|
| Engine Capacity `V_d` | Spinner (L) | 0.5 – 8.0 L, step 0.1 | 1.2 | Converted to m³ internally |
| Max RPM (Redline) | Spinner (RPM) | 3000 – 12000, step 500 | 6000 | Controls VE graph x-axis extent |
| Idle RPM | Spinner (RPM) | 400 – 1500, step 50 | 800 | Minimum sustained RPM |
| Combustion Constant `k` | Spinner | TBD | TBD | Mutually linked with Peak Torque (see §3.2) |
| Peak Torque | Spinner (Nm) | TBD | TBD | Mutually linked with k (see §3.2) |
| Engine Drag `C_drag` | Spinner | TBD | TBD | Controls engine braking intensity |
| Target AFR | Spinner | 10.0 – 20.0, step 0.1 | 14.7 | |
| VE Map | 2D interactive graph | — | — | See §3.3 |

**Derived read-only display** (below k / Peak Torque inputs):
- Peak Power (kW) — computed live from current k, V_d, VE_max

### 3.2 k ↔ Peak Torque Mutual Link

The relationship between k and Peak Torque is:

```
τ_peak = (V_d × ρ_air × VE_max × k) / (4π)
k      = (τ_peak × 4π) / (V_d × ρ_air × VE_max)
```

**Rules:**
- `k` is the primary parameter — represents the engine's intrinsic thermal scaling.
- `τ_peak` is derived and updates live whenever k, V_d, or the VE map changes.
- When the user edits `τ_peak` directly → back-calculate k and store as the new primary.
- When V_d or VE map changes → k is held fixed, τ_peak updates automatically.

### 3.3 VE Map

- **X-axis:** 0 RPM to Max RPM, one draggable point every 1000 RPM.
- **Y-axis:** Volumetric Efficiency 0 – 100%.
- **Interaction:** drag points vertically; a smooth curve is drawn through all points
  (linear interpolation minimum; cubic spline preferred).
- **Default shape:** typical naturally-aspirated gasoline curve — rises from ~60% at idle,
  peaks ~80% at mid-RPM, falls back to ~60% at redline.

---

## 4. Direct Control Panel

### 4.1 Mode Toggle

The panel has two **mutually exclusive** modes selected by a toggle switch:

| Mode | Accelerator slider | Torque slider |
|------|--------------------|---------------|
| **Direct** | Greyed out / inactive | User-controlled input |
| **Engine** | User-controlled input | Read-only — mirrors engine output live |

### 4.2 Mode Switching Behaviour

- **Engine → Direct:** torque slider snaps to the last engine-output torque value
  (no discontinuity in wheel torque).
- **Direct → Engine:** engine takes over immediately from wherever the accelerator is set.

### 4.3 Purpose of Each Mode

- **Direct Torque** — bypasses the engine model entirely. Used to test wheel behaviour,
  suspension, and vehicle dynamics in isolation with a known, fixed torque.
- **Engine (Accelerator)** — full engine pipeline active. Torque is an output of the
  model, not a user input. Torque slider becomes a live telemetry readout.

---

## 5. Out of Scope (Future)

- Altitude / air density variation (`ρ_air` configurable).
- Partial-throttle engine braking (smooth transition, not hard switch at α = 0).
- Turbocharging / supercharging (modifies effective VE beyond 100%).
- Gear-dependent RPM state (requires full drivetrain dynamics).
- RPM as a state variable — currently set by the RPM slider; will eventually be driven
  by engine–load dynamics once drivetrain inertia is modelled.
