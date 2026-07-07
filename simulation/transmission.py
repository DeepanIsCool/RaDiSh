from __future__ import annotations

import math


class TransmissionModel:
    """
    Manual transmission model.

    Gear map  : {-1: reverse_ratio, 0: 0.0, 1: r1, 2: r2, ...}
    Update    : call once per physics tick with engine torque + wheel RPM.
    State     : engine_rpm persists between frames (one-frame lag breaks the
                circular dependency with the engine model).

    Clutch engagement e ∈ [0, 1]
        0 = fully disengaged (pedal fully pressed)
        1 = fully locked     (pedal released)
    """

    def __init__(self) -> None:
        # ── Parameters (overwritten by update_from_cfg) ───────────────────────
        self.gear_ratios:      dict[int, float] = {
            -1: -3.32, 0: 0.0,
             1: 3.54, 2: 2.10, 3: 1.48, 4: 1.12, 5: 0.85,
        }
        self.final_drive:      float = 3.90
        self.eta:              float = 0.95   # mechanical efficiency
        self.I_engine:         float = 0.15   # engine-side inertia kg·m²
        self.clutch_torque_max: float = 350.0  # Nm
        # Launch feasibility: when the clutch locks from rest, the engine bogs
        # against the vehicle inertia reflected through the gearing
        # (I_refl = m·r² / GR_tot²). It survives only while I_refl stays below
        # launch_inertia_ratio × I_engine; in a tall gear I_refl is huge and the
        # engine stalls. Self-scales with vehicle mass and tyre radius.
        self.launch_inertia_ratio: float = 20.0

        # ── Persistent state ──────────────────────────────────────────────────
        self.engine_rpm:  float = 800.0
        self.is_stalled:  bool  = False

    # ── Config ────────────────────────────────────────────────────────────────

    def update_from_cfg(self, cfg: dict) -> None:
        fwd  = cfg.get("forward_ratios", [3.54, 2.10, 1.48, 1.12, 0.85])
        rev  = float(cfg.get("reverse_ratio", 3.32))
        self.gear_ratios = {-1: -rev, 0: 0.0}
        for i, r in enumerate(fwd, start=1):
            self.gear_ratios[i] = float(r)
        self.final_drive       = float(cfg.get("final_drive",       3.90))
        self.eta               = float(cfg.get("eta",               0.95))
        self.I_engine          = float(cfg.get("I_engine",          0.15))
        self.clutch_torque_max = float(cfg.get("clutch_torque_max", 350.0))
        self.launch_inertia_ratio = float(cfg.get("launch_inertia_ratio", 20.0))

    # ── Core update ───────────────────────────────────────────────────────────

    def update(self,
               engine_torque: float,
               wheel_rpm:     float,
               gear:          int,
               e:             float,
               dt:            float,
               idle_rpm:      float = 800.0,
               max_rpm:       float = 6000.0,
               mass_kg:       float = 1500.0,
               tyre_radius_m: float = 0.33) -> tuple[float, float]:
        """
        Returns (wheel_torque_nm, engine_rpm).

        engine_torque : Nm from engine model (computed with LAST frame's engine_rpm)
        wheel_rpm     : signed wheel RPM from vehicle dynamics (+ = forward)
        gear          : current gear index (-1 = R, 0 = N, 1..N = forward)
        e             : clutch engagement [0 = disengaged, 1 = locked]
        dt            : timestep (s)
        """
        e = max(0.0, min(1.0, e))

        # ── Stall passthrough ─────────────────────────────────────────────────
        if self.is_stalled:
            return (0.0, 0.0)

        GR = self.gear_ratios.get(gear, 0.0)

        # ── Neutral ───────────────────────────────────────────────────────────
        if GR == 0.0:
            self.engine_rpm = self._free_rev(engine_torque, dt)
            self.engine_rpm = max(idle_rpm, min(max_rpm, self.engine_rpm))
            return (0.0, self.engine_rpm)

        GR_tot = GR * self.final_drive   # combined ratio (signed)

        # ── Clutch torque capacity ────────────────────────────────────────────
        cap = e * self.clutch_torque_max
        # Transfer clamped to engine output and clutch capacity
        tau_clutch = max(-cap, min(cap, engine_torque))

        # ── Engine RPM integration ────────────────────────────────────────────
        tau_net  = engine_torque - tau_clutch
        omega    = self.engine_rpm * 2.0 * math.pi / 60.0
        dw       = tau_net / max(1e-6, self.I_engine)
        new_rpm  = self.engine_rpm + dw * dt * 60.0 / (2.0 * math.pi)

        # Fully locked: the engine is rigidly tied to the wheels at
        # engine_rpm = wheel_rpm × GR_tot.
        if e >= 1.0:
            kinematic_rpm = abs(wheel_rpm) * abs(GR_tot)
            if kinematic_rpm < idle_rpm:
                # Standing / near-stall launch. The locked clutch drags the
                # engine toward the (near-zero) wheel speed; whether it pulls
                # away or bogs to a stall depends on the vehicle inertia
                # reflected through the gearing. In a tall gear I_refl is huge
                # and the engine cannot hold idle → stall.
                I_refl = (mass_kg * tyre_radius_m * tyre_radius_m
                          / max(1e-6, GR_tot * GR_tot))
                if I_refl > self.launch_inertia_ratio * self.I_engine:
                    self.is_stalled = True
                    self.engine_rpm = 0.0
                    return (0.0, 0.0)
                # Gear low enough: engine holds idle and creeps the car away.
                new_rpm = idle_rpm
            else:
                new_rpm = kinematic_rpm

        new_rpm = max(0.0, min(max_rpm, new_rpm))

        # ── Stall detection ───────────────────────────────────────────────────
        # Only stall when the engine is under load (e > 0.1) AND producing no
        # positive torque (engine braking / overrun) AND below idle.
        # When engine_torque > 0 the engine is trying to drive the vehicle from
        # rest — stall must not fire before the wheel torque has a chance to
        # accelerate the drivetrain.
        if e > 0.1 and new_rpm < idle_rpm and engine_torque <= 0.0:
            self.is_stalled = True
            self.engine_rpm = 0.0
            return (0.0, 0.0)

        # ── Rev limiter ───────────────────────────────────────────────────────
        wheel_torque = tau_clutch * GR_tot * self.eta
        if new_rpm >= max_rpm:
            new_rpm      = max_rpm
            wheel_torque = 0.0

        self.engine_rpm = new_rpm
        return (wheel_torque, new_rpm)

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _free_rev(self, engine_torque: float, dt: float) -> float:
        """Integrate engine RPM freely (neutral / clutch fully out)."""
        omega = self.engine_rpm * 2.0 * math.pi / 60.0
        dw    = engine_torque / max(1e-6, self.I_engine)
        return self.engine_rpm + dw * dt * 60.0 / (2.0 * math.pi)
