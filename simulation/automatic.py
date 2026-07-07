from __future__ import annotations

import math


# Shift-schedule presets. Each mode sets where the controller up/downshifts as a
# fraction of the idle→redline RPM span, plus the minimum time between shifts.
#   up RPM   = idle + span × (up_base + up_span × throttle)
#   down RPM = idle + span × (dn_base + dn_span × throttle)
_MODE_KEYS = ("up_base", "up_span", "dn_base", "dn_span", "shift_time")

DEFAULT_DRIVE_MODES: dict[str, dict] = {
    "ECO":   {"up_base": 0.20, "up_span": 0.35, "dn_base": 0.06,
              "dn_span": 0.18, "shift_time": 0.60},
    "CITY":  {"up_base": 0.30, "up_span": 0.55, "dn_base": 0.10,
              "dn_span": 0.25, "shift_time": 0.40},
    "SPORT": {"up_base": 0.55, "up_span": 0.42, "dn_base": 0.22,
              "dn_span": 0.30, "shift_time": 0.25},
}
DEFAULT_DRIVE_MODE = "CITY"


class AutomaticTransmissionModel:
    """
    Automatic transmission: torque-converter coupling + automatic shift control.

    Reuses the same gear-ratio / final-drive / efficiency math as the manual
    ``TransmissionModel`` but replaces the driver clutch and gear selection:

      • Coupling — a fluid torque converter. Engine (pump) and gearbox input
        (turbine) spin at different speeds. At low speed ratio it multiplies
        torque (TR up to ``stall_torque_ratio``) and loads the engine via a
        pump torque ∝ Nᵉ², so the engine settles at a stall speed and the car
        creeps from rest with no stall. A lock-up clutch rigidly couples the
        two above ``lockup_sr`` to remove fluid-slip losses at cruise.

      • Gear selection — a controller picks the gear in Drive from engine RPM
        and throttle, with throttle-dependent shift points, hysteresis and a
        cooldown so it doesn't hunt. Park/Reverse/Neutral map to fixed gears.

    Drive range : "P" | "R" | "N" | "D"
    The engine never stalls (RPM is clamped to idle), as in a real automatic.
    """

    def __init__(self) -> None:
        # ── Geartrain (shared form with the manual model) ─────────────────────
        self.gear_ratios: dict[int, float] = {
            -1: -3.32, 0: 0.0,
             1: 3.54, 2: 2.10, 3: 1.48, 4: 1.12, 5: 0.85,
        }
        self.final_drive: float = 3.90
        self.eta:         float = 0.95
        self.I_engine:    float = 0.18   # auto trans: slightly more rotating mass

        # ── Torque converter ──────────────────────────────────────────────────
        self.converter_capacity: float = 4.0e-5  # pump load coeff [Nm / rpm²]
        self.stall_torque_ratio: float = 2.0     # TR at zero speed ratio
        self.coupling_sr:        float = 0.86    # SR where TR reaches 1.0
        self.lockup_sr:          float = 0.90    # engage lock-up clutch above this
        self.unlock_sr:          float = 0.80    # release lock-up below this

        # ── Drive modes + active shift schedule ───────────────────────────────
        self.modes: dict[str, dict] = {
            name: dict(params) for name, params in DEFAULT_DRIVE_MODES.items()
        }
        self.drive_mode: str = DEFAULT_DRIVE_MODE
        # Active schedule (fractions of the idle→redline RPM span):
        self.up_base = self.up_span = self.dn_base = self.dn_span = 0.0
        self.shift_time: float = 0.4
        self.set_drive_mode(DEFAULT_DRIVE_MODE)

        # ── State ──────────────────────────────────────────────────────────────
        self.engine_rpm: float = 800.0
        self.gear:       int   = 0
        self.lockup:     bool  = False
        self._cooldown:  float = 0.0

    # ── Drive mode ──────────────────────────────────────────────────────────────

    def set_drive_mode(self, name: str) -> None:
        """Apply a named mode's shift schedule. Unknown names are ignored."""
        if name not in self.modes:
            return
        self.drive_mode = name
        p = self.modes[name]
        self.up_base    = float(p.get("up_base", 0.30))
        self.up_span    = float(p.get("up_span", 0.55))
        self.dn_base    = float(p.get("dn_base", 0.10))
        self.dn_span    = float(p.get("dn_span", 0.25))
        self.shift_time = float(p.get("shift_time", 0.40))

    # ── Config ────────────────────────────────────────────────────────────────

    def update_from_cfg(self, cfg: dict) -> None:
        fwd = cfg.get("forward_ratios", [3.54, 2.10, 1.48, 1.12, 0.85])
        rev = float(cfg.get("reverse_ratio", 3.32))
        self.gear_ratios = {-1: -rev, 0: 0.0}
        for i, r in enumerate(fwd, start=1):
            self.gear_ratios[i] = float(r)
        self.final_drive = float(cfg.get("final_drive", 3.90))
        self.eta         = float(cfg.get("eta",         0.95))
        self.I_engine    = float(cfg.get("I_engine",    0.18))
        self.converter_capacity = float(cfg.get("converter_capacity", self.converter_capacity))
        self.stall_torque_ratio = float(cfg.get("stall_torque_ratio", self.stall_torque_ratio))

        # ── Drive modes ──────────────────────────────────────────────────────
        modes = cfg.get("drive_modes")
        if modes:
            built: dict[str, dict] = {}
            for m in modes:
                name = str(m.get("name", "")).strip()
                if name:
                    built[name] = {k: float(m.get(k, DEFAULT_DRIVE_MODES["CITY"][k]))
                                   for k in _MODE_KEYS}
            if built:
                self.modes = built
        # Keep the live selection if it still exists, else fall back to the
        # configured default, else the first mode.
        default = cfg.get("default_drive_mode", self.drive_mode)
        if self.drive_mode in self.modes:
            target = self.drive_mode
        elif default in self.modes:
            target = default
        else:
            target = next(iter(self.modes))
        self.set_drive_mode(target)

    # ── Core update ─────────────────────────────────────────────────────────────

    def update(self,
               engine_torque: float,
               wheel_rpm:     float,
               drive_range:   str,
               throttle:      float,
               dt:            float,
               idle_rpm:      float = 800.0,
               max_rpm:       float = 6000.0) -> tuple[float, float, int, bool]:
        """
        Returns (wheel_torque_nm, engine_rpm, gear, lockup).

        drive_range : "P" | "R" | "N" | "D"
        throttle    : α ∈ [0, 1]
        """
        self._cooldown = max(0.0, self._cooldown - dt)
        throttle = max(0.0, min(1.0, throttle))

        # ── Range → gear ──────────────────────────────────────────────────────
        if drive_range in ("N", "P"):
            self.gear = 0
        elif drive_range == "R":
            self.gear = -1
        else:  # "D"
            if self.gear <= 0:
                self.gear = 1
            self._shift_logic(throttle, idle_rpm, max_rpm)

        GR = self.gear_ratios.get(self.gear, 0.0)

        # ── Neutral / Park: converter unloaded, engine free-revs ──────────────
        if GR == 0.0:
            self.lockup = False
            self.engine_rpm = self._integrate_engine(engine_torque, dt, idle_rpm, max_rpm)
            return (0.0, self.engine_rpm, self.gear, False)

        GR_tot      = GR * self.final_drive          # signed combined ratio
        turbine_rpm = abs(wheel_rpm) * abs(GR_tot)   # gearbox-input speed
        sr          = min(1.0, turbine_rpm / max(1.0, self.engine_rpm))

        # ── Lock-up clutch hysteresis (only in higher gears) ──────────────────
        if self.lockup and (sr < self.unlock_sr or self.gear < 2):
            self.lockup = False
        elif not self.lockup and sr >= self.lockup_sr and self.gear >= 2:
            self.lockup = True

        if self.lockup:
            # Rigid coupling: engine tied to turbine, full torque passes through.
            self.engine_rpm = max(idle_rpm, min(max_rpm, turbine_rpm))
            tau_turbine     = engine_torque
        else:
            # Fluid coupling: pump load ∝ Nᵉ² and falls to zero as SR → 1;
            # turbine torque is multiplied by TR (≥ 1) toward the wheels.
            tr        = self._torque_ratio(sr)
            tau_pump  = self.converter_capacity * self.engine_rpm ** 2 * max(0.0, 1.0 - sr * sr)
            tau_turbine = tr * tau_pump
            self.engine_rpm = self._integrate_engine(engine_torque - tau_pump,
                                                     dt, idle_rpm, max_rpm)

        wheel_torque = tau_turbine * GR_tot * self.eta
        return (wheel_torque, self.engine_rpm, self.gear, self.lockup)

    # ── Internals ───────────────────────────────────────────────────────────────

    def _integrate_engine(self, net_torque: float, dt: float,
                          idle_rpm: float, max_rpm: float) -> float:
        dw = net_torque / max(1e-6, self.I_engine)          # rad/s²
        rpm = self.engine_rpm + dw * dt * 60.0 / (2.0 * math.pi)
        return max(idle_rpm, min(max_rpm, rpm))             # never stalls

    def _torque_ratio(self, sr: float) -> float:
        """Linear TR: stall_torque_ratio at SR=0 → 1.0 at the coupling point."""
        if sr >= self.coupling_sr:
            return 1.0
        return self.stall_torque_ratio + \
            (1.0 - self.stall_torque_ratio) * (sr / self.coupling_sr)

    def _top_gear(self) -> int:
        return max((g for g in self.gear_ratios if g > 0), default=1)

    def _shift_logic(self, throttle: float, idle_rpm: float, max_rpm: float) -> None:
        if self._cooldown > 0.0:
            return
        span = max(1.0, max_rpm - idle_rpm)
        up_rpm = idle_rpm + span * (self.up_base + self.up_span * throttle)
        dn_rpm = idle_rpm + span * (self.dn_base + self.dn_span * throttle)
        top = self._top_gear()
        if self.gear < top and self.engine_rpm > up_rpm:
            self.gear += 1
            self._cooldown = self.shift_time
        elif self.gear > 1 and self.engine_rpm < dn_rpm:
            self.gear -= 1
            self._cooldown = self.shift_time
