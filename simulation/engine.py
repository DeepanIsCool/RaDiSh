from __future__ import annotations

import math


class EngineModel:
    """
    4-stroke internal combustion engine model.

    Core equation (per simulation frame):
        ṁ_air = (V_d/2) × (N/60) × ρ_air × VE(N) × α
        τ     = ṁ_air × k / ω          where ω = 2πN/60
              = (V_d × ρ_air × VE(N) × α × k) / (4π)

    N cancels, so torque curve shape is entirely determined by the VE map.
    Engine braking at α = 0: τ = −C_drag × N
    """

    RHO_AIR = 1.2       # kg/m³ — sea-level standard air density (fixed)

    # Default VE map for a naturally-aspirated gasoline engine
    _DEFAULT_VE: list[list[float]] = [
        [0,    0.60],
        [1000, 0.65],
        [2000, 0.72],
        [3000, 0.80],
        [4000, 0.85],
        [5000, 0.80],
        [6000, 0.68],
    ]

    def __init__(self) -> None:
        self.capacity_l: float = 2.0        # engine displacement, litres
        self.max_rpm:    float = 6000.0     # redline
        self.idle_rpm:   float = 800.0      # minimum sustained RPM
        self.k:          float = 1_232_000.0  # combustion constant (J·s/kg)
        self.c_drag:     float = 0.05       # engine braking drag (Nm/RPM)
        self.afr_target: float = 14.7       # stoichiometric AFR
        self.ve_map: list[list[float]] = [list(row) for row in self._DEFAULT_VE]

    # ── Config ────────────────────────────────────────────────────────────────

    def update_from_cfg(self, cfg: dict) -> None:
        """Apply a config dict (as emitted by EngineConfigBody)."""
        self.capacity_l = float(cfg.get("capacity_l",  self.capacity_l))
        self.max_rpm    = float(cfg.get("max_rpm",     self.max_rpm))
        self.idle_rpm   = float(cfg.get("idle_rpm",    self.idle_rpm))
        self.k          = float(cfg.get("k",           self.k))
        self.c_drag     = float(cfg.get("c_drag",      self.c_drag))
        self.afr_target = float(cfg.get("afr_target",  self.afr_target))
        ve = cfg.get("ve_map")
        if ve:
            self.ve_map = [list(row) for row in ve]

    # ── VE lookup ─────────────────────────────────────────────────────────────

    def interp_ve(self, rpm: float) -> float:
        """Linear interpolation through the VE map."""
        pts = self.ve_map
        if not pts:
            return 0.0
        if rpm <= pts[0][0]:
            return pts[0][1]
        if rpm >= pts[-1][0]:
            return pts[-1][1]
        for i in range(len(pts) - 1):
            r0, v0 = pts[i]
            r1, v1 = pts[i + 1]
            if r0 <= rpm <= r1:
                t = (rpm - r0) / (r1 - r0)
                return v0 + t * (v1 - v0)
        return pts[-1][1]

    # ── Torque ────────────────────────────────────────────────────────────────

    def compute_torque(self, rpm: float, alpha: float) -> float:
        """
        Net torque (Nm) at the given RPM and throttle position α ∈ [0, 1].
        Returns negative torque for engine braking when α ≤ 0.
        """
        rpm   = max(0.0, rpm)
        alpha = max(0.0, min(1.0, alpha))

        if rpm < 1.0:
            return 0.0

        if alpha <= 0.0:
            if rpm <= self.idle_rpm:
                # Engine idling: ECU injects just enough fuel to hold idle speed.
                # Model as a tiny effective alpha — produces gentle creep torque.
                _idle_alpha = 0.05
                ve    = self.interp_ve(self.idle_rpm)
                omega = 2.0 * math.pi * self.idle_rpm / 60.0
                m_air = (self.capacity_l * 0.001 / 2.0) * (self.idle_rpm / 60.0) \
                        * self.RHO_AIR * ve * _idle_alpha
                return m_air * self.k / omega
            else:
                # Overrun: wheels spinning engine above idle → resist only the excess.
                return -self.c_drag * (rpm - self.idle_rpm)

        ve      = self.interp_ve(rpm)
        omega   = 2.0 * math.pi * rpm / 60.0
        m_air   = (self.capacity_l * 0.001 / 2.0) * (rpm / 60.0) * self.RHO_AIR * ve * alpha
        return m_air * self.k / omega

    def compute_fuel_rate(self, rpm: float, alpha: float) -> float:
        """Fuel mass flow rate (kg/s)."""
        rpm   = max(0.0, rpm)
        alpha = max(0.0, min(1.0, alpha))
        if rpm < 1.0 or alpha <= 0.0:
            return 0.0
        ve    = self.interp_ve(rpm)
        m_air = (self.capacity_l * 0.001 / 2.0) * (rpm / 60.0) * self.RHO_AIR * ve * alpha
        return m_air / max(0.1, self.afr_target)

    # ── Derived performance figures ───────────────────────────────────────────

    def ve_max(self) -> float:
        return max(v for _, v in self.ve_map) if self.ve_map else 1.0

    def peak_torque_nm(self) -> float:
        """Peak torque (Nm) at α = 1 at the VE-map peak."""
        return (self.capacity_l * 0.001 * self.RHO_AIR * self.ve_max() * self.k) / (4.0 * math.pi)

    def peak_power_kw(self) -> float:
        """
        Peak power (kW). Power = ṁ_air × k; maximised where VE × N is largest
        (i.e. the right-most high-VE point on the map).
        """
        best = 0.0
        for rpm, ve in self.ve_map:
            if rpm <= 0:
                continue
            m_air = (self.capacity_l * 0.001 / 2.0) * (rpm / 60.0) * self.RHO_AIR * ve
            best  = max(best, m_air * self.k)
        return best / 1000.0

    def k_from_peak_torque(self, tau_nm: float) -> float:
        """Back-calculate k to achieve the given peak torque at α = 1."""
        denom = (self.capacity_l * 0.001 * self.RHO_AIR * self.ve_max()) / (4.0 * math.pi)
        return tau_nm / denom if denom > 1e-12 else 0.0

    def k_from_peak_power(self, power_kw: float) -> float:
        """Back-calculate k to achieve the given peak power at α = 1."""
        best_m_air = 0.0
        for rpm, ve in self.ve_map:
            if rpm <= 0:
                continue
            m_air = (self.capacity_l * 0.001 / 2.0) * (rpm / 60.0) * self.RHO_AIR * ve
            best_m_air = max(best_m_air, m_air)
        return (power_kw * 1000.0) / best_m_air if best_m_air > 1e-12 else 0.0
