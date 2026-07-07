from __future__ import annotations


class BrakeModel:
    """
    Friction brake model.

    Produces a retarding force opposing vehicle motion. Pedal input b ∈ [0, 1]
    scales the maximum clamping torque; front/rear bias splits it across axles.
    Grip-limited per the same μ ceiling used by the drive model — beyond it the
    wheel would lock (lock-up / ABS is out of scope for now).

    Independent of clutch / gear: the friction brake acts directly on the
    wheels, so it works in Neutral and when the engine is stalled.
    """

    def __init__(self) -> None:
        self.max_torque:  float = 3500.0   # Nm, total at full pedal (all axles)
        self.front_bias:  float = 0.65     # fraction to front axle (0..1)
        self.mu:          float = 1.0      # tyre-road friction (matches _DYN_MU)

    def update_from_cfg(self, cfg: dict) -> None:
        self.max_torque = float(cfg.get("max_brake_torque", 3500.0))
        self.front_bias = float(cfg.get("front_bias",       0.65))
        self.mu         = float(cfg.get("brake_mu",         1.0))

    def decel_force(self, brake: float, veh_speed: float,
                    mass_kg: float, tyre_radius_m: float) -> float:
        """
        Return a longitudinal force (N, positive magnitude) opposing motion.
        The caller applies the sign against veh_speed and clamps to a stop.
        """
        brake = max(0.0, min(1.0, brake))
        if brake <= 0.0 or abs(veh_speed) < 1e-4:
            return 0.0
        r          = max(0.01, tyre_radius_m)
        torque     = brake * self.max_torque
        force      = torque / r                       # N at the contact patch
        grip_limit = self.mu * mass_kg * 9.81         # can't exceed total grip
        return min(force, grip_limit)
