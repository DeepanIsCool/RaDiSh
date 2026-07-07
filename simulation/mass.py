"""
Component mass estimators — single source of truth for how each vehicle
component contributes to total mass, driven by its chosen parameters.

The wheel frame computes its own mass/inertia in
``gui.tabs.vehicle_design.wheel_frame_section.resolve_frame`` (steel-rail model);
the functions here cover the remaining components so the UI labels and the
viewport physics agree on one set of formulas.

All return kilograms.
"""
from __future__ import annotations

# Areal density of the body shell (panels + structure) per m² of footprint.
_BODY_AREAL_DENSITY = 55.0   # kg/m²


def engine_mass(cfg: dict) -> float:
    """Larger displacement → heavier block / heads / internals."""
    v_d = float(cfg.get("capacity_l", 2.0))
    return 40.0 + 55.0 * v_d


def transmission_mass(cfg: dict) -> float:
    """More gears → more shafts, synchros and a larger case.

    Automatics add a torque converter, valve body and pump (~30 kg).
    """
    n = int(cfg.get("n_forward_gears", 5))
    base = 35.0 + 9.0 * n          # case + clutch/reverse + per-gear
    if cfg.get("trans_type", "manual") == "automatic":
        base += 30.0
    return base


def brake_mass(cfg: dict) -> float:
    """Bigger brakes (higher torque capacity) → larger rotors and calipers."""
    max_torque = float(cfg.get("max_brake_torque", 3500.0))
    return 6.0 + 0.009 * max_torque


def body_mass(cfg: dict, frame_length_m: float, frame_width_m: float) -> float:
    """
    Larger body → more panelling. Footprint is the wheel-frame extent grown by
    the front/rear/side overhangs from the Vehicle Body config.
    """
    fo = float(cfg.get("front_overhang", 0.0))
    ro = float(cfg.get("rear_overhang",  0.0))
    so = float(cfg.get("side_overhang",  0.0))
    length = max(0.1, frame_length_m + fo + ro)
    width  = max(0.1, frame_width_m + 2.0 * so)
    return _BODY_AREAL_DENSITY * length * width
