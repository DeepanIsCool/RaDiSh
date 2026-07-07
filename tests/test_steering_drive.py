"""
Physics test: steering × drive × vehicle template × direction.

Tests that yaw_rate sign and speed sign are correct for every combination.

Run:  conda run -n py310 python tests/test_steering_drive.py
"""

from __future__ import annotations
import math, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# resolve_frame is a pure dict function — no QApplication needed.
# We suppress the Qt import side-effect with a minimal stub for the widgets module.
import types
stub = types.ModuleType("gui.widgets")
stub.AssetCombo = object
sys.modules.setdefault("gui.widgets", stub)

from gui.tabs.vehicle_design.wheel_frame_section import resolve_frame  # noqa: E402

# ── Physics constants (mirror viewport.py) ────────────────────────────────────
_DT       = 0.016
_MASS     = 1_500.0
_IZ       = 2_500.0
_CF       = 60_000.0
_CR       = 60_000.0
_MU       = 0.85


# ── Inline geometry (mirrors viewport._update_geometry) ──────────────────────

def _geometry(res: dict) -> tuple:
    """Return (lf, lr, y_nonsteer, wheelbase, steering_mode)."""
    flen  = res["frame_length_m"]
    axles = res["axles"]
    steer_ys    = [(a["position"] - 0.5) * flen for a in axles if a.get("steerable")]
    nonsteer_ys = [(a["position"] - 0.5) * flen for a in axles if not a.get("steerable")]

    if not steer_ys and nonsteer_ys:
        steer_ys    = [min(nonsteer_ys)]
        nonsteer_ys = [y for y in nonsteer_ys if y != steer_ys[0]]
    if not steer_ys:
        steer_ys = [-flen * 0.35]

    s_mode = res["steering_mode"]

    if not nonsteer_ys:
        y_f = min(steer_ys);  y_r = max(steer_ys)
        return abs(y_f), abs(y_r), 0.0, max(0.5, y_r - y_f), s_mode

    if s_mode == "both":
        y_f = min(steer_ys);  y_r = max(steer_ys)
        y_ns = sum(nonsteer_ys) / len(nonsteer_ys)
        return abs(y_f), abs(y_r), y_ns, max(0.5, y_r - y_f), s_mode

    y_s = sum(steer_ys)    / len(steer_ys)
    y_n = sum(nonsteer_ys) / len(nonsteer_ys)
    return abs(y_s), abs(y_n), y_n, max(0.5, abs(y_n - y_s)), s_mode


# ── Physics simulator ─────────────────────────────────────────────────────────

def simulate(cfg: dict, torque_Nm: float, steer_deg: float,
             n_settle: int = 120, n_steer: int = 180) -> dict:
    """
    Phase 1 (n_settle ticks): apply torque, steer = 0  → reach cruising speed.
    Phase 2 (n_steer  ticks): hold torque + apply steer_deg.
    Returns final state dict.
    """
    res   = resolve_frame(cfg)
    lf, lr, y_nonsteer, wb, s_mode = _geometry(res)
    axles = res["axles"]

    n_drive  = sum(a["wheels"] for a in axles if a.get("drivable"))
    n_total  = max(1, sum(a["wheels"] for a in axles))
    r_tyre   = max(0.01, res["tyre_radius_m"])
    w_fac    = min(1.5, math.sqrt(max(0.05, res["tyre_width_m"]) / 0.20))

    v = vy = yr = 0.0   # speed, lateral vel, yaw-rate

    def _tick(v, vy, yr, steer_active):
        # Longitudinal
        if n_drive > 0 and abs(torque_Nm) > 0.1:
            tau_w  = torque_Nm / n_drive
            F_w    = tau_w / r_tyre
            N_w    = _MASS * 9.81 / n_total
            F_lim  = _MU * N_w * w_fac
            F_act  = F_w if abs(F_w) <= F_lim else math.copysign(F_lim, F_w)
            v     += (F_act * n_drive / _MASS) * _DT
        drag = 0.50 if abs(torque_Nm) < 0.1 else 0.05
        v *= (1.0 - drag * _DT)

        # Slip angles
        if abs(v) >= 0.5 and steer_active:
            sr     = -math.radians(steer_deg)
            if s_mode == "rear":
                df, dr = 0.0, sr
            elif s_mode == "both":
                df, dr = sr, -sr
            else:
                df, dr = sr, 0.0

            sv    = math.copysign(1.0, v)
            vr    = abs(v)
            af    = sv * df - math.atan2(vy + yr * lf, vr)
            ar    = sv * dr - math.atan2(vy - yr * lr, vr)
        else:
            af = ar = 0.0

        # Lateral dynamics
        L    = max(0.1, lf + lr)
        Fzf  = _MASS * 9.81 * lr / L
        Fzr  = _MASS * 9.81 * lf / L
        def sat(F, Fz): return max(-_MU * Fz, min(_MU * Fz, F))
        Fyf  = sat(_CF * af, Fzf)
        Fyr  = sat(_CR * ar, Fzr)

        vy  += ((Fyf + Fyr) / _MASS - v * yr) * _DT
        yr  += ((lf * Fyf - lr * Fyr) / _IZ)  * _DT

        if abs(v) > 0.1:
            lim = abs(v) * math.tan(math.radians(45))
            vy  = max(-lim, min(lim, vy))
        if abs(v) < 2.0:
            fade = 1.0 - abs(v) / 2.0
            k = math.exp(-fade * 10.0 * _DT)
            vy *= k;  yr *= k
        return v, vy, yr

    for _ in range(n_settle):
        v, vy, yr = _tick(v, vy, yr, steer_active=False)
    for _ in range(n_steer):
        v, vy, yr = _tick(v, vy, yr, steer_active=True)

    return {"v": v, "vy": vy, "yaw_rate": yr}


# ── Vehicle templates ─────────────────────────────────────────────────────────

BASE = dict(tyre_radius_cm=33, tyre_width_cm=22, differential="open")

VEHICLES = {
    "bike      (1+1)": {**BASE,
        "frame_length_m": 2.5, "frame_width_m": 0.8,
        "tyre_radius_cm": 30,  "tyre_width_cm": 10,
        "groups": {
            "front":  {"axle_count": 1, "position_pct": 15, "wheels_per_axle": 1},
            "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
            "rear":   {"axle_count": 1, "position_pct": 85, "wheels_per_axle": 1},
        },
    },
    "tuktuk    (1+2)": {**BASE,
        "frame_length_m": 3.0, "frame_width_m": 1.4,
        "tyre_radius_cm": 28,  "tyre_width_cm": 15,
        "groups": {
            "front":  {"axle_count": 1, "position_pct": 15, "wheels_per_axle": 1},
            "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
            "rear":   {"axle_count": 1, "position_pct": 85, "wheels_per_axle": 2},
        },
    },
    "car       (2+2)": {**BASE,
        "frame_length_m": 4.0, "frame_width_m": 1.8,
        "groups": {
            "front":  {"axle_count": 1, "position_pct": 20, "wheels_per_axle": 2},
            "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
            "rear":   {"axle_count": 1, "position_pct": 80, "wheels_per_axle": 2},
        },
    },
    "truck     (2+4)": {**BASE,
        "frame_length_m": 8.0, "frame_width_m": 2.5,
        "tyre_radius_cm": 50,  "tyre_width_cm": 30,
        "groups": {
            "front":  {"axle_count": 1, "position_pct": 12, "wheels_per_axle": 2},
            "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 4},
            "rear":   {"axle_count": 1, "position_pct": 85, "wheels_per_axle": 4},
        },
    },
    "long truck (2+4+4)": {**BASE,
        "frame_length_m": 12.0, "frame_width_m": 2.5,
        "tyre_radius_cm": 50,   "tyre_width_cm": 30,
        "groups": {
            "front":  {"axle_count": 1, "position_pct":  8, "wheels_per_axle": 2},
            "middle": {"axle_count": 1, "position_pct": 50, "wheels_per_axle": 4},
            "rear":   {"axle_count": 1, "position_pct": 90, "wheels_per_axle": 4},
        },
    },
}

STEER_MODES = ["front", "rear", "both"]
DRIVE_MODES = ["front", "rear", "both"]

TORQUE_FWD = +1500.0   # Nm — forward
TORQUE_REV = -1500.0   # Nm — reverse
STEER_RIGHT = +40.0    # degrees
STEER_LEFT  = -40.0

# Expected yaw_rate sign for (steer_mode, direction, steer_side)
# + = CCW (left),  − = CW (right)
# Front right → CW (−), rear right → CCW (+), both right → CW (−, stronger)
EXPECTED = {
    #           fwd_right  fwd_left  rev_right  rev_left
    "front": (  -1,       +1,       +1,        -1  ),
    "rear":  (  +1,       -1,       -1,        +1  ),
    "both":  (  -1,       +1,       +1,        -1  ),
}


# ── Runner ────────────────────────────────────────────────────────────────────

def _sign(x: float) -> int:
    return 1 if x > 1e-4 else (-1 if x < -1e-4 else 0)


def run_all():
    PASS = "✓"; FAIL = "✗"
    total = fails = 0

    for vname, vbase in VEHICLES.items():
        print(f"\n{'═'*70}")
        print(f"  {vname}")
        print(f"{'═'*70}")

        for s_mode in STEER_MODES:
            for d_mode in DRIVE_MODES:
                cfg = {**vbase, "steering_mode": s_mode, "drive_mode": d_mode}
                res = resolve_frame(cfg)
                lf, lr, y_ns, wb, _ = _geometry(res)
                n_drive = sum(a["wheels"] for a in res["axles"] if a.get("drivable"))

                # Skip if no drivable wheels (can't test motion)
                if n_drive == 0:
                    continue

                ex = EXPECTED[s_mode]     # (fwd_r, fwd_l, rev_r, rev_l)
                cases = [
                    ("FWD+R", TORQUE_FWD, STEER_RIGHT, ex[0], +1),
                    ("FWD+L", TORQUE_FWD, STEER_LEFT,  ex[1], +1),
                    ("REV+R", TORQUE_REV, STEER_RIGHT, ex[2], -1),
                    ("REV+L", TORQUE_REV, STEER_LEFT,  ex[3], -1),
                ]

                results = []
                for label, torque, steer, exp_yaw_sign, exp_v_sign in cases:
                    st = simulate(cfg, torque, steer)
                    got_v    = _sign(st["v"])
                    got_yaw  = _sign(st["yaw_rate"])
                    v_ok  = (got_v == exp_v_sign)
                    yr_ok = (got_yaw == exp_yaw_sign)
                    ok = v_ok and yr_ok

                    total += 1
                    if not ok:
                        fails += 1

                    tag = PASS if ok else FAIL
                    v_tag  = "" if v_ok  else f"[v_sign={got_v} exp={exp_v_sign}]"
                    yr_tag = "" if yr_ok else f"[yaw_sign={got_yaw} exp={exp_yaw_sign}]"
                    results.append(
                        f"    {label}  v={st['v']:+.2f} m/s  "
                        f"yaw={st['yaw_rate']:+.3f} r/s  "
                        f"{tag} {v_tag}{yr_tag}"
                    )

                header = (f"  steer={s_mode:<5}  drive={d_mode:<5}  "
                          f"wb={wb:.2f}m  lf={lf:.2f}  lr={lr:.2f}  "
                          f"n_drive={n_drive}")
                print(header)
                for r in results:
                    print(r)

    print(f"\n{'═'*70}")
    print(f"  RESULT: {total - fails}/{total} passed"
          f"{'  — ALL OK' if fails == 0 else f'  — {fails} FAILED'}")
    print(f"{'═'*70}\n")
    return fails


if __name__ == "__main__":
    sys.exit(run_all())
