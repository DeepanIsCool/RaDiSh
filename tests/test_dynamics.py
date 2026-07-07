"""
Dynamics test: torque levels, RPM, tyre slip, skid detection,
and oversteer / understeer tendency.

Run:  conda run -n py310 python tests/test_dynamics.py

Physics assumptions (same as viewport.py):
  - Static weight distribution (no longitudinal weight transfer)
  - Simplified tyre: Coulomb friction ceiling with tyre-width efficiency factor
  - Driven wheel has its own angular inertia; diverges from vehicle speed when slipping
  - Passive wheels always kinematic (ω = v / r)
  - Bicycle model for lateral dynamics (Fy capped at μ × Fz)
"""

from __future__ import annotations
import math, sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import types
_stub = types.ModuleType("gui.widgets")
_stub.AssetCombo = object
sys.modules.setdefault("gui.widgets", _stub)

from gui.tabs.vehicle_design.wheel_frame_section import resolve_frame  # noqa

# ── Constants (mirror viewport.py) ────────────────────────────────────────────
_DT   = 0.016
_MASS = 1_500.0
_IZ   = 2_500.0
_CF   = 60_000.0
_CR   = 60_000.0
_MU   = 0.85
_M_WHEEL = 12.0           # kg per wheel (approx, for inertia calc)
_TWO_PI  = 2.0 * math.pi


# ── Geometry (same as test_steering_drive.py) ─────────────────────────────────

def _geometry(res: dict):
    flen  = res["frame_length_m"]
    axles = res["axles"]
    s_mode = res["steering_mode"]

    steer_ys    = [(a["position"] - 0.5) * flen for a in axles if a.get("steerable")]
    nonsteer_ys = [(a["position"] - 0.5) * flen for a in axles if not a.get("steerable")]

    if not steer_ys and nonsteer_ys:
        steer_ys    = [min(nonsteer_ys)]
        nonsteer_ys = [y for y in nonsteer_ys if y != steer_ys[0]]
    if not steer_ys:
        steer_ys = [-flen * 0.35]

    if not nonsteer_ys:
        yf, yr = min(steer_ys), max(steer_ys)
        return abs(yf), abs(yr), 0.0, max(0.5, yr - yf)

    if s_mode == "both":
        yf, yr = min(steer_ys), max(steer_ys)
        yns = sum(nonsteer_ys) / len(nonsteer_ys)
        return abs(yf), abs(yr), yns, max(0.5, yr - yf)

    ys = sum(steer_ys)    / len(steer_ys)
    yn = sum(nonsteer_ys) / len(nonsteer_ys)
    return abs(ys), abs(yn), yn, max(0.5, abs(yn - ys))


# ── Core simulator with wheel-omega tracking ──────────────────────────────────

def simulate(cfg: dict, torque_Nm: float, steer_deg: float,
             n_settle: int = 200, n_steer: int = 300) -> dict:
    """
    Phase 1 (n_settle): constant torque, zero steer  → reach cruising speed.
    Phase 2 (n_steer):  constant torque + steer_deg   → cornering dynamics.

    Returned keys
    -------------
    v            : final vehicle speed (m/s, + forward)
    omega_w      : final driven-wheel angular velocity (rad/s, + forward)
    driven_rpm   : |omega_w| × 60 / (2π)
    passive_rpm  : |v / r_tyre| × 60 / (2π)   (kinematic, always matches vehicle)
    slip_ratio   : (ω_w·r − v) / max(|ω_w·r|, |v|, ε)  — 0 = no slip, >0 = traction slip
    F_w_last     : last per-wheel drive force attempt (N)
    F_lim        : per-wheel friction ceiling (N)
    slipping     : True if |F_w_last| > F_lim
    skidding     : True if slip_ratio > 0.30 (severe slip, wheel essentially free-spinning)
    alpha_f      : front slip angle (rad) at final tick
    alpha_r      : rear  slip angle (rad) at final tick
    yaw_rate     : final yaw rate (rad/s)
    """
    res    = resolve_frame(cfg)
    lf, lr, y_ns, wb = _geometry(res)
    axles  = res["axles"]
    s_mode = res["steering_mode"]

    n_drive = sum(a["wheels"] for a in axles if a.get("drivable"))
    n_total = max(1, sum(a["wheels"] for a in axles))
    r       = max(0.01, res["tyre_radius_m"])
    w_fac   = min(1.5, math.sqrt(max(0.05, res["tyre_width_m"]) / 0.20))
    I_w     = 0.5 * _M_WHEEL * r * r    # single driven-wheel moment of inertia

    # per-wheel friction limit (static weight distribution, no weight transfer)
    N_w   = _MASS * 9.81 / n_total
    F_lim = _MU * N_w * w_fac

    v = vy = yr_rate = 0.0
    omega_w = 0.0        # driven wheel angular velocity (rad/s)
    alpha_f = alpha_r = 0.0
    F_w_last = 0.0

    def _tick(v, vy, yr_rate, omega_w, steer_on: bool):
        nonlocal alpha_f, alpha_r, F_w_last

        # ── Longitudinal torque ───────────────────────────────────────────────
        if n_drive > 0 and abs(torque_Nm) > 0.1:
            tau_w    = torque_Nm / n_drive
            F_w_att  = tau_w / r           # per-wheel force attempt
            F_w_last = F_w_att

            if abs(F_w_att) <= F_lim:
                # Traction — rolling constraint: ω_w locks to vehicle
                F_total  = F_w_att * n_drive
                omega_w  = v / r
            else:
                # Slip — friction ceiling; excess torque spins wheel faster
                F_total  = math.copysign(F_lim, F_w_att) * n_drive
                tau_excess = abs(tau_w) - F_lim * r
                omega_w   += math.copysign(tau_excess / I_w, tau_w) * _DT

            v += (F_total / _MASS) * _DT
        else:
            F_w_last = 0.0
            omega_w  = v / r    # coasting: passive kinematic
            v        *= (1.0 - 0.50 * _DT)

        drag = 0.05 if abs(torque_Nm) > 0.1 else 0.0
        v   *= (1.0 - drag * _DT)

        # ── Lateral dynamics ──────────────────────────────────────────────────
        if steer_on:
            sr = -math.radians(steer_deg)
            if s_mode == "rear":
                df, dr = 0.0, sr
            elif s_mode == "both":
                df, dr = sr, -sr
            else:
                df, dr = sr, 0.0
        else:
            df = dr = 0.0

        if abs(v) >= 0.5:
            sv   = math.copysign(1.0, v)
            vr   = abs(v)
            af   = sv * df - math.atan2(vy + yr_rate * lf, vr)
            ar   = sv * dr - math.atan2(vy - yr_rate * lr, vr)
        else:
            af = ar = 0.0

        alpha_f, alpha_r = af, ar

        L    = max(0.1, lf + lr)
        Fzf  = _MASS * 9.81 * lr / L
        Fzr  = _MASS * 9.81 * lf / L
        def sat(F, Fz): return max(-_MU * Fz, min(_MU * Fz, F))
        Fyf  = sat(_CF * af, Fzf)
        Fyr  = sat(_CR * ar, Fzr)

        vy       += ((Fyf + Fyr) / _MASS - v * yr_rate) * _DT
        yr_rate  += ((lf * Fyf - lr * Fyr) / _IZ)        * _DT

        if abs(v) > 0.1:
            lim = abs(v) * math.tan(math.radians(45))
            vy  = max(-lim, min(lim, vy))
        if abs(v) < 2.0:
            fade = 1.0 - abs(v) / 2.0
            k    = math.exp(-fade * 10.0 * _DT)
            vy  *= k;  yr_rate *= k

        return v, vy, yr_rate, omega_w

    for t in range(n_settle + n_steer):
        v, vy, yr_rate, omega_w = _tick(v, vy, yr_rate, omega_w, steer_on=(t >= n_settle))

    v_w        = omega_w * r
    eps        = 1e-3
    slip_ratio = (v_w - v) / max(abs(v_w), abs(v), eps) if abs(v_w - v) > eps else 0.0

    return {
        "v":           v,
        "omega_w":     omega_w,
        "driven_rpm":  abs(omega_w) * 60.0 / _TWO_PI,
        "passive_rpm": abs(v / r)   * 60.0 / _TWO_PI,
        "slip_ratio":  slip_ratio,
        "F_w_last":    F_w_last,
        "F_lim":       F_lim,
        "slipping":    abs(F_w_last) > F_lim,
        "skidding":    abs(slip_ratio) > 0.30,
        "alpha_f":     alpha_f,
        "alpha_r":     alpha_r,
        "yaw_rate":    yr_rate,
    }


# ── Vehicle templates ─────────────────────────────────────────────────────────

BASE = dict(differential="open")

VEHICLES = {
    "bike  (1+1)": {**BASE,
        "frame_length_m": 2.5, "frame_width_m": 0.8,
        "tyre_radius_cm": 30,  "tyre_width_cm": 10,
        "groups": {
            "front":  {"axle_count": 1, "position_pct": 15, "wheels_per_axle": 1},
            "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
            "rear":   {"axle_count": 1, "position_pct": 85, "wheels_per_axle": 1},
        },
    },
    "tuktuk(1+2)": {**BASE,
        "frame_length_m": 3.0, "frame_width_m": 1.4,
        "tyre_radius_cm": 28,  "tyre_width_cm": 15,
        "groups": {
            "front":  {"axle_count": 1, "position_pct": 15, "wheels_per_axle": 1},
            "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
            "rear":   {"axle_count": 1, "position_pct": 85, "wheels_per_axle": 2},
        },
    },
    "car   (2+2)": {**BASE,
        "frame_length_m": 4.0, "frame_width_m": 1.8,
        "tyre_radius_cm": 33,  "tyre_width_cm": 22,
        "groups": {
            "front":  {"axle_count": 1, "position_pct": 20, "wheels_per_axle": 2},
            "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
            "rear":   {"axle_count": 1, "position_pct": 80, "wheels_per_axle": 2},
        },
    },
    "truck (2+4)": {**BASE,
        "frame_length_m": 8.0, "frame_width_m": 2.5,
        "tyre_radius_cm": 50,  "tyre_width_cm": 30,
        "groups": {
            "front":  {"axle_count": 1, "position_pct": 12, "wheels_per_axle": 2},
            "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 4},
            "rear":   {"axle_count": 1, "position_pct": 85, "wheels_per_axle": 4},
        },
    },
    "l-truck(2+4+4)": {**BASE,
        "frame_length_m": 12.0, "frame_width_m": 2.5,
        "tyre_radius_cm": 50,   "tyre_width_cm": 30,
        "groups": {
            "front":  {"axle_count": 1, "position_pct":  8, "wheels_per_axle": 2},
            "middle": {"axle_count": 1, "position_pct": 50, "wheels_per_axle": 4},
            "rear":   {"axle_count": 1, "position_pct": 90, "wheels_per_axle": 4},
        },
    },
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _bar(label, value, hi, width=20, fmt=".1f") -> str:
    filled = max(0, min(width, round(abs(value) / hi * width)))
    bar = "█" * filled + "░" * (width - filled)
    return f"[{bar}] {value:{fmt}}"

def _tendency(af, ar) -> str:
    if abs(af) < 1e-4 and abs(ar) < 1e-4:
        return "—  (straight)"
    ratio = abs(ar) / max(abs(af), 1e-6)
    if ratio > 1.25:
        return f"OVERSTEER  (|αr|/|αf|={ratio:.2f})"
    elif ratio < 0.75:
        return f"UNDERSTEER (|αr|/|αf|={ratio:.2f})"
    else:
        return f"neutral    (|αr|/|αf|={ratio:.2f})"

def _slip_label(s: dict) -> str:
    if s["skidding"]:   return "SKID  !!!"
    if s["slipping"]:   return "slip"
    return "traction"


# ── Test sections ─────────────────────────────────────────────────────────────

TORQUES = [
    ("zero",    0),
    ("low",     300),
    ("medium",  1_200),
    ("high",    2_000),
    ("extreme", 5_000),   # beyond slider; valid physics
]

DRIVE_MODES = ["front", "rear", "both"]
STEER_MODES = ["front", "rear", "both"]


def test_torque_levels():
    """Torque sweep — traction limit, slip, RPM for each vehicle + drive mode."""
    print("\n" + "═" * 78)
    print("  TEST 1 — TORQUE LEVELS  (straight line, front-steer)")
    print("  Verifies: traction ceiling, slip onset, speed, RPM")
    print("═" * 78)

    for vname, vbase in VEHICLES.items():
        for d_mode in DRIVE_MODES:
            cfg = {**vbase, "steering_mode": "front", "drive_mode": d_mode}
            res = resolve_frame(cfg)
            n_d = sum(a["wheels"] for a in res["axles"] if a.get("drivable"))
            n_t = max(1, sum(a["wheels"] for a in res["axles"]))
            r   = res["tyre_radius_m"]
            w   = min(1.5, math.sqrt(max(0.05, res["tyre_width_m"]) / 0.20))
            N_w = _MASS * 9.81 / n_t
            F_lim = _MU * N_w * w

            if n_d == 0:
                continue

            print(f"\n  {vname}  drive={d_mode}  n_drive={n_d}  n_total={n_t}")
            print(f"  r_tyre={r*100:.0f}cm  F_lim/wheel={F_lim:.0f}N  "
                  f"(τ_slip_total={(F_lim * r * n_d):.0f} Nm)")
            print(f"  {'Torque':>9}  {'F_w/whl':>9}  {'F_lim':>7}  "
                  f"{'Status':>10}  {'Speed':>8}  {'Pssv RPM':>9}  "
                  f"{'Drvn RPM':>9}  {'Slip':>6}")
            print("  " + "─" * 76)

            for tlabel, torque in TORQUES:
                s = simulate(cfg, torque, 0.0)
                F_w_pw = torque / max(1, n_d) / r if torque else 0.0
                label  = _slip_label(s)
                remark = ""
                if s["skidding"]:
                    remark = " ← wheel free-spinning"
                elif s["slipping"]:
                    remark = " ← partial slip"
                print(f"  {tlabel:>9}  {F_w_pw:>8.0f}N  {F_lim:>6.0f}N"
                      f"  {label:>10}  {s['v']:>+7.2f}m/s"
                      f"  {s['passive_rpm']:>8.1f}  {s['driven_rpm']:>8.1f}"
                      f"  {s['slip_ratio']:>+5.2f}{remark}")


def test_passive_wheel_rpm():
    """Passive wheels must always rotate at v/r regardless of torque or steer."""
    print("\n" + "═" * 78)
    print("  TEST 2 — PASSIVE WHEEL RPM  (car 2+2, front-steer + rear-drive)")
    print("  Rule: passive_rpm == v / (2π·r) × 60  at ALL times")
    print("═" * 78)

    cfg = {**VEHICLES["car   (2+2)"], "steering_mode": "front", "drive_mode": "rear"}
    res = resolve_frame(cfg)
    r   = res["tyre_radius_m"]

    print(f"\n  {'Scenario':22}  {'v (m/s)':>9}  {'v/r rpm':>9}  "
          f"{'passive_rpm':>11}  {'match':>6}")
    print("  " + "─" * 66)

    scenarios = [
        ("coasting (τ=0)",          0,     0.0),
        ("low τ, no steer",         300,   0.0),
        ("high τ, no steer",        2000,  0.0),
        ("extreme τ (slip!)",       5000,  0.0),
        ("medium τ + 40° steer",    1200, 40.0),
        ("extreme τ + 40° steer",   5000, 40.0),
    ]
    all_ok = True
    for label, torque, steer in scenarios:
        s = simulate(cfg, torque, steer)
        expected = abs(s["v"] / r) * 60.0 / _TWO_PI
        match    = abs(s["passive_rpm"] - expected) < 0.1
        icon     = "✓" if match else "✗"
        if not match:
            all_ok = False
        print(f"  {label:22}  {s['v']:>+9.2f}  {expected:>9.1f}  "
              f"{s['passive_rpm']:>11.1f}  {icon:>6}")

    print(f"\n  Passive RPM == kinematic: {'ALL PASS ✓' if all_ok else 'FAILURES ✗'}")


def test_slip_and_skid():
    """Driven wheel omega diverges from vehicle when torque > friction limit."""
    print("\n" + "═" * 78)
    print("  TEST 3 — SLIP & SKID DETECTION  (per vehicle, rear drive)")
    print("  Rule: slipping → driven_rpm > passive_rpm; skidding → slip_ratio > 0.30")
    print("═" * 78)

    for vname, vbase in VEHICLES.items():
        cfg = {**vbase, "steering_mode": "front", "drive_mode": "rear"}
        res = resolve_frame(cfg)
        n_d = sum(a["wheels"] for a in res["axles"] if a.get("drivable"))
        if n_d == 0:
            continue
        r   = res["tyre_radius_m"]
        N_w = _MASS * 9.81 / max(1, sum(a["wheels"] for a in res["axles"]))
        w   = min(1.5, math.sqrt(max(0.05, res["tyre_width_m"]) / 0.20))
        F_lim = _MU * N_w * w
        tau_slip = F_lim * r * n_d

        print(f"\n  {vname}  (τ_slip ≈ {tau_slip:.0f} Nm total  |  F_lim={F_lim:.0f} N/wheel)")
        print(f"  {'Torque':>9}  {'Status':>10}  {'Pssv RPM':>9}  {'Drvn RPM':>9}  "
              f"{'Slip Ratio':>10}  {'Δ RPM':>8}")
        print("  " + "─" * 68)

        for tlabel, torque in TORQUES:
            s = simulate(cfg, torque, 0.0)
            delta = s["driven_rpm"] - s["passive_rpm"]
            label = _slip_label(s)
            # Verify slip logic
            expected_slip = torque / n_d / r > F_lim
            ok = (s["slipping"] == expected_slip)
            flag = "" if ok else "  ← LOGIC ERROR"
            print(f"  {tlabel:>9}  {label:>10}  {s['passive_rpm']:>9.1f}"
                  f"  {s['driven_rpm']:>9.1f}  {s['slip_ratio']:>+10.3f}"
                  f"  {delta:>+7.1f}{flag}")


def test_oversteer_understeer():
    """
    Steady-state cornering: compare |alpha_r| vs |alpha_f|.
    Weight distribution (static) determines tendency without longitudinal weight transfer.
    Drive mode can amplify or reduce tendency when combined with longitudinal dynamics,
    but our simplified model (no friction circle) isolates the geometric effect.
    """
    print("\n" + "═" * 78)
    print("  TEST 4 — OVER / UNDERSTEER TENDENCY  (steady-state cornering)")
    print("  Metric: |αr| / |αf|  >1.25 = oversteer  <0.75 = understeer")
    print("  Physics: static weight distrib, no friction circle, no weight transfer")
    print("═" * 78)

    TORQUE_CORNER = 1_200   # Nm — enough to maintain speed through the corner
    STEER_ANG     = 25.0    # degrees reference angle

    for vname, vbase in VEHICLES.items():
        print(f"\n  {vname}")
        res_base = resolve_frame({**vbase, "steering_mode": "front", "drive_mode": "rear"})
        lf, lr, _, wb = _geometry(res_base)
        L = lf + lr
        Fzf = _MASS * 9.81 * lr / L
        Fzr = _MASS * 9.81 * lf / L
        print(f"  wb={wb:.2f}m  lf={lf:.2f}  lr={lr:.2f}  "
              f"Fz_front={Fzf:.0f}N  Fz_rear={Fzr:.0f}N")
        print(f"  {'Drive':>6}  {'Steer':>6}  {'Speed':>7}  "
              f"{'αf (°)':>8}  {'αr (°)':>8}  {'|yr| r/s':>9}  Tendency")
        print("  " + "─" * 72)

        for s_mode in ["front", "rear", "both"]:
            for d_mode in ["front", "rear", "both"]:
                cfg = {**vbase, "steering_mode": s_mode, "drive_mode": d_mode}
                res = resolve_frame(cfg)
                n_d = sum(a["wheels"] for a in res["axles"] if a.get("drivable"))
                if n_d == 0:
                    continue

                # Test both right and left turns; average for symmetry
                sr = simulate(cfg, TORQUE_CORNER, +STEER_ANG, n_settle=250, n_steer=350)
                sl = simulate(cfg, TORQUE_CORNER, -STEER_ANG, n_settle=250, n_steer=350)

                af = (abs(sr["alpha_f"]) + abs(sl["alpha_f"])) / 2
                ar = (abs(sr["alpha_r"]) + abs(sl["alpha_r"])) / 2
                spd = (abs(sr["v"]) + abs(sl["v"])) / 2
                yr  = (abs(sr["yaw_rate"]) + abs(sl["yaw_rate"])) / 2
                tend = _tendency(af, ar)

                print(f"  {d_mode:>6}  {s_mode:>6}"
                      f"  {spd:>6.2f}m/s"
                      f"  {math.degrees(af):>+7.3f}°"
                      f"  {math.degrees(ar):>+7.3f}°"
                      f"  {yr:>8.3f}"
                      f"  {tend}")


def test_reverse_slip():
    """Slip and skid behave correctly in reverse (negative torque)."""
    print("\n" + "═" * 78)
    print("  TEST 5 — REVERSE SLIP  (car 2+2, rear drive)")
    print("  Rule: negative torque → negative v, slip_ratio negative if over-limit")
    print("═" * 78)

    cfg = {**VEHICLES["car   (2+2)"], "steering_mode": "front", "drive_mode": "rear"}
    print(f"\n  {'Torque':>11}  {'Status':>10}  {'v':>9}  "
          f"{'Pssv RPM':>9}  {'Drvn RPM':>9}  {'Slip':>6}")
    print("  " + "─" * 68)

    for tlabel, torque in TORQUES:
        rev_torque = -torque
        s = simulate(cfg, rev_torque, 0.0)
        label = _slip_label(s)
        print(f"  {tlabel:>11}  {label:>10}"
              f"  {s['v']:>+8.2f}  {s['passive_rpm']:>9.1f}"
              f"  {s['driven_rpm']:>9.1f}  {s['slip_ratio']:>+5.2f}")


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    test_torque_levels()
    test_passive_wheel_rpm()
    test_slip_and_skid()
    test_oversteer_understeer()
    test_reverse_slip()
    print()
