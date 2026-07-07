"""
Lateral skid bar verification tests.

The lateral skid value per axle is:
    lat_skid = Fy_axle / (mu * Fz_axle)   in range [−1, +1]

where:
    Fy_axle  — bicycle-model lateral tyre force on that axle (+ = leftward force on body)
    Fz_axle  — static normal load on that axle
    mu       — peak friction coefficient (0.85)

Positive sign  → lateral force pushes body LEFT  (left-turn centripetal / right cornering force)
Negative sign  → lateral force pushes body RIGHT (right-turn centripetal / left cornering force)

The bar shows |lat_skid|: 0 = no lateral load, 1 = tyre at lateral friction limit.
Middle axles, trailer axles, and vehicles going straight always show 0.

Run:  conda run -n py310 python tests/test_lateral_skid.py
"""
from __future__ import annotations
import math, sys, os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from PyQt6.QtWidgets import QApplication
_app = QApplication.instance() or QApplication(sys.argv)

from gui.tabs.vehicle_design.viewport import ViewportWidget

THEME = {k: "#111" for k in (
    "viewport_bg", "viewport_grid_minor", "viewport_grid_major", "viewport_origin")}

PASS = "✓";  FAIL = "✗"
total = fails = 0


def check(label: str, cond: bool, detail: str = "") -> None:
    global fails, total
    total += 1
    icon = PASS if cond else FAIL
    if not cond:
        fails += 1
    print(f"  {icon}  {label}" + (f"  [{detail}]" if detail else ""))


def fresh(cfg: dict) -> tuple[ViewportWidget, list[dict]]:
    vp = ViewportWidget(THEME)
    vp.set_wheel_frame(cfg)
    states: list[dict] = []
    vp.state_updated.connect(states.append)
    return vp, states


def run(vp: ViewportWidget, states: list, torque: float,
        steer: float = 0.0, n: int = 250) -> dict:
    vp.set_torque(torque); vp.set_steer(steer)
    for _ in range(n): vp._tick()
    return states[-1]


def lat(s: dict, group: str) -> float:
    a = next((x for x in s["axles"] if x["group"] == group), None)
    return a["lateral_skid"] if a else 0.0


# ── Configs ────────────────────────────────────────────────────────────────────

_BASE = dict(differential="open")

CAR = {**_BASE,
    "frame_length_m": 4.0, "frame_width_m": 1.8,
    "tyre_radius_cm": 33,  "tyre_width_cm": 22,
    "groups": {
        "front":  {"axle_count": 1, "position_pct": 20, "wheels_per_axle": 2},
        "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
        "rear":   {"axle_count": 1, "position_pct": 80, "wheels_per_axle": 2},
    },
}

CAR_WITH_MIDDLE = {**_BASE,
    "frame_length_m": 6.0, "frame_width_m": 2.0,
    "tyre_radius_cm": 40,  "tyre_width_cm": 25,
    "groups": {
        "front":  {"axle_count": 1, "position_pct": 12, "wheels_per_axle": 2},
        "middle": {"axle_count": 1, "position_pct": 50, "wheels_per_axle": 4},
        "rear":   {"axle_count": 1, "position_pct": 85, "wheels_per_axle": 4},
    },
}

CAR_TRAILER = {**_BASE,
    "frame_length_m": 6.0, "frame_width_m": 2.4,
    "tyre_radius_cm": 50,  "tyre_width_cm": 30,
    "groups": {
        "front":  {"axle_count": 1, "position_pct": 12, "wheels_per_axle": 2},
        "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
        "rear":   {"axle_count": 1, "position_pct": 85, "wheels_per_axle": 4},
    },
    "fifth_wheel": {
        "enabled": True, "hitch_pct": 90, "trailer_length_m": 8.0,
        "max_angle_deg": 45, "axle_count": 2, "wheels_per_axle": 4,
        "axle_position_pct": 75, "axle_separation_cm": 5,
    },
}


# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 1: Straight line → lat_skid = 0 on all axles")
print("─" * 70)

for label, cfg, mode in [
    ("front-steer, fwd",  {**CAR, "steering_mode": "front", "drive_mode": "rear"}, "front"),
    ("front-steer, rev",  {**CAR, "steering_mode": "front", "drive_mode": "rear"}, "front"),
    ("rear-steer,  fwd",  {**CAR, "steering_mode": "rear",  "drive_mode": "front"}, "rear"),
    ("both-steer,  fwd",  {**CAR, "steering_mode": "both",  "drive_mode": "rear"}, "both"),
]:
    torque = 200.0 if "fwd" in label else -200.0
    vp, states = fresh(cfg)
    s = run(vp, states, torque, steer=0.0)
    for g in ("front", "rear"):
        lsk = lat(s, g)
        check(f"{label}  {g} lat_skid = 0",
              abs(lsk) < 1e-6,
              f"lat_skid={lsk:.6f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 2: Any turn → |lat_skid| > 0 on tractor axles")
print("─" * 70)

for steer, turn in [(30.0, "right"), (-30.0, "left")]:
    for smode in ["front", "rear", "both"]:
        cfg = {**CAR, "steering_mode": smode, "drive_mode": "rear"}
        vp, states = fresh(cfg)
        s = run(vp, states, 200.0, steer=steer)
        lf = lat(s, "front");  lr = lat(s, "rear")
        check(f"steer={smode:<5} {turn}: |front| > 0",
              abs(lf) > 0.01,
              f"lat_front={lf:+.4f}")
        check(f"steer={smode:<5} {turn}: |rear|  > 0",
              abs(lr) > 0.01,
              f"lat_rear={lr:+.4f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 3: Magnitude always ≤ 1 (bounded by friction saturation)")
print("─" * 70)

for steer in [5.0, 20.0, 40.0, -40.0]:
    cfg = {**CAR, "steering_mode": "front", "drive_mode": "rear"}
    vp, states = fresh(cfg)
    s = run(vp, states, 500.0, steer=steer)
    for g in ("front", "rear"):
        lsk = lat(s, g)
        check(f"steer={steer:+.0f}°  {g}: |lat_skid| ≤ 1",
              abs(lsk) <= 1.0 + 1e-9,
              f"lat_skid={lsk:+.4f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 4: Right and left turns give opposite signs")
print("─" * 70)

for smode in ["front", "rear", "both"]:
    cfg = {**CAR, "steering_mode": smode, "drive_mode": "rear"}
    vp_r, states_r = fresh(cfg);  s_r = run(vp_r, states_r, 300.0, steer=+30.0)
    vp_l, states_l = fresh(cfg);  s_l = run(vp_l, states_l, 300.0, steer=-30.0)

    for g in ("front", "rear"):
        lsk_r = lat(s_r, g)
        lsk_l = lat(s_l, g)
        # Signs must be opposite (non-zero), magnitudes must match
        sign_ok = (lsk_r * lsk_l < 0)           # opposite signs
        mag_ok  = abs(abs(lsk_r) - abs(lsk_l)) < 0.02  # same magnitude (symmetric)
        check(f"steer={smode:<5} {g}: R and L turns opposite sign",
              sign_ok,
              f"right={lsk_r:+.4f}  left={lsk_l:+.4f}")
        check(f"steer={smode:<5} {g}: |right| ≈ |left| (symmetric)",
              mag_ok,
              f"|R|={abs(lsk_r):.4f}  |L|={abs(lsk_l):.4f}  Δ={abs(abs(lsk_r)-abs(lsk_l)):.4f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 5: Front vs rear steer — turn direction differs")
print("─" * 70)

# Front steer right (+20°): vehicle turns RIGHT → centripetal force rightward
#   → both Fy_f and Fy_r are negative (rightward force on vehicle body).
# Rear steer right (+20°): the rear steers right, pushing the tail right,
#   so the vehicle actually turns LEFT → centripetal force leftward
#   → both Fy_f and Fy_r are positive (leftward force on vehicle body).
# This confirms the correct bicycle-model axle assignments.

for smode, expected_sign, description in [
    ("front", -1, "right steer → vehicle turns RIGHT → Fy < 0"),
    ("rear",  +1, "right steer → vehicle turns LEFT  → Fy > 0"),
]:
    cfg = {**CAR, "steering_mode": smode, "drive_mode": "rear"}
    vp, states = fresh(cfg)
    s = run(vp, states, 200.0, steer=20.0)
    lf = lat(s, "front");  lr = lat(s, "rear")
    check(f"steer={smode:<5} right: front sign {'+' if expected_sign>0 else '−'}  ({description})",
          math.copysign(1, lf) == expected_sign,
          f"lat_front={lf:+.4f}")
    check(f"steer={smode:<5} right: rear  sign {'+' if expected_sign>0 else '−'}",
          math.copysign(1, lr) == expected_sign,
          f"lat_rear={lr:+.4f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 6: Middle axles always 0 (not in bicycle model)")
print("─" * 70)

vp, states = fresh({**CAR_WITH_MIDDLE, "steering_mode": "front", "drive_mode": "rear"})
s = run(vp, states, 300.0, steer=30.0)
mid_axles = [a for a in s["axles"] if a["group"] == "middle"]
check(f"vehicle with middle axle: middle group exists in state",
      len(mid_axles) > 0,
      f"found {len(mid_axles)}")
for a in mid_axles:
    check(f"middle axle '{a['label']}': lat_skid = 0",
          abs(a["lateral_skid"]) < 1e-9,
          f"lat_skid={a['lateral_skid']}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 7: Trailer axles always 0")
print("─" * 70)

vp, states = fresh({**CAR_TRAILER, "steering_mode": "front", "drive_mode": "rear"})
s = run(vp, states, 200.0, steer=25.0)
trailer_axles = [a for a in s["axles"] if a["group"] == "trailer"]
check(f"trailer axles visible in state (expected 2)",
      len(trailer_axles) == 2,
      f"found {len(trailer_axles)}")
for a in trailer_axles:
    check(f"'{a['label']}': lat_skid = 0",
          abs(a["lateral_skid"]) < 1e-9,
          f"lat_skid={a['lateral_skid']}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 8: Idle state → lat_skid resets to 0")
print("─" * 70)

vp, states = fresh({**CAR, "steering_mode": "front", "drive_mode": "rear"})
# Build up some cornering
run(vp, states, 300.0, steer=30.0, n=200)
s_corner = states[-1]
check("During cornering: |front lat| > 0",
      abs(lat(s_corner, "front")) > 0.01,
      f"lat_front={lat(s_corner,'front'):+.4f}")

# Let vehicle come to rest (zero torque, zero steer, many ticks)
run(vp, states, 0.0, steer=0.0, n=500)
s_idle = states[-1]
check("After idle: front lat_skid = 0",
      abs(lat(s_idle, "front")) < 1e-6,
      f"lat_front={lat(s_idle,'front'):.6f}")
check("After idle: rear lat_skid = 0",
      abs(lat(s_idle, "rear")) < 1e-6,
      f"lat_rear={lat(s_idle,'rear'):.6f}")
check("After idle: _lat_skid_front instance var = 0",
      abs(vp._lat_skid_front) < 1e-6,
      f"_lat_skid_front={vp._lat_skid_front:.6f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 9: Partial saturation — first lateral-dynamics tick")
print("─" * 70)

# For this 285 kg vehicle, the tyre stiffness CF=60000 N/rad (calibrated for
# a 1500 kg car) means the lateral force saturates (|Fy| = mu*Fz = 1188 N)
# above α ≈ 1.13°.  Once lateral dynamics run for even one tick, vy builds
# up fast and saturates the slip angle by the second tick.
#
# To test partial saturation we must inspect the FIRST emitted state where
# v crosses 0.5 m/s (so lateral equations activate) and vy = yr = 0 still.
# At that instant:  alpha_f ≈ steer_rad  (before any vy / yr buildup),
# so  lat_skid ≈ CF * steer_rad / (mu * Fz_f) — linear and strictly < 1
# for steer < 1.13°.  The FINAL state (steady-state) is always ±1.

cfg = {**CAR, "steering_mode": "front", "drive_mode": "rear"}
prev_abs = 0.0
for steer_deg in [0.3, 0.6, 0.9]:
    vp, states = fresh(cfg)
    run(vp, states, 200.0, steer=steer_deg, n=30)   # run past v=0.5 threshold

    # Find the very first state where lat_skid flips from 0
    first_lsk = None
    for s in states:
        lsk = abs(lat(s, "front"))
        if lsk > 1e-6:
            first_lsk = lsk
            break

    check(f"steer={steer_deg:.1f}°: first non-zero |lat_front| found",
          first_lsk is not None,
          "no non-zero state found")
    if first_lsk is not None:
        check(f"steer={steer_deg:.1f}°: first |lat_front| < 1  (not saturated yet, vy≈0)",
              first_lsk < 1.0,
              f"|lat_front|_first={first_lsk:.4f}")
        check(f"steer={steer_deg:.1f}°: first |lat_front| > previous steer angle (monotone)",
              first_lsk > prev_abs - 0.01,
              f"|lat_front|_first={first_lsk:.4f}  prev={prev_abs:.4f}")
        prev_abs = first_lsk

# At large steer (20°+) the friction limit is hit quickly even at moderate speed.
# (Full-saturation across all angles is covered by Section 3 with 500 Nm.)
for steer_deg in [20.0, 40.0]:
    vp, states = fresh(cfg)
    run(vp, states, 200.0, steer=steer_deg, n=250)
    lsk_ss = abs(lat(states[-1], "front"))
    check(f"steer={steer_deg:.0f}°: steady-state |lat_front| = 1.0  (friction saturated)",
          abs(lsk_ss - 1.0) < 1e-6,
          f"|lat_front|={lsk_ss:.6f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 10: Reverse cornering — same steer direction, same centripetal side")
print("─" * 70)

# Forward right steer → vehicle turns right → centripetal force rightward → lat < 0.
# Reverse right steer → vehicle still curves toward the right (path goes right)
#   but the CAR goes backward, so the car's reference yaw flips.
#   The centripetal acceleration is still toward the right-hand centre of curvature,
#   so Fy still points rightward → lat < 0 (same sign as forward).
# Confirmed by the sign_v flip in the bicycle model which preserves path correctness.

cfg = {**CAR, "steering_mode": "front", "drive_mode": "rear"}
vp_fwd, states_fwd = fresh(cfg);  s_fwd = run(vp_fwd, states_fwd, +200.0, steer=+20.0)
vp_rev, states_rev = fresh(cfg);  s_rev = run(vp_rev, states_rev, -200.0, steer=+20.0)

lsk_fwd_f = lat(s_fwd, "front");  lsk_fwd_r = lat(s_fwd, "rear")
lsk_rev_f = lat(s_rev, "front");  lsk_rev_r = lat(s_rev, "rear")

check("Front steer right fwd: lat_front < 0  (rightward centripetal)",
      lsk_fwd_f < 0,
      f"fwd_front={lsk_fwd_f:+.4f}")
check("Front steer right rev: lat_front < 0  (same centripetal direction)",
      lsk_rev_f < 0,
      f"rev_front={lsk_rev_f:+.4f}")
check("Front steer right: both fwd and rev generate significant lateral force",
      abs(lsk_fwd_f) > 0.5 and abs(lsk_rev_f) > 0.5,
      f"|fwd|={abs(lsk_fwd_f):.4f}  |rev|={abs(lsk_rev_f):.4f}")
check("Rear axle: same sign in fwd and rev",
      lsk_fwd_r * lsk_rev_r > 0,
      f"fwd_rear={lsk_fwd_r:+.4f}  rev_rear={lsk_rev_r:+.4f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("═" * 70)
print(f"  RESULT: {total - fails}/{total} passed"
      f"{'  — ALL OK' if fails == 0 else f'  — {fails} FAILED'}")
print("═" * 70)
sys.exit(0 if fails == 0 else 1)
