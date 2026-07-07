"""
Wheel RPM and slip/skid test: alternating positive/negative torques.

Checks:
  1. Passive wheels always track vehicle speed (RPM = v/r, signed)
  2. Driven wheels within friction limit → RPM matches passive, slip = 0
  3. Driven wheels above friction limit → |driven RPM| > |passive RPM|, slip > 0
  4. Torque reversal: RPM sign flips correctly, slip stays positive for overspin
  5. Zero-torque coast: driven RPM snaps back to passive (excess_omega = 0)
  6. Differential in turns: outer wheel faster than inner

Run:  conda run -n py310 python tests/test_wheel_rpm_slip.py
"""
from __future__ import annotations
import math, sys, os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from PyQt6.QtWidgets import QApplication
_app = QApplication.instance() or QApplication(sys.argv)

from gui.tabs.vehicle_design.viewport import ViewportWidget
from gui.tabs.vehicle_design.wheel_frame_section import resolve_frame

THEME = {k: "#111" for k in (
    "viewport_bg", "viewport_grid_minor", "viewport_grid_major", "viewport_origin")}

CAR = {
    "frame_length_m": 4.0, "frame_width_m": 1.8,
    "tyre_radius_cm": 33,  "tyre_width_cm": 22,
    "steering_mode": "front", "drive_mode": "rear", "differential": "open",
    "groups": {
        "front":  {"axle_count": 1, "position_pct": 20, "wheels_per_axle": 2},
        "middle": {"axle_count": 0, "position_pct": 50, "wheels_per_axle": 2},
        "rear":   {"axle_count": 1, "position_pct": 80, "wheels_per_axle": 2},
    },
}

res    = resolve_frame(CAR)
r      = res["tyre_radius_m"]
mass   = res["mass_kg"]
axles  = res["axles"]
n_drv  = sum(a["wheels"] for a in axles if a.get("drivable"))
n_tot  = sum(a["wheels"] for a in axles)
w_fac  = min(1.5, math.sqrt(max(0.05, res["tyre_width_m"]) / 0.20))
F_lim  = 0.85 * (mass * 9.81 / n_tot) * w_fac
TAU_SLIP = F_lim * r * n_drv         # total torque at onset of slip

TAU_LOW  =  TAU_SLIP * 0.60          # within limit (no slip)
TAU_HIGH =  TAU_SLIP * 2.50          # well above limit (overspin)

TWO_PI = 2 * math.pi
PASS = "✓"; FAIL = "✗"

print(f"Car: mass={mass:.0f} kg  F_lim/wheel={F_lim:.0f} N  tau_slip={TAU_SLIP:.0f} Nm")
print(f"TAU_LOW={TAU_LOW:.0f} Nm  TAU_HIGH={TAU_HIGH:.0f} Nm")
print()

fails = 0
total = 0


def check(label: str, cond: bool, detail: str = "") -> None:
    global fails, total
    total += 1
    icon = PASS if cond else FAIL
    if not cond:
        fails += 1
    print(f"  {icon}  {label}" + (f"  [{detail}]" if detail else ""))


def fresh() -> tuple[ViewportWidget, list[dict]]:
    vp = ViewportWidget(THEME)
    vp.set_wheel_frame(CAR)
    vp.set_steer(0.0)
    states: list[dict] = []
    vp.state_updated.connect(states.append)
    return vp, states


def run(vp: ViewportWidget, states: list, torque: float,
        steer: float = 0.0, n: int = 200) -> dict:
    vp.set_torque(torque)
    vp.set_steer(steer)
    for _ in range(n):
        vp._tick()
    return states[-1]


def rpm_of(s: dict, group: str, side: str) -> float:
    a = next(x for x in s["axles"] if x["group"] == group)
    return a[f"rpm_{side}"]


def slip_of(s: dict, group: str, side: str) -> float:
    a = next(x for x in s["axles"] if x["group"] == group)
    return a[f"slip_{side}"]


# ══════════════════════════════════════════════════════════════════════════════
# SECTION 1 — Passive wheels always match vehicle speed (signed)
# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 1: Passive wheel RPM = v/r (signed), slip always 0")
print("─" * 70)

for tau, label in [(TAU_LOW, "low +torque"), (TAU_HIGH, "high +torque"),
                   (-TAU_LOW, "low -torque"), (-TAU_HIGH, "high -torque")]:
    vp, states = fresh()
    s = run(vp, states, tau)
    v = s["speed_ms"]
    expected_rpm = v / r * 60 / TWO_PI        # signed kinematic
    front_rpm = rpm_of(s, "front", "left")    # passive wheel
    front_slip = slip_of(s, "front", "left")
    rpm_err = abs(front_rpm - expected_rpm)

    check(f"{label:18s}  front RPM sign matches vehicle direction",
          (front_rpm * v >= 0) or abs(v) < 0.1,
          f"RPM={front_rpm:+.1f} v={v*3.6:+.1f}km/h")
    check(f"{'':18s}  front RPM ≈ v/r (within 1%)",
          rpm_err < max(abs(expected_rpm) * 0.01, 0.5),
          f"got={front_rpm:+.1f} exp={expected_rpm:+.1f}")
    check(f"{'':18s}  front slip = 0 (passive)",
          abs(front_slip) < 1e-6,
          f"slip={front_slip:.6f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 2 — Driven wheels within friction limit: no slip, RPM matches passive
# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 2: Within friction limit → driven RPM = passive RPM, slip = 0")
print("─" * 70)

for tau, label in [(TAU_LOW, "+low torque"), (-TAU_LOW, "-low torque")]:
    vp, states = fresh()
    s = run(vp, states, tau)
    front_rpm = rpm_of(s, "front", "left")
    rear_rpm  = rpm_of(s,  "rear", "left")
    rear_slip = slip_of(s,  "rear", "left")

    check(f"{label}: driven RPM sign matches passive",
          (rear_rpm * front_rpm >= 0) or abs(front_rpm) < 1,
          f"rear={rear_rpm:+.1f} front={front_rpm:+.1f}")
    check(f"{label}: driven RPM ≈ passive (within 1%)",
          abs(rear_rpm - front_rpm) < max(abs(front_rpm) * 0.01, 0.5),
          f"Δ={abs(rear_rpm-front_rpm):.2f}")
    check(f"{label}: driven slip = 0",
          abs(rear_slip) < 1e-6,
          f"slip={rear_slip:.6f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 3 — Above friction limit: driven faster than passive, slip > 0
# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 3: Above friction limit → |driven| > |passive|, slip > 0")
print("─" * 70)

for tau, label in [(TAU_HIGH, "+high torque"), (-TAU_HIGH, "-high torque")]:
    vp, states = fresh()
    s = run(vp, states, tau)
    front_rpm = rpm_of(s, "front", "left")
    rear_rpm  = rpm_of(s,  "rear", "left")
    rear_slip = slip_of(s,  "rear", "left")

    check(f"{label}: |driven RPM| > |passive RPM|",
          abs(rear_rpm) > abs(front_rpm) + 10,
          f"|rear|={abs(rear_rpm):.0f}  |front|={abs(front_rpm):.0f}")
    check(f"{label}: driven and passive same direction",
          (rear_rpm * front_rpm >= 0) or abs(front_rpm) < 1,
          f"rear={rear_rpm:+.1f} front={front_rpm:+.1f}")
    check(f"{label}: slip > 0 (overspinning, direction-agnostic)",
          rear_slip > 0.05,
          f"slip={rear_slip:+.4f}")
    check(f"{label}: front slip = 0",
          abs(slip_of(s, "front", "left")) < 1e-6,
          f"slip_front={slip_of(s,'front','left'):.6f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 4 — Torque reversal: RPM crosses zero, slip stays positive throughout
# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 4: Torque reversal (+high → -high)")
print("─" * 70)

vp, states = fresh()

# Phase A: high positive torque → build up forward overspin
run(vp, states, TAU_HIGH, n=150)
sA = states[-1]
rpmA = rpm_of(sA, "rear", "left")
slpA = slip_of(sA, "rear", "left")

check("Phase A (+torque): driven RPM positive (forward spin)",
      rpmA > 10,
      f"rpm={rpmA:+.1f}")
check("Phase A (+torque): slip positive (overspinning)",
      slpA > 0.05,
      f"slip={slpA:+.4f}")

# Phase B: switch to high negative torque
run(vp, states, -TAU_HIGH, n=50)
sB = states[-1]
rpmB  = rpm_of(sB, "rear", "left")
slpB  = slip_of(sB, "rear", "left")
vB    = sB["speed_ms"]

check("Phase B (-torque 50t): slip still positive (still overspinning)",
      slpB > 0.01,
      f"slip={slpB:+.4f}  rpm={rpmB:+.1f}  v={vB*3.6:+.1f}km/h")

# Phase C: negative torque long enough to flip wheel spin direction
run(vp, states, -TAU_HIGH, n=200)
sC = states[-1]
rpmC  = rpm_of(sC, "rear", "left")
slpC  = slip_of(sC, "rear", "left")
vC    = sC["speed_ms"]
frpmC = rpm_of(sC, "front", "left")

check("Phase C (-torque 250t): driven RPM negative (reverse spin)",
      rpmC < -10,
      f"rpm={rpmC:+.1f}  v={vC*3.6:+.1f}km/h")
check("Phase C: passive front RPM same sign as driven rear",
      (rpmC * frpmC >= 0) or abs(frpmC) < 1,
      f"rear={rpmC:+.1f}  front={frpmC:+.1f}")
check("Phase C: slip positive (reverse overspin is still overspinning)",
      slpC > 0.05,
      f"slip={slpC:+.4f}")
check("Phase C: |driven| > |passive| in reverse",
      abs(rpmC) > abs(frpmC) + 10,
      f"|rear|={abs(rpmC):.0f}  |front|={abs(frpmC):.0f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 5 — Coast (zero torque): excess_omega resets, slip returns to 0
# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 5: Zero torque after overspin → slip clears immediately")
print("─" * 70)

vp5, states5 = fresh()
run(vp5, states5, TAU_HIGH, n=100)          # build overspin
run(vp5, states5, 0.0, n=1)                 # one coast tick

sCoast = states5[-1]
rpmFront = rpm_of(sCoast, "front", "left")
rpmRear  = rpm_of(sCoast, "rear",  "left")
slpRear  = slip_of(sCoast, "rear", "left")

check("Coast: excess_omega snaps to 0 after one tick",
      abs(vp5._excess_omega) < 1e-9,
      f"excess_omega={vp5._excess_omega:.6f}")
check("Coast: driven RPM ≈ passive RPM (within 1%)",
      abs(rpmRear - rpmFront) < max(abs(rpmFront) * 0.01, 0.5),
      f"rear={rpmRear:+.1f}  front={rpmFront:+.1f}")
check("Coast: slip = 0",
      abs(slpRear) < 1e-6,
      f"slip={slpRear:.6f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 6 — Alternating cycles with asymmetric timing
#
# Why asymmetric: symmetric phases exactly cancel the excess omega (correct
# physics, but ends at slip=0 — nothing to test).  To see BOTH forward and
# reverse overspin, the negative phase must be longer: first N_cancel ticks
# drain the positive excess to 0, the remaining ticks build reverse excess.
#
# Phase timing:
#   A: +TAU_HIGH  × 80t  →  forward  overspin baseline (excess = +E)
#   B: -TAU_HIGH  × 200t →  first 80t cancel E→0, next 120t build −E' (rev)
#   C: +TAU_HIGH  × 200t →  first 120t cancel −E'→0, next 80t build +E  (fwd)
#   D: -TAU_HIGH  × 200t →  same as B → reverse overspin again
# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 6: Alternating cycles (asymmetric) — overspin in both directions")
print("─" * 70)

# Also trace the transition mid-Phase B to show the zero-crossing
vp6, states6 = fresh()

phases = [
    ( TAU_HIGH,  80, "fwd overspin",         lambda rpm, slp: rpm > 10 and slp > 0.05),
    (-TAU_HIGH, 200, "rev overspin",          lambda rpm, slp: rpm < -10 and slp > 0.05),
    ( TAU_HIGH, 200, "fwd overspin (again)",  lambda rpm, slp: rpm > 10 and slp > 0.05),
    (-TAU_HIGH, 200, "rev overspin (again)",  lambda rpm, slp: rpm < -10 and slp > 0.05),
]

for ph_idx, (tau, n_ticks, label, pred) in enumerate(phases):
    direction = "+" if tau > 0 else "−"

    if ph_idx == 1:
        # Trace the B phase mid-way to show zero-crossing of excess_omega
        print(f"  Phase B mid-transition trace (tick=0 is start of -torque phase):")
        print(f"  {'tick':>5}  {'excess_ω':>10}  {'rear RPM':>10}  {'front RPM':>10}  "
              f"{'slip':>8}  state")
        for sub in range(5):
            run(vp6, states6, tau, n=40)
            tick = (sub + 1) * 40
            s_mid = states6[-1]
            e_mid = vp6._excess_omega
            r_mid = rpm_of(s_mid, "rear", "left")
            f_mid = rpm_of(s_mid, "front", "left")
            g_mid = slip_of(s_mid, "rear", "left")
            state_str = ("rev-overspin" if r_mid < -1 and abs(r_mid) > abs(f_mid) + 1
                         else "cancelling" if e_mid > 0
                         else "kinematic" if abs(g_mid) < 0.01
                         else "unknown")
            print(f"  {tick:>5}  {e_mid:>+10.1f}  {r_mid:>+10.1f}  "
                  f"{f_mid:>+10.1f}  {g_mid:>+8.4f}  {state_str}")
        s = states6[-1]
    else:
        run(vp6, states6, tau, n=n_ticks)
        s = states6[-1]

    rear_rpm  = rpm_of(s, "rear", "left")
    front_rpm = rpm_of(s, "front", "left")
    rear_slip = slip_of(s, "rear", "left")
    ok = pred(rear_rpm, rear_slip)

    check(f"Phase {ph_idx+1} ({direction}{abs(tau):.0f}Nm ×{n_ticks}t): {label}",
          ok,
          f"rear={rear_rpm:+.0f}rpm  front={front_rpm:+.0f}rpm  "
          f"slip={rear_slip:+.4f}  excess_ω={vp6._excess_omega:+.1f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 7 — Differential: right turn, outer wheel faster regardless of torque
# ══════════════════════════════════════════════════════════════════════════════
print("─" * 70)
print("SECTION 7: Differential — outer wheel (L in right turn) faster than inner (R)")
print("─" * 70)

for tau, label in [(TAU_LOW, "+low  (no slip)"),
                   (TAU_HIGH, "+high (overspin)"),
                   (-TAU_LOW, "-low  (no slip)"),
                   (-TAU_HIGH, "-high (overspin)")]:
    vp7, states7 = fresh()
    run(vp7, states7, tau, steer=30.0, n=300)
    s7 = states7[-1]
    rear_L = rpm_of(s7, "rear", "left")   # outer in right turn
    rear_R = rpm_of(s7, "rear", "right")  # inner in right turn

    check(f"{label}: |L(outer)| > |R(inner)| in right turn",
          abs(rear_L) > abs(rear_R) + 1,
          f"L={rear_L:+.1f}  R={rear_R:+.1f}  Δ={abs(rear_L)-abs(rear_R):.1f}")
print()

# ══════════════════════════════════════════════════════════════════════════════
print("═" * 70)
print(f"  RESULT: {total - fails}/{total} passed"
      f"{'  — ALL OK' if fails == 0 else f'  — {fails} FAILED'}")
print("═" * 70)
sys.exit(0 if fails == 0 else 1)
