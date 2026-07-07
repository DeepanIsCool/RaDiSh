"""
Trace RPM and slip through a positive→negative torque switch.
Verifies signed RPM and correct slip direction.

Run:  conda run -n py310 python tests/test_rpm_trace.py
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
r_tyre = res["tyre_radius_m"]
TWO_PI = 2 * math.pi

vp = ViewportWidget(THEME)
vp.set_wheel_frame(CAR)
vp.set_steer(0.0)

states: list[dict] = []
vp.state_updated.connect(states.append)

# ── Phase 1: high positive torque (well above friction limit → overspin) ──────
vp.set_torque(1234.0)
for _ in range(150):
    vp._tick()

print("After +torque (150 ticks):  speed=%.1f km/h   excess_omega=+%.1f rad/s" % (
    vp._veh_speed * 3.6, vp._excess_omega))
rear1 = next(a for a in states[-1]["axles"] if a["group"] == "rear")
print("  REAR  rpm_R=%+.1f  rpm_L=%+.1f  slip_R=%+.4f" % (
    rear1["rpm_right"], rear1["rpm_left"], rear1["slip_right"]))

# ── Phase 2: switch to high negative torque (braking / reverse drive) ─────────
vp.set_torque(-1234.0)

print()
print("Switching to -torque. Tracing every 10 ticks...")
print()
hdr = ("%-4s  %-9s  %-13s  %-10s  %-9s  %-12s  %-13s  state" %
       ("tick", "v (km/h)", "excess_omega", "v_surf_R", "v_kin_R",
        "rpm_R signed", "rpm_R emitted"))
print(hdr)
print("-" * len(hdr))

for i in range(1, 251):
    vp._tick()
    if i % 10 == 0:
        vs = vp._v_right + vp._excess_omega * r_tyre
        vk = vp._v_right
        rpm_signed  = vs / r_tyre * 60 / TWO_PI
        rpm_emitted = states[-1]["axles"][1]["rpm_right"]   # what display gets
        slip_emit   = states[-1]["axles"][1]["slip_right"]

        state_str = ""
        if vs > 0.1 and vp._veh_speed > 0.1:
            state_str = "fwd-spin / fwd-vehicle"
        elif vs > 0.1 and vp._veh_speed < -0.1:
            state_str = "fwd-spin / REV-vehicle"
        elif abs(vs) < 0.1:
            state_str = "<<< wheel stopped >>>"
        elif vs < -0.1 and vp._veh_speed > 0.1:
            state_str = "UNDERSPIN: rev-wheel / fwd-vehicle"
        elif vs < -0.1 and vp._veh_speed < -0.1:
            state_str = "rev-spin / rev-vehicle"

        print("%-4d  %+9.1f  %+13.1f  %+10.3f  %+9.3f  %+12.1f  %+13.1f  %s  (slip=%+.3f)" % (
            i, vp._veh_speed*3.6, vp._excess_omega, vs, vk,
            rpm_signed, rpm_emitted, state_str, slip_emit))

print()
print("Bug check:")
print("  If 'rpm_R signed' and 'rpm_R emitted' differ in sign → sign bug in _emit_state")
print("  If slip is POSITIVE when 'UNDERSPIN' state → slip sign bug")
