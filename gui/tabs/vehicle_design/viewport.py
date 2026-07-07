from __future__ import annotations

import math

from PyQt6.QtWidgets import QWidget, QPushButton
from PyQt6.QtCore import Qt, QRect, QTimer, QPointF, pyqtSignal
from PyQt6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QTransform,
    QPainterPath, QPolygonF,
)

from gui.tabs.vehicle_design.wheel_frame_section import resolve_frame
from simulation.engine import EngineModel
from simulation.transmission import TransmissionModel
from simulation.automatic import AutomaticTransmissionModel
from simulation.brakes import BrakeModel
from simulation.mass import engine_mass, transmission_mass, brake_mass, body_mass


# ── Constants ─────────────────────────────────────────────────────────────────

_PX_PER_M  = 40.0
_DUAL_GAP  = 4        # px gap between dual tyres on one side
_RAIL_W    = 4        # px chassis-rail width
_DT        = 0.016    # physics timestep (s) ≈ 60 fps

# Lateral dynamics (bicycle model)
_DYN_MASS = 1_500.0   # kg
_DYN_IZ   = 2_500.0   # kg·m²
_DYN_CF   = 60_000.0  # N/rad  front cornering stiffness
_DYN_CR   = 60_000.0  # N/rad  rear  cornering stiffness
_DYN_MU   = 0.85      # peak friction coefficient
_WHEEL_MASS_KG = 12.0 # single-wheel mass for rotational inertia estimate

# Colours
_C_RAIL           = QColor(50,  56,  72)
_C_AXLE           = QColor(60,  68,  88)
_C_STEER          = QColor(40,  90, 190)
_C_DRIVE          = QColor(180, 120,  30)
_C_BOTH           = QColor(80,  140,  70)
_C_WHEEL          = QColor(30,  30,  30)
_C_STRIP          = QColor(85,  85,  85)
_C_WRIM           = QColor(55,  60,  76)
_C_TRACTOR_RAIL2  = QColor(62,  66,  82)   # tractor frame border (slightly blue-gray)
_C_TRAILER_RAIL2  = QColor(72,  70,  68)   # trailer frame border (warm neutral gray)
_C_HITCH_OUTER    = QColor(70,  85, 125)   # outer hitch ring
_C_HITCH_INNER    = QColor(155, 170, 210)  # inner hitch dot

# Basic component rendering colours
_C_BODY_FILL    = QColor(31,  97, 141)    # dark teal body interior
_C_ENGINE       = QColor(255, 220,   0)   # yellow engine block
_C_TRANS        = QColor(45,  185,  65)   # green transmission block
_C_FUEL         = QColor(105, 205, 235)   # light blue fuel tank
_C_BRAKE        = QColor(220,  30,  30)   # red brake indicator
_C_DRIVESHAFT   = QColor(100, 110, 130)   # drive shaft line
_C_FLOOR        = QColor( 48,  52,  68)   # chassis floor panel


def _px(m: float) -> int:
    return round(m * _PX_PER_M)


def _cornered_poly_path(pts: list, vertex_corners: list) -> "QPainterPath":
    """
    Build a closed QPainterPath for a polygon where each vertex has an
    independent corner style.

    pts            : list of QPointF (polygon vertices, in order)
    vertex_corners : list of (style, params) per vertex, where
        style  = "angular" | "bevelled" | "rounded"
        params = () for angular
               = (depth, angle_rad) for bevelled
                   depth = cut along the more-vertical edge (y-axis); angle from the vertical;
                   cut along the more-horizontal edge = depth / tan(angle)
               = (radius, ecc) for rounded
                   ecc scales the cut on the more-vertical edge (matches addRoundedRect rx/ry)
    """
    n = len(pts)
    path = QPainterPath()
    first = True

    def _emit(pt: "QPointF") -> None:
        nonlocal first
        if first:
            path.moveTo(pt)
            first = False
        else:
            path.lineTo(pt)

    for i in range(n):
        A = pts[(i - 1) % n]
        B = pts[i]
        C = pts[(i + 1) % n]
        style, params = vertex_corners[i]

        ab = math.hypot(B.x() - A.x(), B.y() - A.y())
        bc = math.hypot(C.x() - B.x(), C.y() - B.y())

        if style == "angular" or ab < 1e-6 or bc < 1e-6:
            _emit(B)
        elif style == "rounded":
            r   = float(params[0])
            ecc = float(params[1]) if len(params) > 1 else 1.0
            # Map eccentricity to axis-aligned radii: rx along horizontal edge, ry=rx*ecc vertical.
            # Detect which edge is more horizontal by comparing abs(dx) to abs(dy).
            ab_horiz = abs(B.x() - A.x()) > abs(B.y() - A.y())
            r_in  = r if ab_horiz else r * ecc    # incoming edge
            r_out = r * ecc if ab_horiz else r     # outgoing edge
            d_in  = min(r_in,  ab / 2.0)
            d_out = min(r_out, bc / 2.0)
            t1 = d_in  / ab
            P1 = QPointF(B.x() - t1 * (B.x() - A.x()), B.y() - t1 * (B.y() - A.y()))
            t2 = d_out / bc
            P2 = QPointF(B.x() + t2 * (C.x() - B.x()), B.y() + t2 * (C.y() - B.y()))
            _emit(P1)
            path.quadTo(B, P2)
        else:  # bevelled
            depth = float(params[0])
            angle = float(params[1]) if len(params) > 1 else math.radians(45.0)
            tan_a = max(1e-6, math.tan(angle))
            # Replicate the original rectangle logic axis-adaptively:
            # the more-vertical edge gets `depth`; the more-horizontal edge gets depth/tan(angle).
            ab_horiz = abs(B.x() - A.x()) > abs(B.y() - A.y())
            d_in  = (depth / tan_a) if ab_horiz else depth
            d_out = depth if ab_horiz else (depth / tan_a)
            d_in  = min(d_in,  ab / 2.0)
            d_out = min(d_out, bc / 2.0)
            t1 = d_in  / ab
            P1 = QPointF(B.x() - t1 * (B.x() - A.x()), B.y() - t1 * (B.y() - A.y()))
            t2 = d_out / bc
            P2 = QPointF(B.x() + t2 * (C.x() - B.x()), B.y() + t2 * (C.y() - B.y()))
            _emit(P1)
            path.lineTo(P2)

    path.closeSubpath()
    return path


# ── Terrain (two-octave value noise) ─────────────────────────────────────────

_TCELL = 40   # world pixels per terrain cell  (= 1 m at _PX_PER_M = 40)

# Colour gradient stops: (noise value, (R, G, B))
_TSTOPS: list[tuple[float, tuple[int, int, int]]] = [
    (0.00, ( 38,  82,  36)),   # dark forest
    (0.28, ( 56, 118,  50)),   # mid grass
    (0.52, ( 72, 142,  58)),   # bright grass
    (0.68, (100, 135,  62)),   # yellow-green
    (0.80, (132, 118,  72)),   # dry straw
    (1.00, (110,  80,  44)),   # brown earth
]


def _noise_val(cx: int, cy: int, seed: int) -> float:
    """Deterministic pseudo-random value in [0, 1] for integer cell (cx, cy)."""
    h = ((cx * 1619 + cy * 31337) ^ (seed * 9173)) & 0x7FFFFFFF
    h ^= h >> 13
    h  = (h * 0x4D34D34D) & 0x7FFFFFFF
    return (h & 0xFFFF) / 65535.0


def _terrain_value(wx: float, wy: float) -> float:
    """Two-octave smoothstep noise at world-pixel position (wx, wy)."""
    result = 0.0
    for scale, seed, weight in ((200.0, 0, 0.60), (70.0, 7919, 0.40)):
        nx = wx / scale;  ny = wy / scale
        x0 = int(math.floor(nx));  y0 = int(math.floor(ny))
        fx = nx - x0;  fy = ny - y0
        fx = fx * fx * (3.0 - 2.0 * fx)   # smoothstep
        fy = fy * fy * (3.0 - 2.0 * fy)
        result += (
            _noise_val(x0,   y0,   seed) * (1 - fx) * (1 - fy) +
            _noise_val(x0+1, y0,   seed) * fx       * (1 - fy) +
            _noise_val(x0,   y0+1, seed) * (1 - fx) * fy       +
            _noise_val(x0+1, y0+1, seed) * fx       * fy
        ) * weight
    return min(1.0, result)


def _terrain_color(v: float) -> QColor:
    for i in range(len(_TSTOPS) - 1):
        v0, c0 = _TSTOPS[i]
        v1, c1 = _TSTOPS[i + 1]
        if v <= v1:
            t = (v - v0) / (v1 - v0)
            return QColor(
                round(c0[0] + t * (c1[0] - c0[0])),
                round(c0[1] + t * (c1[1] - c0[1])),
                round(c0[2] + t * (c1[2] - c0[2])),
            )
    return QColor(*_TSTOPS[-1][1])


_terrain_cache: dict[tuple[int, int], QColor] = {}


def _get_terrain_color(cx: int, cy: int) -> QColor:
    key = (cx, cy)
    if key not in _terrain_cache:
        v = _terrain_value((cx + 0.5) * _TCELL, (cy + 0.5) * _TCELL)
        _terrain_cache[key] = _terrain_color(v)
    return _terrain_cache[key]


class ViewportWidget(QWidget):
    """
    800 × 800 top-down physics viewport.

    Inputs (called by DirectControlWidget via tab wiring):
        set_steer(angle_deg)   — reference steer angle; + = right
        set_torque(torque_Nm)  — total torque on all drivable wheels; + = forward

    Physics:
        Longitudinal: torque → per-wheel force → friction limit → acceleration
        Lateral:      v8.7 bicycle model (slip angles, cornering forces)
        Differential: open (inner/outer rotate at different ω) or locked (equal ω)

    Tyre width effect: wider tyre → higher friction ceiling (up to ×1.5 at 44 cm).

    Signals:
        state_updated(dict) — emitted every tick:
            mass_kg, inertia_kgm2, speed_ms, speed_kmh, vy_ms, yaw_rads,
            rpm_left, rpm_right, traction_frac
    """

    state_updated         = pyqtSignal(dict)
    engine_torque_changed = pyqtSignal(float)   # mirrors engine output to direct_control

    _MINOR_STEP  = 40
    _MAJOR_EVERY = 5

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)

        self._bg    = QColor(theme["viewport_bg"])
        self._minor = QColor(theme["viewport_grid_minor"])
        self._major = QColor(theme["viewport_grid_major"])
        self._orig  = QColor(theme["viewport_origin"])

        # Raw config + resolved frame
        self._resolved: dict = {}

        # Derived geometry (updated by set_wheel_frame)
        self._frame_length_m: float = 4.0
        self._frame_width_m:  float = 1.8
        self._tyre_radius_m:  float = 0.33
        self._tyre_width_m:   float = 0.20
        self._differential:   str   = "open"
        self._steering_mode:  str   = "front"
        self._total_wheels:   int   = 4
        self._resolved_axles: list  = []

        # Bicycle-model geometry
        self._lf:            float = 1.5   # dist from centre to steerable axle
        self._lr:            float = 1.5   # dist from centre to non-steerable centroid
        self._wheelbase_m:   float = 3.0
        self._steer_track_m: float = 1.8
        self._y_nonsteer_m:  float = 1.5   # ICC level, + = rearward from vehicle centre

        # Vehicle world state
        self._veh_x:       float = 0.0
        self._veh_y:       float = 0.0
        self._veh_heading: float = 0.0    # rad, CW from north
        self._veh_speed:   float = 0.0    # m/s, + = forward
        self._veh_steer:   float = 0.0    # degrees reference angle, + = right
        self._vy:          float = 0.0    # lateral velocity (+ = left)
        self._yaw_rate:    float = 0.0    # rad/s (+ = CCW)
        self._in_slip:     bool  = False  # True while dynamic model is active

        # Control inputs
        self._applied_torque: float = 0.0   # Nm, total on all drivable wheels
        self._direct_rpm:     float = 0.0   # when non-zero, overrides torque physics

        # Engine mode
        self._control_mode:    str   = "direct"
        self._alpha:           float = 0.0
        self._engine_model:    EngineModel      = EngineModel()
        self._transmission:    TransmissionModel = TransmissionModel()
        self._auto:            AutomaticTransmissionModel = AutomaticTransmissionModel()
        self._trans_type:      str   = "manual"   # "manual" | "automatic"
        self._drive_range:     str   = "N"        # auto selector: P|R|N|D
        self._auto_lockup:     bool  = False
        self._gear:            int   = 0      # current gear index (default Neutral)
        self._clutch_e:        float = 1.0    # engagement: 1=locked, 0=disengaged
        self._brake_model:     BrakeModel = BrakeModel()
        self._brake_input:     float = 0.0    # brake pedal b ∈ [0,1]

        # Dynamic mass / inertia. Total mass is the sum of every component's
        # contribution; the wheel frame also provides the base yaw inertia.
        self._comp_mass: dict[str, float] = {
            "frame": _DYN_MASS, "engine": 0.0, "transmission": 0.0,
            "brakes": 0.0, "body": 0.0,
        }
        self._frame_inertia_kgm2: float = _DYN_IZ
        self._body_cfg:      dict  = {}      # last vehicle-body cfg (needs frame dims)
        self._mass_kg:       float = _DYN_MASS
        self._inertia_kgm2:  float = _DYN_IZ

        # Per-side wheel rotation (for differential rendering)
        self._rot_right: float = 0.0
        self._rot_left:  float = 0.0

        # Suspension simulation state
        self._prev_speed:  float = 0.0
        self._susp_roll:   float = 0.0
        self._susp_pitch:  float = 0.0

        # Component visibility (controlled by checkboxes in ComponentDesignerWidget)
        self._comp_visibility: dict[str, bool] = {
            "Wheel Frame":   True,
            "Vehicle Body":  True,
            "Engine":        False,
            "Transmission":  False,
            "Brakes":        False,
        }

        # Chassis floor / vehicle body config (from VehicleBodyBody)
        self._chassis_floor_cfg: dict = {}

        # Camera
        self._zoom:     float = 1.0
        self._pan_x:    float = 0.0   # screen-pixel offset from viewport centre
        self._pan_y:    float = 0.0
        self._drag_pos: QPointF | None = None   # right-drag pan start

        # Per-side kinematic contact-patch speeds (m/s, updated each tick)
        self._v_right: float = 0.0
        self._v_left:  float = 0.0

        # Excess angular velocity of driven wheels above the kinematic (no-slip) rate.
        # > 0  →  overspinning (drive slip / wheelspin)
        # < 0  →  underspinning (brake lockup, future)
        # = 0  →  pure rolling, or no driven wheels
        self._excess_omega: float = 0.0

        # Fifth wheel / articulating trailer
        self._fw_enabled:        bool  = False
        self._fw_hitch_pct:      float = 1.00
        self._fw_trailer_len_m:  float = 8.0
        self._fw_trailer_wid_m:  float = 1.8
        self._fw_max_angle_rad:  float = math.radians(45)
        self._fw_kingpin_dist_m: float = 5.6
        self._fw_trailer_axles:  list  = []
        self._fw_wheels:         int   = 4
        self._trailer_heading:   float = 0.0
        self._trailer_rot:       float = 0.0

        self._show_terrain: bool = False

        self._terrain_btn = QPushButton("Terrain", self)
        self._terrain_btn.setCheckable(True)
        self._terrain_btn.setChecked(False)
        self._terrain_btn.setFixedSize(72, 22)
        self._terrain_btn.move(8, 8)
        self._terrain_btn.setStyleSheet("""
            QPushButton {
                background: #161630;
                color: #a0a0cc;
                border: 1px solid #252538;
                border-radius: 3px;
                font-size: 10px;
                padding: 0 6px;
            }
            QPushButton:hover {
                background: #1e1e3c;
                color: #d0d0f0;
            }
            QPushButton:checked {
                background: #1a2540;
                color: #eeeef8;
                border: 1px solid #3d7eff;
            }
        """)
        self._terrain_btn.toggled.connect(self._on_terrain_toggled)

        self._renderer_3d = None
        self._is_3d_active = False

        self._mode_3d_btn = QPushButton("3D View", self)
        self._mode_3d_btn.setCheckable(True)
        self._mode_3d_btn.setChecked(False)
        self._mode_3d_btn.setFixedSize(72, 22)
        self._mode_3d_btn.move(88, 8)
        self._mode_3d_btn.setStyleSheet(self._terrain_btn.styleSheet())
        self._mode_3d_btn.toggled.connect(self._on_3d_toggled)

        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    # ── Public API ────────────────────────────────────────────────────────────

    def _on_terrain_toggled(self, checked: bool) -> None:
        self._show_terrain = checked
        self.update()

    def _on_3d_toggled(self, checked: bool) -> None:
        self._is_3d_active = checked
        if checked and self._renderer_3d is None:
            from gui.tabs.vehicle_design.panda3d_renderer import Panda3DRenderer
            self._renderer_3d = Panda3DRenderer(self.width(), self.height())
            self._renderer_3d.set_clear_color(self._bg)
        self.update()

    def set_component_visibility(self, component: str, visible: bool) -> None:
        self._comp_visibility[component] = visible
        self.update()

    def set_chassis_floor(self, cfg: dict) -> None:
        self._chassis_floor_cfg = cfg
        self._body_cfg = cfg
        self._update_mass_inertia()
        self.update()

    def set_wheel_frame(self, cfg: dict) -> None:
        self._resolved = resolve_frame(cfg)
        self._update_geometry()
        self._update_fifth_wheel()
        self._update_mass_inertia()
        self.update()

    def set_steer(self, angle_deg: float) -> None:
        self._veh_steer = angle_deg

    def set_torque(self, torque_Nm: float) -> None:
        self._applied_torque = torque_Nm

    def set_rpm(self, rpm: float) -> None:
        self._direct_rpm = rpm

    def set_engine_cfg(self, cfg: dict) -> None:
        self._engine_model.update_from_cfg(cfg)
        self._comp_mass["engine"] = engine_mass(cfg)
        self._update_mass_inertia()

    def set_control_mode(self, mode: str) -> None:
        self._control_mode = mode
        if mode == "direct":
            self._alpha = 0.0

    def set_accelerator(self, alpha: float) -> None:
        self._alpha = max(0.0, min(1.0, alpha))

    def set_transmission_cfg(self, cfg: dict) -> None:
        self._transmission.update_from_cfg(cfg)
        self._auto.update_from_cfg(cfg)
        self._trans_type = cfg.get("trans_type", "manual")
        self._comp_mass["transmission"] = transmission_mass(cfg)
        self._update_mass_inertia()

    def set_drive_range(self, rng: str) -> None:
        """Automatic selector: 'P' | 'R' | 'N' | 'D'."""
        self._drive_range = rng

    def set_drive_mode(self, name: str) -> None:
        """Automatic shift style: 'ECO' | 'CITY' | 'SPORT' | …"""
        self._auto.set_drive_mode(name)

    def set_brakes_cfg(self, cfg: dict) -> None:
        self._brake_model.update_from_cfg(cfg)
        self._comp_mass["brakes"] = brake_mass(cfg)
        self._update_mass_inertia()

    def set_brake(self, b: float) -> None:
        self._brake_input = max(0.0, min(1.0, b))

    def set_gear(self, gear: int) -> None:
        self._gear = gear
        # Returning to Neutral restarts a stalled engine (re-idles).
        if gear == 0 and self._transmission.is_stalled:
            self._transmission.is_stalled = False
            self._transmission.engine_rpm = self._engine_model.idle_rpm

    def set_clutch(self, e: float) -> None:
        self._clutch_e = max(0.0, min(1.0, e))

    # ── Geometry ──────────────────────────────────────────────────────────────

    def _update_geometry(self) -> None:
        res = self._resolved
        self._frame_length_m  = res.get("frame_length_m", 4.0)
        self._frame_width_m   = res.get("frame_width_m",  1.8)
        self._tyre_radius_m   = res.get("tyre_radius_m",  0.33)
        self._tyre_width_m    = res.get("tyre_width_m",   0.20)
        self._differential    = res.get("differential",   "open")
        self._steering_mode   = res.get("steering_mode",  "front")
        self._resolved_axles  = res.get("axles", [])
        self._total_wheels    = max(1, sum(a["wheels"] for a in self._resolved_axles))
        flen = self._frame_length_m

        # Axle positions in frame-centric metres: (pos − 0.5) × flen
        # negative = forward, positive = rearward
        steer_ys    = [(a["position"] - 0.5) * flen for a in self._resolved_axles if a.get("steerable")]
        nonsteer_ys = [(a["position"] - 0.5) * flen for a in self._resolved_axles if not a.get("steerable")]

        # Fallback: no steerable axle tagged → promote frontmost as steerable
        if not steer_ys and nonsteer_ys:
            steer_ys    = [min(nonsteer_ys)]
            nonsteer_ys = [y for y in nonsteer_ys if y != steer_ys[0]]
        if not steer_ys:
            steer_ys = [-flen * 0.35]

        if not nonsteer_ys:
            # All axles steer (pure both_steer, no passive axles).
            # ICC reference at vehicle centre (y = 0).
            y_front = min(steer_ys);  y_rear = max(steer_ys)
            self._y_nonsteer_m  = 0.0
            self._wheelbase_m   = max(0.5, y_rear - y_front)
            self._lf            = abs(y_front)
            self._lr            = abs(y_rear)
            self._steer_track_m = self._frame_width_m
            return

        if self._steering_mode == "both":
            # both_steer with passive middle axle(s).
            # Bicycle model spans front ↔ rear steer axles (not their mean, which
            # can collapse to near-zero and produce zero normal load on both).
            # y_nonsteer_m keeps the passive centroid for Ackermann rendering.
            y_front = min(steer_ys);  y_rear = max(steer_ys)
            self._lf            = abs(y_front)
            self._lr            = abs(y_rear)
            self._wheelbase_m   = max(0.5, y_rear - y_front)
            self._y_nonsteer_m  = sum(nonsteer_ys) / len(nonsteer_ys)
            self._steer_track_m = self._frame_width_m
            return

        y_s = sum(steer_ys)    / len(steer_ys)
        y_n = sum(nonsteer_ys) / len(nonsteer_ys)

        self._y_nonsteer_m  = y_n
        self._wheelbase_m   = max(0.5, abs(y_n - y_s))
        self._lf            = abs(y_s)
        self._lr            = abs(y_n)
        self._steer_track_m = self._frame_width_m

    def _update_mass_inertia(self) -> None:
        # Frame contributes its own steel-rail mass + base yaw inertia.
        self._comp_mass["frame"]  = self._resolved.get("mass_kg",      _DYN_MASS)
        self._frame_inertia_kgm2  = self._resolved.get("inertia_kgm2", _DYN_IZ)

        # Body mass depends on the frame footprint grown by its overhangs.
        self._comp_mass["body"] = body_mass(
            self._body_cfg,
            self._resolved.get("frame_length_m", 4.0),
            self._resolved.get("frame_width_m",  1.8),
        )

        frame_kg = max(1.0, self._comp_mass["frame"])
        self._mass_kg = max(1.0, sum(self._comp_mass.values()))
        # Scale the frame's yaw inertia by how much heavier the fully-equipped
        # vehicle is than the bare frame (first-order; components lumped at CG).
        self._inertia_kgm2 = self._frame_inertia_kgm2 * (self._mass_kg / frame_kg)

    def _update_fifth_wheel(self) -> None:
        fw = self._resolved.get("fifth_wheel", {})
        self._fw_enabled        = fw.get("enabled", False)
        self._fw_hitch_pct      = fw.get("hitch_pct", 1.00)
        self._fw_trailer_len_m  = fw.get("trailer_length_m", 8.0)
        self._fw_trailer_wid_m  = fw.get("trailer_width_m",  1.8)
        self._fw_max_angle_rad  = math.radians(fw.get("max_angle_deg", 45))
        self._fw_kingpin_dist_m = max(0.5, fw.get("kingpin_dist_m", 5.6))
        self._fw_trailer_axles  = fw.get("trailer_axles", [])
        self._fw_wheels         = fw.get("wheels", 4)
        if not self._fw_enabled:
            self._trailer_heading = self._veh_heading

    # ── Ackermann ─────────────────────────────────────────────────────────────

    @staticmethod
    def _ackermann_pair(ref_deg: float, wb: float, track: float) -> tuple[float, float]:
        if abs(ref_deg) < 1e-6:
            return 0.0, 0.0
        sign    = 1.0 if ref_deg > 0 else -1.0
        ref_rad = math.radians(abs(ref_deg))
        R = max(0.01, wb) / math.tan(ref_rad)
        T = max(0.01, track)
        inner = math.degrees(math.atan2(abs(wb), R - T / 2))
        outer = math.degrees(math.atan2(abs(wb), R + T / 2))
        right = sign * (inner if sign > 0 else outer)
        left  = sign * (outer if sign > 0 else inner)
        return right, left

    # ── Longitudinal torque physics ───────────────────────────────────────────

    def _apply_drive_torque(self) -> None:
        axles   = self._resolved_axles
        n_drive = sum(a["wheels"] for a in axles if a.get("drivable"))
        r       = max(0.01, self._tyre_radius_m)
        tau     = self._applied_torque

        if n_drive > 0 and abs(tau) > 0.1:
            tau_w = tau / n_drive                        # torque per driven wheel
            F_w   = tau_w / r                           # longitudinal force attempt per wheel
            N_w   = self._mass_kg * 9.81 / self._total_wheels
            w_fac = min(1.5, math.sqrt(max(0.05, self._tyre_width_m) / 0.20))
            F_lim = _DYN_MU * N_w * w_fac               # per-wheel friction ceiling

            if abs(F_w) <= F_lim:
                # Rolling contact — kinematic constraint holds, wheel tracks vehicle.
                F_act             = F_w
                self._excess_omega = 0.0
            else:
                # Slip — friction ceiling applies; excess torque spins wheel above kinematic.
                # Wheel rotational inertia (solid-disk approx per wheel):
                F_act        = math.copysign(F_lim, F_w)
                I_w          = 0.5 * _WHEEL_MASS_KG * r * r
                excess_tau   = abs(tau_w) - F_lim * r
                self._excess_omega += math.copysign(excess_tau / I_w, tau_w) * _DT

            self._veh_speed += (F_act * n_drive / self._mass_kg) * _DT
        else:
            # Coasting — wheels roll kinematically with vehicle; no excess spin.
            self._excess_omega = 0.0

        # Rolling resistance / coast-down. When the powertrain is decoupled from
        # the wheels (neutral, clutch in, or stalled) the vehicle coasts on
        # momentum with only rolling + aero drag. Engine braking, when in gear,
        # arrives separately as negative drive torque from the transmission.
        decoupled = (self._gear == 0 or self._clutch_e < 0.5
                     or self._transmission.is_stalled)
        if decoupled:
            drag = 0.05
        else:
            drag = 0.50 if abs(tau) < 0.1 else 0.05
        self._veh_speed *= (1.0 - drag * _DT)

        # Friction braking — opposes motion, never drives it backward.
        F_brake = self._brake_model.decel_force(
            self._brake_input, self._veh_speed, self._mass_kg, self._tyre_radius_m)
        if F_brake > 0.0:
            dv = (F_brake / self._mass_kg) * _DT
            if dv >= abs(self._veh_speed):
                self._veh_speed = 0.0          # came to rest this frame
            else:
                self._veh_speed -= math.copysign(dv, self._veh_speed)

    # ── Physics tick ─────────────────────────────────────────────────────────

    def _tick(self) -> None:
        # Engine + transmission mode: one-frame lag on engine RPM
        if self._control_mode == "engine":
            n_driven  = max(1, sum(
                a["wheels"] for a in self._resolved_axles if a.get("drivable")))
            wheel_rpm = (self._veh_speed / max(0.01, self._tyre_radius_m)
                         * 60.0 / (2.0 * math.pi))
            # Modern Brake Throttle Override (BTO) / Smart Pedal safety logic:
            # If brakes are applied (> 5%), override throttle input to idle (0.0).
            effective_alpha = self._alpha
            if self._brake_input > 0.05:
                effective_alpha = 0.0

            if self._trans_type == "automatic":
                eng_tau = self._engine_model.compute_torque(
                    self._auto.engine_rpm, effective_alpha)
                wheel_tau, _, self._gear, self._auto_lockup = self._auto.update(
                    eng_tau, wheel_rpm, self._drive_range, effective_alpha, _DT,
                    self._engine_model.idle_rpm, self._engine_model.max_rpm)
            else:
                eng_tau = self._engine_model.compute_torque(
                    self._transmission.engine_rpm, effective_alpha)
                wheel_tau, _ = self._transmission.update(
                    eng_tau, wheel_rpm, self._gear, self._clutch_e, _DT,
                    self._engine_model.idle_rpm, self._engine_model.max_rpm,
                    self._mass_kg, self._tyre_radius_m)
            self._applied_torque = wheel_tau * n_driven
            self.engine_torque_changed.emit(self._applied_torque)

        if abs(self._direct_rpm) > 0.1:
            # Direct RPM mode: bypass torque physics; set speed kinematically
            self._veh_speed   = (self._direct_rpm * 2.0 * math.pi
                                 * max(0.01, self._tyre_radius_m) / 60.0)
            self._excess_omega = 0.0
        else:
            self._apply_drive_torque()
        v = self._veh_speed

        idle = (abs(v) < 1e-4 and abs(self._vy) < 0.01
                and abs(self._yaw_rate) < 0.01 and abs(self._applied_torque) < 0.1)
        if idle:
            self._vy = self._yaw_rate = 0.0
            self._v_right = self._v_left = 0.0
            self._emit_state()
            return

        lf = max(0.05, self._lf)
        lr = max(0.05, self._lr)
        L  = lf + lr

        # steer_rad: + = left, − = right (veh_steer convention: + = right)
        steer_rad = -math.radians(self._veh_steer)

        # Distribute steer angle to front/rear axles based on steering mode.
        # "both": front normal, rear counter-steers (same ICC, tighter radius).
        if self._steering_mode == "rear":
            delta_f, delta_r = 0.0, steer_rad
        elif self._steering_mode == "both":
            delta_f, delta_r = steer_rad, -steer_rad
        else:  # front (default)
            delta_f, delta_r = steer_rad, 0.0

        # ── Kinematic solution: geometric yaw rate for a perfect-grip path ──
        tan_net   = math.tan(delta_f) - math.tan(delta_r)
        omega_kin = v * tan_net / L if abs(tan_net) > 1e-9 else 0.0
        d_fixed   = abs(self._y_nonsteer_m) if abs(self._y_nonsteer_m) > 0.05 else lr
        vy_kin    = omega_kin * d_fixed   # no-slip condition at fixed-axle centroid

        # Compare centripetal force needed for kinematic path against tyre grip.
        F_centripetal = self._mass_kg * abs(v * omega_kin)   # = m·v²/R
        F_grip        = _DYN_MU * self._mass_kg * 9.81

        # Enter slip when grip is exceeded; exit only after vy and yaw_rate
        # have naturally decayed — prevents steer=0 from snapping out of a skid.
        if F_centripetal > F_grip:
            self._in_slip = True
        elif self._in_slip:
            if abs(self._vy) < 0.08 and abs(self._yaw_rate) < 0.04:
                self._in_slip = False

        if not self._in_slip:
            # Within grip: vehicle tracks the geometric (kinematic) path exactly.
            self._yaw_rate = omega_kin
            self._vy       = vy_kin
        else:
            # Grip exceeded (or recovering): dynamic slip model active.
            # State is already initialised from kinematic values at slip entry,
            # so there is no discontinuity.
            Fz_f = self._mass_kg * 9.81 * lr / L
            Fz_r = self._mass_kg * 9.81 * lf / L

            def _sat(Fy: float, Fz: float) -> float:
                lim = _DYN_MU * Fz
                return max(-lim, min(lim, Fy))

            # Cornering stiffness scales with mass so the Euler stability
            # criterion CF·dt/(m·v) stays well below 1 for all vehicle sizes.
            _mass_ratio = self._mass_kg / _DYN_MASS
            sign_v  = math.copysign(1.0, v) if abs(v) > 0.01 else 1.0
            v_ref   = max(abs(v), 0.05)
            alpha_f = sign_v * delta_f - math.atan2(self._vy + self._yaw_rate * lf, v_ref)
            alpha_r = sign_v * delta_r - math.atan2(self._vy - self._yaw_rate * lr, v_ref)

            Fy_f = _sat(_DYN_CF * _mass_ratio * alpha_f, Fz_f)
            Fy_r = _sat(_DYN_CR * _mass_ratio * alpha_r, Fz_r)

            self._vy       += ((Fy_f + Fy_r) / self._mass_kg - v * self._yaw_rate) * _DT
            self._yaw_rate += ((lf * Fy_f - lr * Fy_r) / self._inertia_kgm2) * _DT

            if abs(v) > 0.1:
                lim = abs(v) * math.tan(math.radians(45.0))
                self._vy = max(-lim, min(lim, self._vy))

        h = self._veh_heading
        self._veh_x += ( v * math.sin(h) - self._vy * math.cos(h)) * _PX_PER_M * _DT
        self._veh_y += (-v * math.cos(h) - self._vy * math.sin(h)) * _PX_PER_M * _DT
        self._veh_heading = (self._veh_heading - self._yaw_rate * _DT) % (2.0 * math.pi)

        # ── Per-side wheel rotation (differential) ────────────────────────────
        T  = self._steer_track_m
        sa = abs(self._veh_steer)
        if sa > 0.1 and self._wheelbase_m > 0.1:
            R_t  = self._wheelbase_m / math.tan(math.radians(sa))
            sign = math.copysign(1.0, self._veh_steer)
            # right side = inner in a right turn → shorter path → slower
            v_r  = v * (R_t - sign * T / 2) / R_t
            v_l  = v * (R_t + sign * T / 2) / R_t
        else:
            v_r = v_l = v

        self._v_right = v_r
        self._v_left  = v_l

        r = max(0.01, self._tyre_radius_m)
        TWO_PI = 2.0 * math.pi
        if self._differential == "locked":
            avg = (v_r + v_l) / 2.0
            self._rot_right = (self._rot_right + avg * _DT / r) % TWO_PI
            self._rot_left  = (self._rot_left  + avg * _DT / r) % TWO_PI
        else:
            self._rot_right = (self._rot_right + v_r * _DT / r) % TWO_PI
            self._rot_left  = (self._rot_left  + v_l * _DT / r) % TWO_PI

        # ── Kinematic fifth-wheel model ───────────────────────────────────────
        if self._fw_enabled:
            L_t = self._fw_kingpin_dist_m
            dh  = self._veh_heading - self._trailer_heading
            # normalise to (-π, π)
            dh = (dh + math.pi) % (2 * math.pi) - math.pi
            self._trailer_heading += (self._veh_speed * math.sin(dh) / L_t) * _DT
            # clamp articulation angle
            art = self._trailer_heading - self._veh_heading
            art = (art + math.pi) % (2 * math.pi) - math.pi
            art = max(-self._fw_max_angle_rad, min(self._fw_max_angle_rad, art))
            self._trailer_heading = self._veh_heading + art
            # passive wheel rotation (rolls with tractor speed)
            self._trailer_rot = (self._trailer_rot + self._veh_speed * _DT / r) % TWO_PI

        # ── Suspension Dynamics (2-DOF Sprung Mass model) ────────────────────
        # Calculate accelerations (longitudinal and lateral)
        ax = (self._veh_speed - self._prev_speed) / _DT
        self._prev_speed = self._veh_speed
        ay = self._veh_speed * self._yaw_rate

        # Steady-state roll and pitch targets (in radians)
        # Roll: lean outwards (negative ay produces positive roll_target, lean to right in left turn)
        roll_target = -ay * 0.02
        # Pitch: nose-dive under braking (negative ax produces negative pitch_target, dive forward)
        pitch_target = ax * 0.015

        # Integrate with critically damped low-pass filter (natural frequency ~1.5 Hz)
        self._susp_roll  += (roll_target  - self._susp_roll)  * 8.0 * _DT
        self._susp_pitch += (pitch_target - self._susp_pitch) * 8.0 * _DT

        self._emit_state()
        self.update()

    # ── State export ─────────────────────────────────────────────────────────

    def _emit_state(self) -> None:
        self.state_updated.emit({
            "mass_kg":     self._mass_kg,
            "speed_ms":    self._veh_speed,
            "speed_kmh":   self._veh_speed * 3.6,
            "heading_deg": math.degrees(self._veh_heading) % 360.0,
            "trans_type":  self._trans_type,
            "gear":        self._gear,
            "drive_range": self._drive_range,
            "lockup":      self._auto_lockup,
            "engine_rpm":  (self._auto.engine_rpm if self._trans_type == "automatic"
                            else self._transmission.engine_rpm),
        })

    # ── Input events ─────────────────────────────────────────────────────────

    def wheelEvent(self, event) -> None:
        """Scroll wheel zooms centred on the vehicle."""
        delta = event.angleDelta().y()
        if delta == 0:
            return
        if self._is_3d_active and self._renderer_3d is not None:
            factor = 0.9 if delta > 0 else 1.1
            self._renderer_3d.cam_distance = max(2.0, min(50.0, self._renderer_3d.cam_distance * factor))
        else:
            factor = 1.15 if delta > 0 else 1.0 / 1.15
            self._zoom = max(0.1, min(20.0, self._zoom * factor))
        self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.RightButton:
            self._drag_pos = event.position()

    def mouseMoveEvent(self, event) -> None:
        if (event.buttons() & Qt.MouseButton.RightButton) and self._drag_pos is not None:
            d = event.position() - self._drag_pos
            if self._is_3d_active and self._renderer_3d is not None:
                # Orbit camera in 3D mode
                self._renderer_3d.cam_yaw = (self._renderer_3d.cam_yaw - d.x() * 0.5) % 360.0
                self._renderer_3d.cam_pitch = max(-85.0, min(85.0, self._renderer_3d.cam_pitch + d.y() * 0.3))
            else:
                self._pan_x += d.x()
                self._pan_y += d.y()
            self._drag_pos = event.position()
            self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.RightButton:
            self._drag_pos = None

    def mouseDoubleClickEvent(self, _event) -> None:
        if self._is_3d_active and self._renderer_3d is not None:
            self._renderer_3d.cam_distance = 8.5
            self._renderer_3d.cam_yaw = 180.0
            self._renderer_3d.cam_pitch = 12.0
            self._susp_roll = self._susp_pitch = 0.0
            self._prev_speed = 0.0
        else:
            self._veh_x = self._veh_y = self._veh_heading = 0.0
            self._veh_speed = self._vy = self._yaw_rate = 0.0
            self._in_slip = False
            self._rot_right = self._rot_left = 0.0
            self._trailer_heading = 0.0
            self._trailer_rot = 0.0
            self._excess_omega = 0.0
            self._v_right = self._v_left = 0.0
            self._pan_x = self._pan_y = 0.0
            self._zoom = 1.0
            self._susp_roll = self._susp_pitch = 0.0
            self._prev_speed = 0.0
        self.update()

    # ── Paint ─────────────────────────────────────────────────────────────────

    def paintEvent(self, _event) -> None:
        if self._is_3d_active and self._renderer_3d is not None:
            self._paint_3d()
            return

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        W, H = self.width(), self.height()
        p.fillRect(0, 0, W, H, self._bg)

        # Screen position where the vehicle appears (centre + pan offset)
        scx = W / 2.0 + self._pan_x
        scy = H / 2.0 + self._pan_y
        z   = self._zoom

        # World-to-screen transform:
        #   screen = (world - vehicle_pos) * zoom + (scx, scy)
        tr = QTransform()
        tr.translate(scx, scy)
        tr.scale(z, z)
        tr.translate(-self._veh_x, -self._veh_y)
        p.setTransform(tr)

        # Terrain then grid (both in world coords — transform maps to screen)
        if self._show_terrain:
            self._draw_terrain(p, W, H, scx, scy)
        self._draw_grid(p, W, H, scx, scy)

        # World-origin crosshair
        ox_s = -self._veh_x * z + scx
        oy_s = -self._veh_y * z + scy
        if -20 < ox_s < W + 20 and -20 < oy_s < H + 20:
            hw = max(1, round(12.0 / z))
            p.setPen(QPen(self._orig, max(1, round(1.0 / z))))
            p.drawLine(-hw, 0, hw, 0)
            p.drawLine(0, -hw, 0, hw)

        th = self._veh_heading

        # Tractor (drawn first, trailer on top)
        if self._resolved and self._comp_visibility.get("Wheel Frame", True):
            p.save()
            p.translate(self._veh_x, self._veh_y)
            p.rotate(math.degrees(th))
            self._draw_vehicle(p)
            p.restore()

        # Trailer + hitch connector
        if self._resolved and self._fw_enabled:
            flen_px     = max(20, _px(self._frame_length_m))
            hitch_y_loc = (self._fw_hitch_pct - 0.5) * flen_px
            rear_y_loc  = flen_px // 2
            # Rear edge of the tractor chassis floor (where the connector starts).
            wh_px_t   = max(4, _px(self._tyre_radius_m)) * 2
            _ext      = self._floor_extent(self._resolved_axles, flen_px,
                                           -flen_px // 2, wh_px_t)
            floor_rear_loc = _ext[1] if _ext else rear_y_loc
            # Kingpin in world pixels
            hx = self._veh_x - hitch_y_loc * math.sin(th)
            hy = self._veh_y + hitch_y_loc * math.cos(th)
            drawbar_px = max(0, round(hitch_y_loc - rear_y_loc))
            p.save()
            p.translate(hx, hy)
            p.rotate(math.degrees(self._trailer_heading))
            self._draw_trailer(p, drawbar_px)
            p.restore()

            if hitch_y_loc > floor_rear_loc:
                tc_x = self._veh_x - floor_rear_loc * math.sin(th)
                tc_y = self._veh_y + floor_rear_loc * math.cos(th)
                lw = 2.0 / z
                p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
                p.setPen(QPen(_C_AXLE, lw)); p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawLine(int(tc_x), int(tc_y), int(hx), int(hy))
                r = max(2, round(7.0 / z))
                p.setPen(QPen(_C_AXLE, max(1, round(1.0 / z))))
                p.setBrush(QBrush(_C_HITCH_OUTER))
                p.drawEllipse(int(hx) - r, int(hy) - r, 2 * r, 2 * r)
                ri = max(1, round(3.0 / z))
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QBrush(_C_HITCH_INNER))
                p.drawEllipse(int(hx) - ri, int(hy) - ri, 2 * ri, 2 * ri)
                p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

        # Back to screen space for HUD elements
        p.resetTransform()
        self._draw_compass(p, W)
        p.end()

    def _paint_3d(self) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        W, H = self.width(), self.height()
        
        self._renderer_3d.update_buffer_size(W, H)
        img = self._renderer_3d.render_3d(
            x=self._veh_x / _PX_PER_M,
            y=self._veh_y / _PX_PER_M,
            heading_deg=math.degrees(self._veh_heading),
            steer_deg=self._veh_steer,
            roll_left_rad=self._rot_left,
            roll_right_rad=self._rot_right,
            susp_roll_deg=math.degrees(self._susp_roll),
            susp_pitch_deg=math.degrees(self._susp_pitch)
        )
        if img is not None:
            p.drawImage(self.rect(), img)
            
        self._draw_compass(p, W)
        p.end()

    # ── Grid ─────────────────────────────────────────────────────────────────

    def _draw_grid(self, p: QPainter, W: int, H: int,
                   scx: float, scy: float) -> None:
        """Draw grid in world pixel coords (world transform already applied)."""
        z     = self._zoom
        step  = self._MINOR_STEP
        every = self._MAJOR_EVERY
        lw    = 1.0 / z          # stays ~1 px on screen

        # Visible world bounds
        wx0 = self._veh_x - scx / z
        wx1 = self._veh_x + (W - scx) / z
        wy0 = self._veh_y - scy / z
        wy1 = self._veh_y + (H - scy) / z

        y0i = int(wy0) - step;  y1i = int(wy1) + step
        x0i = int(wx0) - step;  x1i = int(wx1) + step
        for n in range(math.floor(wx0 / step) - 1, math.ceil(wx1 / step) + 2):
            wx = n * step
            p.setPen(QPen(self._major if n % every == 0 else self._minor, lw))
            p.drawLine(wx, y0i, wx, y1i)
        for n in range(math.floor(wy0 / step) - 1, math.ceil(wy1 / step) + 2):
            wy = n * step
            p.setPen(QPen(self._major if n % every == 0 else self._minor, lw))
            p.drawLine(x0i, wy, x1i, wy)

    # ── Terrain ──────────────────────────────────────────────────────────────

    def _draw_terrain(self, p: QPainter, W: int, H: int,
                      scx: float, scy: float) -> None:
        z   = self._zoom
        wx0 = self._veh_x - scx / z
        wx1 = self._veh_x + (W - scx) / z
        wy0 = self._veh_y - scy / z
        wy1 = self._veh_y + (H - scy) / z

        cx0 = int(math.floor(wx0 / _TCELL))
        cx1 = int(math.ceil( wx1 / _TCELL))
        cy0 = int(math.floor(wy0 / _TCELL))
        cy1 = int(math.ceil( wy1 / _TCELL))

        if (cx1 - cx0 + 1) * (cy1 - cy0 + 1) > 3600:
            return   # too many cells at low zoom; skip terrain

        p.setPen(Qt.PenStyle.NoPen)
        for cy in range(cy0, cy1 + 1):
            for cx in range(cx0, cx1 + 1):
                p.setBrush(QBrush(_get_terrain_color(cx, cy)))
                p.drawRect(cx * _TCELL, cy * _TCELL, _TCELL, _TCELL)

    # ── Compass ───────────────────────────────────────────────────────────────

    def _draw_compass(self, p: QPainter, W: int) -> None:
        f = QFont(); f.setPixelSize(9); p.setFont(f)
        p.setPen(self._orig)
        p.drawText(QRect(W - 22, 6, 16, 12), Qt.AlignmentFlag.AlignLeft, "N")
        p.drawLine(W - 14, 20, W - 14, 32)
        p.drawLine(W - 14, 20, W - 18, 26)
        p.drawLine(W - 14, 20, W - 10, 26)

    # ── Vehicle renderer ──────────────────────────────────────────────────────

    def _draw_vehicle(self, p: QPainter) -> None:
        res     = self._resolved
        flen_m  = self._frame_length_m
        fwid_m  = self._frame_width_m
        flen_px = max(20, _px(flen_m))
        fwid_px = max(10, _px(fwid_m))
        tr_half = fwid_px // 2
        rl_half = max(6, fwid_px // 6)
        wr_px   = max(4, _px(self._tyre_radius_m))
        ww_px   = max(2, _px(self._tyre_width_m))
        wh_px   = wr_px * 2
        start_y = -flen_px // 2

        sa = abs(self._veh_steer)
        R_icc = (self._wheelbase_m / math.tan(math.radians(sa))
                 * math.copysign(1.0, self._veh_steer)) if sa > 0.1 else None

        # ── Layer 1: chassis frame rails ─────────────────────────────────────────
        # Border of the chassis-floor outline (with overhangs), plus central
        # support rails on rectangular frames.
        rail_w      = max(2, round(fwid_px * 0.04))   # match the trailer rail thickness
        chassis_col = QColor(self._resolved.get("chassis_color", "#3e4258"))
        sw_half     = ww_px // 2 + max(1, rail_w)   # single-wheel axle half-span (Layer 2)

        self._draw_chassis_rails(p, self._resolved_axles, flen_px, start_y,
                                 tr_half, ww_px, wh_px, chassis_col, rail_w)

        _vb_on = self._comp_visibility.get("Vehicle Body", True)

        # ── Layer 2: chassis floor (support rails below it) ───────────────────
        # Central support rails — below the chassis floor (part of the under-frame).
        self._draw_support_rails(p, self._resolved_axles, flen_px, start_y,
                                 wh_px, chassis_col, rail_w, rl_half)

        if _vb_on and self._chassis_floor_cfg.get("view_chassis_floor", False):
            self._draw_chassis_floor(p, self._resolved_axles, flen_px, start_y,
                                     tr_half, ww_px, wh_px)

        # ── Layer 3: axles, brakes, wheels — above the chassis floor ──────────
        show_brakes = self._comp_visibility.get("Brakes", True)
        br_w = max(4, ww_px // 2) if show_brakes else 0
        br_h = max(4, wh_px // 2) if show_brakes else 0
        for axle in self._resolved_axles:
            pos  = axle["position"]
            ay   = start_y + round(pos * flen_px)
            wpa  = axle["wheels"]
            stbl = axle.get("steerable", False)
            drvb = axle.get("drivable",  False)

            ac = (_C_BOTH  if stbl and drvb else
                  _C_STEER if stbl else
                  _C_DRIVE if drvb else _C_AXLE)

            ax_ext  = max(1, ww_px // 4)
            ax_half = sw_half if wpa == 1 else tr_half + ax_ext
            _apen   = QPen(ac, 2)
            _apen.setCapStyle(Qt.PenCapStyle.FlatCap)
            p.setPen(_apen); p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawLine(-ax_half, ay, ax_half, ay)

            if stbl and R_icc is not None:
                ay_m = (pos - 0.5) * flen_m
                d_i  = self._y_nonsteer_m - ay_m
                if self._steering_mode == "rear":
                    d_i = abs(d_i)
                if abs(d_i) > 0.05:
                    ref = (math.degrees(math.atan2(d_i, abs(R_icc)))
                           * math.copysign(1.0, R_icc))
                    r_deg, l_deg = self._ackermann_pair(ref, abs(d_i), fwid_m)
                else:
                    r_deg = l_deg = 0.0
            else:
                r_deg = l_deg = 0.0

            self._draw_axle_wheels(p, 0, ay, wpa, ww_px, wh_px, tr_half,
                                   r_deg, l_deg,
                                   self._rot_right, self._rot_left,
                                   show_brakes, br_w, br_h)

        # ── Layer 4: transmission (driveshaft first) then engine ──────────────
        if self._comp_visibility.get("Transmission", True):
            self._draw_driveshaft(p, flen_px, start_y)
            self._draw_transmission_block(p, flen_px, fwid_px, rl_half, start_y)
        if self._comp_visibility.get("Engine", True):
            self._draw_engine_block(p, flen_px, fwid_px, rl_half, start_y)

        # ── Layer 5: fuel tank ────────────────────────────────────────────────
        if _vb_on and self._chassis_floor_cfg.get("view_fuel_tank", True):
            self._draw_fuel_tank(p, flen_px, fwid_px, rl_half, start_y)

        # ── Layer 6: vehicle body silhouette (topmost) ────────────────────────
        if _vb_on and self._chassis_floor_cfg.get("view_body", False):
            self._draw_body(p, self._resolved_axles, flen_px, start_y,
                            tr_half, ww_px, wh_px)


    # ── Basic component renderers ─────────────────────────────────────────────

    @staticmethod
    def _label(p: QPainter, x: int, y: int, w: int, h: int,
               text: str, color: QColor) -> None:
        """Draw a centred bold label inside a rect if there is enough space."""
        if w < 12 or h < 12:
            return
        f = QFont()
        f.setPixelSize(max(9, min(h // 3, 16)))
        f.setBold(True)
        p.setFont(f)
        p.setPen(color)
        p.drawText(QRect(x, y, w, h), Qt.AlignmentFlag.AlignCenter, text)

    def _draw_engine_block(self, p: QPainter, flen_px: int, fwid_px: int,
                           rl_half: int, start_y: int) -> None:
        """Yellow engine block: 60 % frame width × 30 % frame height, rounded corners."""
        w = max(12, round(0.60 * fwid_px))
        h = max(12, round(0.30 * flen_px))
        x = -w // 2
        y = start_y + 4
        r = min(max(4, w // 6), h // 2)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_C_ENGINE))
        p.drawRoundedRect(x, y, w, h, r, r)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

    def _draw_transmission_block(self, p: QPainter, flen_px: int, fwid_px: int,
                                  rl_half: int, start_y: int) -> None:
        """Green transmission block with internal gear-array symbols."""
        eng_h = max(12, round(0.30 * flen_px))
        y0 = start_y + 4 + eng_h
        h  = max(10, round(0.30 * flen_px))
        w  = max(12, round(0.60 * 2 * rl_half))
        x  = -w // 2
        r  = min(max(3, w // 8), h // 2)

        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_C_TRANS))
        p.drawRoundedRect(x, y0, w, h, r, r)

        # ── Gear array: two staggered rows of small rounded rectangles ──────────
        pad_x  = max(4, w // 8)
        pad_y  = max(3, h // 5)
        inner_w = w - 2 * pad_x
        inner_h = h - 2 * pad_y

        gear_w = max(3, round(inner_w * 0.14))
        gear_h = max(3, round(inner_h * 0.42))
        gr     = max(1, gear_w // 3)          # gear rect corner radius
        n_gears = max(2, inner_w // (gear_w + max(2, gear_w // 2)))

        # two rows — offset second row by half a step for interleaved look
        step = inner_w / n_gears
        row_y = [y0 + pad_y, y0 + pad_y + inner_h - gear_h]
        gear_color = _C_TRANS.darker(160)
        p.setBrush(QBrush(gear_color))

        for row_idx, gy in enumerate(row_y):
            offset_x = (step / 2) if row_idx == 1 else 0.0
            count = n_gears if row_idx == 0 else max(1, n_gears - 1)
            for i in range(count):
                gx = x + pad_x + offset_x + i * step
                p.drawRoundedRect(
                    int(round(gx)), int(round(gy)),
                    gear_w, gear_h,
                    float(gr), float(gr),
                )

        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

    def _draw_driveshaft(self, p: QPainter, flen_px: int, start_y: int) -> None:
        """Line from the transmission output centre to each drivable axle centre."""
        drive_axles = [a for a in self._resolved_axles if a.get("drivable")]
        if not drive_axles:
            return
        eng_h   = max(12, round(0.30 * flen_px))
        trans_h = max(10, round(0.30 * flen_px))
        shaft_y = start_y + 4 + eng_h + trans_h   # bottom centre of transmission
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setPen(QPen(_C_DRIVESHAFT, 2))
        for axle in drive_axles:
            ay = start_y + round(axle["position"] * flen_px)
            p.drawLine(0, shaft_y, 0, ay)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

    def _draw_fuel_tank(self, p: QPainter, flen_px: int, fwid_px: int,
                        rl_half: int, start_y: int) -> None:
        """Light-blue fuel tank: 60 % frame width × 30 % frame height, rear-anchored."""
        h = max(12, round(0.30 * flen_px))
        w = max(12, round(0.60 * fwid_px))
        x = -w // 2
        y = start_y + flen_px - h - 3
        r = min(max(4, w // 6), h // 2)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_C_FUEL))
        p.drawRoundedRect(x, y, w, h, r, r)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

    @staticmethod
    def _is_rectangular(axles: list) -> bool:
        """A rectangular (car/truck) frame — every axle has ≥2 wheels and there
        are at least two axles. Excludes bikes and tuktuks (single-wheel axles)."""
        return len(axles) >= 2 and all(a.get("wheels", 2) >= 2 for a in axles)

    def _overhangs_px(self, for_trailer: bool, _no_overhang: bool) -> tuple:
        """(front, side, rear) overhangs in px. Tractor and trailer keep separate
        config entries (trailer_* keys); _no_overhang zeroes them all."""
        if _no_overhang:
            return (0, 0, 0)
        cfg = self._chassis_floor_cfg
        pre = "trailer_" if for_trailer else ""
        return (
            max(0, round(_px(cfg.get(pre + "front_overhang", 0.0)))),
            max(0, round(_px(cfg.get(pre + "side_overhang",  0.0)))),
            max(0, round(_px(cfg.get(pre + "rear_overhang",  0.0)))),
        )

    def _floor_extent(self, axles: list, flen_px: int, start_y: int,
                      wh_px: int, _no_overhang: bool = False,
                      for_trailer: bool = False) -> tuple | None:
        """Local-y span of the chassis floor: front edge of the first axle's
        wheels to the rear edge of the last axle's wheels, plus overhangs."""
        front_oh, _side, rear_oh = self._overhangs_px(for_trailer, _no_overhang)
        wheel_half = max(0, wh_px // 2)
        axs = sorted(axles, key=lambda a: a["position"])
        if not axs:
            return None
        y0 = start_y + round(axs[0]["position"]  * flen_px) - wheel_half - front_oh
        y1 = start_y + round(axs[-1]["position"] * flen_px) + wheel_half + rear_oh
        return (y0, y1)

    def _build_floor_outline_path(self, axles: list, flen_px: int, start_y: int,
                                   tr_half: int, ww_px: int, wh_px: int,
                                   _no_overhang: bool = False,
                                   _expand_px: int = 0,
                                   y_force: tuple | None = None,
                                   for_trailer: bool = False) -> QPainterPath:
        """
        Build the chassis floor / body outline QPainterPath from the current config.
        Does NOT subtract wheel-arch notches — callers apply notches themselves.
        Returns an empty path when no axles are defined.
        When _no_overhang is True all overhangs and corner styles are ignored.
        Front and rear end-cap corners support independent styles via
        front_corner / rear_corner config keys (angular, bevelled, rounded, pointed).
        _expand_px: uniform outward expansion used by _draw_body so the body
        silhouette fully covers the chassis floor on all edges.
        """
        cfg      = self._chassis_floor_cfg
        front_oh, side_oh, rear_oh = self._overhangs_px(for_trailer, _no_overhang)
        if _expand_px:
            front_oh += _expand_px
            side_oh  += _expand_px
            rear_oh  += _expand_px

        def _corner_style(end: str) -> tuple:
            """Return (style, params) for the front/rear end cap. Tractor and
            trailer keep separate corner config (trailer_* keys)."""
            key = ("trailer_" if for_trailer else "") + end
            style = "angular" if _no_overhang else (
                cfg.get(f"{key}_corner", "angular") or "angular").lower()
            if style == "bevelled":
                depth = max(1, float(_px(cfg.get(f"{key}_bevel_depth", 0.10))))
                angle = math.radians(
                    max(5.0, min(85.0, float(cfg.get(f"{key}_bevel_angle", 45.0)))))
                return ("bevelled", (depth, angle))
            elif style == "rounded":
                radius = max(1, float(_px(cfg.get(f"{key}_round_radius", 0.20))))
                ecc    = max(0.1, float(cfg.get(f"{key}_round_eccentricity", 1.0)))
                return ("rounded", (radius, ecc))
            return ("angular", ())

        f_corner = _corner_style("front")
        r_corner = _corner_style("rear")

        full_hw   = tr_half + side_oh
        _ww       = max(2, ww_px)
        # side_oh is added so side-overhang applies to single-wheel axles too
        single_hw = _ww // 2 + max(4, _ww // 2) + side_oh
        wheel_half = max(0, wh_px // 2)

        axles_sorted = sorted(axles, key=lambda a: a["position"])
        if not axles_sorted:
            return QPainterPath()

        def _hw(axle) -> int:
            return single_hw if axle.get("wheels", 2) == 1 else full_hw

        kf = [(start_y + round(a["position"] * flen_px), _hw(a)) for a in axles_sorted]

        if y_force is not None:
            # Explicit end caps (used by the trailer: a full-length flat bed).
            y_front = y_force[0] - front_oh
            y_rear  = y_force[1] + rear_oh
            end_hw_f = end_hw_r = full_hw
        else:
            # Default: front edge of first wheels → rear edge of last wheels.
            y_front = kf[0][0]  - front_oh - wheel_half
            y_rear  = kf[-1][0] + rear_oh  + wheel_half
            end_hw_f = kf[0][1]
            end_hw_r = kf[-1][1]

        all_kf: list[tuple[int, int]] = []
        if y_front < kf[0][0]:
            all_kf.append((y_front, end_hw_f))
        all_kf.extend(kf)
        if y_rear > kf[-1][0]:
            all_kf.append((y_rear, end_hw_r))

        # Build polygon vertices: right side top→bottom, left side bottom→top
        right = [QPointF(float(hw), float(y)) for y, hw in all_kf]
        left  = [QPointF(float(-hw), float(y)) for y, hw in reversed(all_kf)]
        poly  = right + left

        # Tag each vertex: front-end, rear-end, or transition (angular)
        def _vcorner(pt: QPointF) -> tuple:
            y = pt.y()
            if abs(y - y_front) < 0.5:
                return f_corner
            if abs(y - y_rear) < 0.5:
                return r_corner
            return ("angular", ())

        vertex_corners = [_vcorner(pt) for pt in poly]

        # If all corners are angular just use a simple polygon — no extra work
        if all(vc[0] == "angular" for vc in vertex_corners):
            path = QPainterPath()
            path.addPolygon(QPolygonF(poly))
            path.closeSubpath()
            return path

        return _cornered_poly_path(poly, vertex_corners)

    def _build_notched_floor_path(self, axles: list, flen_px: int, start_y: int,
                                   tr_half: int, ww_px: int, wh_px: int,
                                   _no_overhang: bool = False,
                                   _skip_single_notch: bool = False,
                                   y_force: tuple | None = None,
                                   for_trailer: bool = False) -> QPainterPath:
        """Floor outline with wheel-arch notches already subtracted.
        Used by both the chassis-rail stroke (Layer 1) and the chassis-floor fill."""
        path = self._build_floor_outline_path(axles, flen_px, start_y, tr_half,
                                              ww_px, wh_px, _no_overhang,
                                              y_force=y_force,
                                              for_trailer=for_trailer)
        if path.isEmpty():
            return path

        _front, side_oh, _rear = self._overhangs_px(for_trailer, _no_overhang)
        full_hw = tr_half + side_oh
        big     = float(full_hw * 6 + 400)

        axles_sorted = sorted(axles, key=lambda a: a["position"])
        diag_px      = math.sqrt(ww_px ** 2 + wh_px ** 2)
        notch_core   = round(diag_px)
        notch_pad    = max(4, round(diag_px * 0.18))
        notch_total  = notch_core + 2 * notch_pad
        r_notch      = float(notch_pad)

        for axle in axles_sorted:
            ay  = start_y + round(axle["position"] * flen_px)
            wpa = axle.get("wheels", 2)
            n   = wpa // 2

            inner_tyre_cx   = (tr_half - (n - 1) * (ww_px + _DUAL_GAP)) if n > 0 else 0
            tyre_inner_edge = float(inner_tyre_cx) - ww_px / 2
            h_gap           = max(6, round(ww_px * 0.75))
            notch_inner     = max(0.0, tyre_inner_edge - h_gap)

            yl = float(ay - notch_total // 2)
            yh = float(notch_total)

            if wpa == 1:
                if not _skip_single_notch:
                    # Pill cutout: half-pill visible when axle is at a frame end
                    slot_hw = float(ww_px // 2 + notch_pad)
                    slot_hh = float(wh_px // 2 + notch_pad)
                    slot = QPainterPath()
                    slot.addRoundedRect(-slot_hw, float(ay) - slot_hh,
                                        2.0 * slot_hw, 2.0 * slot_hh,
                                        slot_hw, slot_hw)
                    path = path.subtracted(slot)
                # else: rail runs straight through — no cutout
            else:
                cr = QPainterPath()
                cr.addRoundedRect(float(notch_inner), yl,
                                  big - float(notch_inner), yh, r_notch, r_notch)
                path = path.subtracted(cr)
                cl = QPainterPath()
                cl.addRoundedRect(-big, yl,
                                  big - float(notch_inner), yh, r_notch, r_notch)
                path = path.subtracted(cl)

        return path

    def _draw_chassis_rails(self, p: QPainter, axles: list, flen_px: int,
                            start_y: int, tr_half: int, ww_px: int, wh_px: int,
                            chassis_col: QColor, rail_w: int,
                            y_force: tuple | None = None,
                            for_trailer: bool = False) -> None:
        """Chassis rails: the stroked border of the chassis-floor outline (with
        overhangs). The central support rails are drawn separately, below the
        floor panel, by _draw_support_rails."""
        outline = self._build_notched_floor_path(
            axles, flen_px, start_y, tr_half, ww_px, wh_px,
            _no_overhang=False, _skip_single_notch=True, y_force=y_force,
            for_trailer=for_trailer)
        if outline.isEmpty():
            return
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rpen = QPen(chassis_col, float(rail_w))
        rpen.setJoinStyle(Qt.PenJoinStyle.MiterJoin)
        rpen.setCapStyle(Qt.PenCapStyle.FlatCap)
        p.setPen(rpen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPath(outline)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(Qt.BrushStyle.NoBrush)

    def _draw_support_rails(self, p: QPainter, axles: list, flen_px: int,
                            start_y: int, wh_px: int, chassis_col: QColor,
                            rail_w: int, rl_half: int,
                            y_force: tuple | None = None,
                            force_rect: bool = False,
                            for_trailer: bool = False) -> None:
        """Two parallel central support rails, drawn below the chassis floor as
        part of the under-frame. The rails straddle the centreline with a gap
        roughly matching the transmission-block width. Only rectangular frames
        get them, unless force_rect is set (the trailer bed is always rectangular)."""
        if not force_rect and not self._is_rectangular(axles):
            return
        ext = y_force if y_force is not None else \
            self._floor_extent(axles, flen_px, start_y, wh_px,
                               for_trailer=for_trailer)
        if ext is None:
            return
        y0, y1 = int(ext[0]), int(ext[1])
        rw  = max(2, rail_w)                          # ensure the rails are visible
        gap = max(3, round(0.60 * 2 * rl_half))       # ≈ transmission-block width
        col = chassis_col.lighter(125)                # stand out against the floor panel
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(col))
        p.drawRect(-(gap // 2) - rw, y0, rw, y1 - y0)   # left rail
        p.drawRect(  gap // 2,       y0, rw, y1 - y0)   # right rail
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        p.setBrush(Qt.BrushStyle.NoBrush)

    def _draw_chassis_floor(self, p: QPainter, axles: list, flen_px: int,
                            start_y: int, tr_half: int, ww_px: int, wh_px: int,
                            y_force: tuple | None = None,
                            for_trailer: bool = False) -> None:
        """Floor panel: shared outline minus open wheel-arch notches."""
        path = self._build_notched_floor_path(axles, flen_px, start_y, tr_half,
                                              ww_px, wh_px, y_force=y_force,
                                              for_trailer=for_trailer)
        if path.isEmpty():
            return
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_C_FLOOR))
        p.drawPath(path)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

    def _draw_body(self, p: QPainter, axles: list, flen_px: int, start_y: int,
                   tr_half: int, ww_px: int, wh_px: int,
                   y_force: tuple | None = None,
                   for_trailer: bool = False) -> None:
        """Body silhouette: same outline as the chassis floor, expanded a hair so
        the floor edge never peeks out, and solid (no wheel cutouts)."""
        path = self._build_floor_outline_path(axles, flen_px, start_y, tr_half,
                                              ww_px, wh_px, _expand_px=2,
                                              y_force=y_force,
                                              for_trailer=for_trailer)
        if path.isEmpty():
            return
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_C_BODY_FILL))
        p.drawPath(path)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

    def _draw_brakes(self, p: QPainter, ay: int,
                     tr_half: int, rl_half: int, wh_px: int, ww_px: int) -> None:
        """Red rectangles attached to the inboard edge of each tyre (½ wheel dims)."""
        br_w = max(4, ww_px // 2)
        br_h = max(4, wh_px // 2)
        hws  = ww_px // 2          # wheel half-width along the axle
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(_C_BRAKE))
        # right brake — flush against the right tyre's inboard face
        p.drawRect(tr_half - hws - br_w,   ay - br_h // 2, br_w, br_h)
        # left brake — mirror, flush against the left tyre's inboard face
        p.drawRect(-tr_half + hws,         ay - br_h // 2, br_w, br_h)

    def _draw_trailer(self, p: QPainter, drawbar_px: int = 0) -> None:
        """
        Draw the articulating trailer.  Painter origin is at the kingpin (hitch).
        drawbar_px > 0: rigid V-frame from kingpin (y=0) to the trailer body
        front corners (y=drawbar_px), then the chassis from y=drawbar_px onward.
        """
        flen_m  = self._fw_trailer_len_m
        fwid_m  = self._fw_trailer_wid_m
        flen_px = max(20, _px(flen_m))
        fwid_px = max(10, _px(fwid_m))
        tr_half = fwid_px // 2
        rl_half = max(6, fwid_px // 6)
        wr_px   = max(4, _px(self._tyre_radius_m))
        ww_px   = max(2, _px(self._tyre_width_m))
        wh_px   = wr_px * 2
        body_y  = drawbar_px          # where the flat bed starts in local coords

        # Trailer is a full-length flat bed from body_y to body_y + flen_px.
        y_force = (body_y, body_y + flen_px)
        # Trailer axles carry no "wheels" key — inject the configured count so the
        # shared floor/rail/body builders treat them like the tractor's axles.
        axles = [{"position": a["position"], "wheels": self._fw_wheels}
                 for a in self._fw_trailer_axles]

        chassis_col = QColor(self._resolved.get("chassis_color", "#3e4258"))
        rail_w      = max(2, round(fwid_px * 0.04))
        _vb_on      = self._comp_visibility.get("Vehicle Body", True)

        # ── V drawbar (rigid frame from kingpin to trailer bed corners) ───────
        if drawbar_px > 0:
            p.setPen(QPen(chassis_col, max(2, rail_w))); p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawLine(-rl_half, body_y,  0, 0)   # left leg
            p.drawLine( rl_half, body_y,  0, 0)   # right leg

        # ── Layer 1: chassis rails (matches the tractor) ──────────────────────
        self._draw_chassis_rails(p, axles, flen_px, body_y, tr_half,
                                 ww_px, wh_px, chassis_col, rail_w,
                                 y_force=y_force, for_trailer=True)

        # ── Layer 2: central support rails (below the floor) + chassis floor ──
        # The trailer bed is always rectangular, so force the beams on.
        self._draw_support_rails(p, axles, flen_px, body_y, wh_px,
                                 chassis_col, rail_w, rl_half, y_force=y_force,
                                 force_rect=(self._fw_wheels >= 2), for_trailer=True)

        if _vb_on and self._chassis_floor_cfg.get("view_chassis_floor", False):
            self._draw_chassis_floor(p, axles, flen_px, body_y, tr_half,
                                     ww_px, wh_px, y_force=y_force, for_trailer=True)

        # ── Layer 3: axles + wheels (passive — no steer, no drive) ────────────
        for axle in axles:
            ay = body_y + round(axle["position"] * flen_px)
            ax_half = rl_half if self._fw_wheels == 1 else tr_half
            p.setPen(QPen(_C_AXLE, 3)); p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawLine(-ax_half, ay, ax_half, ay)
            if self._fw_wheels >= 2:
                p.setPen(QPen(_C_AXLE, 2))
                p.drawLine(-tr_half - rail_w, ay, -tr_half, ay)
                p.drawLine( tr_half + rail_w, ay,  tr_half, ay)
            self._draw_axle_wheels(p, 0, ay, self._fw_wheels,
                                   ww_px, wh_px, tr_half,
                                   0.0, 0.0,
                                   self._trailer_rot, self._trailer_rot)

        # ── Layer 4: body silhouette (topmost) ────────────────────────────────
        if _vb_on and self._chassis_floor_cfg.get("view_body", False):
            self._draw_body(p, axles, flen_px, body_y, tr_half,
                            ww_px, wh_px, y_force=y_force, for_trailer=True)

    def _draw_axle_wheels(self, p: QPainter, cx: int, ay: int,
                          wpa: int, ww: int, wh: int, tr_half: int,
                          steer_r: float, steer_l: float,
                          rot_r: float, rot_l: float,
                          draw_brakes: bool = False,
                          br_w: int = 0, br_h: int = 0) -> None:
        # Brakes are rounded on the outer (away-from-wheel) corners for single-tyre
        # axles, but plain rectangles for quad (dual-tyre) axles where the caliper
        # sits enclosed in the gap between the two tyres.
        br_round = (wpa != 4)
        if wpa == 1:
            # Single centre wheel: brakes on both sides so it rotates with steer
            avg_rot   = (rot_r + rot_l) / 2.0
            avg_steer = (steer_r + steer_l) / 2.0
            self._draw_wheel(p, cx, ay, ww, wh, avg_steer, avg_rot,
                             draw_brakes, draw_brakes, br_w, br_h, br_round)
            return
        n = wpa // 2
        for i in range(n):
            wx_r = cx + tr_half - i * (ww + _DUAL_GAP)
            wx_l = cx - tr_half + i * (ww + _DUAL_GAP)
            if draw_brakes and br_w > 0:
                if wpa == 2:
                    # One tyre per side: brake on the inboard (centre-facing) face
                    bl_r, brr_r = True,  False   # right wheel: left face = inboard
                    bl_l, brr_l = False, True    # left wheel: right face = inboard
                elif wpa == 4 and i == 1:
                    # Dual tyres: brake on inner tyre's outboard face (in the gap)
                    bl_r, brr_r = False, True    # inner right: right face → gap
                    bl_l, brr_l = True,  False   # inner left:  left face  → gap
                else:
                    bl_r = brr_r = bl_l = brr_l = False
            else:
                bl_r = brr_r = bl_l = brr_l = False
            self._draw_wheel(p, wx_r, ay, ww, wh, steer_r, rot_r, bl_r, brr_r, br_w, br_h, br_round)
            self._draw_wheel(p, wx_l, ay, ww, wh, steer_l, rot_l, bl_l, brr_l, br_w, br_h, br_round)

    def _draw_wheel(self, p: QPainter, wx: int, ay: int,
                    ww: int, wh: int, steer_deg: float, rot_rad: float,
                    brake_left: bool = False, brake_right: bool = False,
                    br_w: int = 0, br_h: int = 0,
                    brake_rounded: bool = True) -> None:
        """Top-down cylinder projection with N tread sectors."""
        N       = 16
        TWO_PI  = 2.0 * math.pi
        HALF_PI = math.pi / 2.0
        hw  = wh // 2
        hws = ww // 2

        p.save()
        p.translate(wx, ay)
        if abs(steer_deg) > 0.01:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            p.rotate(steer_deg)

        # Brakes drawn in wheel-local rotated space so they steer with the wheel
        if br_w > 0 and br_h > 0 and (brake_left or brake_right):
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(_C_BRAKE))
            bhy = br_h // 2
            if brake_rounded:
                p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
                r = max(2, min(br_w, br_h) // 3)
                if brake_left:
                    bx0, by0 = -hws - br_w, -bhy
                    path = QPainterPath()
                    path.moveTo(bx0 + r, by0)
                    path.lineTo(bx0 + br_w, by0)
                    path.lineTo(bx0 + br_w, by0 + br_h)
                    path.lineTo(bx0 + r, by0 + br_h)
                    path.arcTo(bx0, by0 + br_h - 2*r, 2*r, 2*r, -90, -90)
                    path.lineTo(bx0, by0 + r)
                    path.arcTo(bx0, by0, 2*r, 2*r, 180, -90)
                    path.closeSubpath()
                    p.drawPath(path)
                if brake_right:
                    bx0, by0 = hws, -bhy
                    path = QPainterPath()
                    path.moveTo(bx0, by0)
                    path.lineTo(bx0 + br_w - r, by0)
                    path.arcTo(bx0 + br_w - 2*r, by0, 2*r, 2*r, 90, -90)
                    path.lineTo(bx0 + br_w, by0 + br_h - r)
                    path.arcTo(bx0 + br_w - 2*r, by0 + br_h - 2*r, 2*r, 2*r, 0, -90)
                    path.lineTo(bx0, by0 + br_h)
                    path.closeSubpath()
                    p.drawPath(path)
            else:
                # Quad-axle: caliper sits enclosed between tyres — plain rectangle
                if brake_left:
                    p.drawRect(-hws - br_w, -bhy, br_w, br_h)
                if brake_right:
                    p.drawRect(hws, -bhy, br_w, br_h)

        # Negate rot_rad for the stripe phase so the tread pattern scrolls
        # rearward (toward +y in painter) when the wheel rolls forward.
        phase = -rot_rad

        bx: list[float] = []
        for k in range(N):
            alpha = (k * TWO_PI / N + phase) % TWO_PI
            if alpha > math.pi:
                alpha -= TWO_PI
            if -HALF_PI < alpha < HALF_PI:
                bx.append(hw * math.sin(alpha))
        bx.sort()

        edges = [-hw] + bx + [hw]
        sec   = int(math.floor((-HALF_PI - phase) * N / TWO_PI)) % N

        p.setPen(Qt.PenStyle.NoPen)
        for i in range(len(edges) - 1):
            y0, y1 = round(edges[i]), round(edges[i + 1])
            if y1 - y0 < 1:
                sec = (sec + 1) % N
                continue
            p.setBrush(QBrush(_C_WHEEL if sec % 2 == 0 else _C_STRIP))
            p.drawRect(-hws, y0, 2 * hws, y1 - y0)
            sec = (sec + 1) % N

        if abs(steer_deg) > 0.01:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        p.restore()
