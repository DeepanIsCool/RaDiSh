from __future__ import annotations

import math

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QRect, QTimer
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont

from gui.tabs.vehicle_design.wheel_frame_section import resolve_frame


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

# Colours
_C_RAIL  = QColor(50,  56,  72)
_C_AXLE  = QColor(60,  68,  88)
_C_STEER = QColor(40,  90, 190)
_C_DRIVE = QColor(180, 120,  30)
_C_BOTH  = QColor(80,  140,  70)
_C_WHEEL = QColor(30,  30,  30)
_C_STRIP = QColor(85,  85,  85)
_C_WRIM  = QColor(55,  60,  76)


def _px(m: float) -> int:
    return round(m * _PX_PER_M)


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
    """

    _MINOR_STEP  = 40
    _MAJOR_EVERY = 5

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setFixedSize(800, 800)

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

        # Control inputs
        self._applied_torque: float = 0.0   # Nm, total on all drivable wheels

        # Per-side wheel rotation (for differential rendering)
        self._rot_right: float = 0.0
        self._rot_left:  float = 0.0

        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    # ── Public API ────────────────────────────────────────────────────────────

    def set_wheel_frame(self, cfg: dict) -> None:
        self._resolved = resolve_frame(cfg)
        self._update_geometry()
        self.update()

    def set_steer(self, angle_deg: float) -> None:
        self._veh_steer = angle_deg

    def set_torque(self, torque_Nm: float) -> None:
        self._applied_torque = torque_Nm

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
            tau_w  = tau / n_drive
            F_w    = tau_w / r
            N_w    = _DYN_MASS * 9.81 / self._total_wheels
            # Wider tyre → larger contact patch → higher friction ceiling
            w_fac  = min(1.5, math.sqrt(max(0.05, self._tyre_width_m) / 0.20))
            F_lim  = _DYN_MU * N_w * w_fac
            F_act  = F_w if abs(F_w) <= F_lim else math.copysign(F_lim, F_w)
            self._veh_speed += (F_act * n_drive / _DYN_MASS) * _DT

        # Rolling resistance: high drag when no torque, light drag when driving
        drag = 0.50 if abs(tau) < 0.1 else 0.05
        self._veh_speed *= (1.0 - drag * _DT)

    # ── Physics tick ─────────────────────────────────────────────────────────

    def _tick(self) -> None:
        self._apply_drive_torque()
        v = self._veh_speed

        idle = (abs(v) < 1e-4 and abs(self._vy) < 0.01
                and abs(self._yaw_rate) < 0.01 and abs(self._applied_torque) < 0.1)
        if idle:
            self._vy = self._yaw_rate = 0.0
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

        if abs(v) >= 0.5:
            # sign_v flips the effective steer for reverse so that the lateral
            # force direction (and resulting yaw) is physically correct.
            sign_v = math.copysign(1.0, v)
            v_ref  = abs(v)
            alpha_f = sign_v * delta_f - math.atan2(self._vy + self._yaw_rate * lf, v_ref)
            alpha_r = sign_v * delta_r - math.atan2(self._vy - self._yaw_rate * lr, v_ref)
        else:
            alpha_f = alpha_r = 0.0

        Fz_f = _DYN_MASS * 9.81 * lr / L
        Fz_r = _DYN_MASS * 9.81 * lf / L

        def _sat(Fy: float, Fz: float) -> float:
            lim = _DYN_MU * Fz
            return max(-lim, min(lim, Fy))

        Fy_f = _sat(_DYN_CF * alpha_f, Fz_f)
        Fy_r = _sat(_DYN_CR * alpha_r, Fz_r)

        self._vy       += ((Fy_f + Fy_r) / _DYN_MASS - v * self._yaw_rate) * _DT
        self._yaw_rate += ((lf * Fy_f - lr * Fy_r) / _DYN_IZ) * _DT

        if abs(v) > 0.1:
            lim = abs(v) * math.tan(math.radians(45.0))
            self._vy = max(-lim, min(lim, self._vy))

        if abs(v) < 2.0:
            fade = 1.0 - abs(v) / 2.0
            k = math.exp(-fade * 10.0 * _DT)
            self._vy       *= k
            self._yaw_rate *= k

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

        r = max(0.01, self._tyre_radius_m)
        TWO_PI = 2.0 * math.pi
        if self._differential == "locked":
            avg = (v_r + v_l) / 2.0
            self._rot_right = (self._rot_right + avg * _DT / r) % TWO_PI
            self._rot_left  = (self._rot_left  + avg * _DT / r) % TWO_PI
        else:
            self._rot_right = (self._rot_right + v_r * _DT / r) % TWO_PI
            self._rot_left  = (self._rot_left  + v_l * _DT / r) % TWO_PI

        self.update()

    # ── Input events ─────────────────────────────────────────────────────────

    def mouseDoubleClickEvent(self, _event) -> None:
        self._veh_x = self._veh_y = self._veh_heading = 0.0
        self._veh_speed = self._vy = self._yaw_rate = 0.0
        self._rot_right = self._rot_left = 0.0
        self.update()

    # ── Paint ─────────────────────────────────────────────────────────────────

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        W, H   = self.width(), self.height()
        cx, cy = W // 2, H // 2
        p.fillRect(0, 0, W, H, self._bg)

        cam_x = cx - self._veh_x
        cam_y = cy - self._veh_y
        self._draw_grid(p, W, H, cam_x, cam_y)

        ox, oy = int(cam_x), int(cam_y)
        if -20 < ox < W + 20 and -20 < oy < H + 20:
            p.setPen(QPen(self._orig, 1))
            p.drawLine(ox - 12, oy, ox + 12, oy)
            p.drawLine(ox, oy - 12, ox, oy + 12)

        if self._resolved:
            p.save()
            p.translate(cx, cy)
            p.rotate(math.degrees(self._veh_heading))
            self._draw_vehicle(p)
            p.restore()

        self._draw_compass(p, W)
        p.end()

    # ── Grid ─────────────────────────────────────────────────────────────────

    def _draw_grid(self, p: QPainter, W: int, H: int,
                   cam_x: float, cam_y: float) -> None:
        step  = self._MINOR_STEP
        every = self._MAJOR_EVERY
        for n in range(math.floor(-cam_x / step) - 1,
                       math.ceil((W - cam_x) / step) + 2):
            x = int(cam_x + n * step)
            if not (-1 <= x <= W + 1):
                continue
            p.setPen(QPen(self._major if n % every == 0 else self._minor, 1))
            p.drawLine(x, 0, x, H)
        for n in range(math.floor(-cam_y / step) - 1,
                       math.ceil((H - cam_y) / step) + 2):
            y = int(cam_y + n * step)
            if not (-1 <= y <= H + 1):
                continue
            p.setPen(QPen(self._major if n % every == 0 else self._minor, 1))
            p.drawLine(0, y, W, y)

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

        # ICC radius for Ackermann rendering
        sa = abs(self._veh_steer)
        R_icc = (self._wheelbase_m / math.tan(math.radians(sa))
                 * math.copysign(1.0, self._veh_steer)) if sa > 0.1 else None

        # Chassis rails
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(_C_RAIL))
        p.drawRect(-rl_half - _RAIL_W, start_y, _RAIL_W, flen_px)
        p.drawRect( rl_half,           start_y, _RAIL_W, flen_px)

        for axle in self._resolved_axles:
            pos  = axle["position"]
            ay   = start_y + round(pos * flen_px)
            wpa  = axle["wheels"]
            stbl = axle.get("steerable", False)
            drvb = axle.get("drivable",  False)

            ac = (_C_BOTH  if stbl and drvb else
                  _C_STEER if stbl else
                  _C_DRIVE if drvb else _C_AXLE)

            # Axle bar + stub axles
            p.setPen(QPen(ac, 3)); p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawLine(-tr_half, ay, tr_half, ay)
            if wpa >= 2:
                p.setPen(QPen(ac, 2))
                p.drawLine(-tr_half - _RAIL_W, ay, -tr_half, ay)
                p.drawLine( tr_half + _RAIL_W, ay,  tr_half, ay)

            # Ackermann steer angles for this axle
            if stbl and R_icc is not None:
                ay_m = (pos - 0.5) * flen_m
                d_i  = self._y_nonsteer_m - ay_m   # + = axle forward of ICC
                # Pure rear-steer: wheels face the same direction as the input.
                # (Ackermann geometry gives d_i < 0 → negative angles, but the
                # user expects right input → rear wheels point right.)
                if self._steering_mode == "rear":
                    d_i = abs(d_i)
                if abs(d_i) > 0.05:
                    ref = math.degrees(math.atan2(d_i, R_icc))
                    r_deg, l_deg = self._ackermann_pair(ref, abs(d_i), fwid_m)
                else:
                    r_deg = l_deg = 0.0
            else:
                r_deg = l_deg = 0.0

            self._draw_axle_wheels(p, 0, ay, wpa, ww_px, wh_px, tr_half,
                                   r_deg, l_deg,
                                   self._rot_right, self._rot_left)

    def _draw_axle_wheels(self, p: QPainter, cx: int, ay: int,
                          wpa: int, ww: int, wh: int, tr_half: int,
                          steer_r: float, steer_l: float,
                          rot_r: float, rot_l: float) -> None:
        n = wpa // 2
        for i in range(n):
            self._draw_wheel(p, cx + tr_half - i * (ww + _DUAL_GAP), ay, ww, wh, steer_r, rot_r)
            self._draw_wheel(p, cx - tr_half + i * (ww + _DUAL_GAP), ay, ww, wh, steer_l, rot_l)
        if wpa % 2 == 1:
            avg_rot = (rot_r + rot_l) / 2.0
            self._draw_wheel(p, cx, ay, ww, wh, (steer_r + steer_l) / 2.0, avg_rot)

    def _draw_wheel(self, p: QPainter, wx: int, ay: int,
                    ww: int, wh: int, steer_deg: float, rot_rad: float) -> None:
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

        p.setPen(QPen(_C_WRIM, 1)); p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(-hws, -hw, 2 * hws, 2 * hw)

        if abs(steer_deg) > 0.01:
            p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        p.restore()
