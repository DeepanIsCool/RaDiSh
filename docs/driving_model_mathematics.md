# LANCER Organic AV Training Simulator
## Driving Model — Mathematical Foundation
### Vehicle Design Viewport · v9.1 Implementation Reference

---

## 1. Coordinate System and Sign Conventions

The world is a 2-D top-down plane rendered at **40 px / m**.

| Axis | Positive direction |
|------|--------------------|
| $x_\text{world}$ | East (screen-right) |
| $y_\text{world}$ | South (screen-down — $y$ increases downward) |

The **vehicle frame** is attached to the vehicle and rotates with it:

| Axis | Direction |
|------|-----------|
| $+x_v$ | Forward (longitudinal) |
| $+y_v$ | Left (lateral, SAE convention) |

**Heading** $\psi$ (rad) is measured **clockwise from North** (screen-up):
- $\psi = 0$ → vehicle faces up (North)
- $\psi = \pi/2$ → vehicle faces right (East)

### State Variables

| Symbol | Unit | Description |
|--------|------|-------------|
| $v$ | m/s | Longitudinal speed, $+$ = forward |
| $v_y$ | m/s | Lateral speed in vehicle frame, $+$ = left |
| $\dot\psi$ | rad/s | Yaw rate, $+$ = counter-clockwise (CCW) |
| $\psi$ | rad | Heading, CW from North |
| $x, y$ | px | World pixel position of vehicle centre |

### Driver Inputs

| Symbol | Unit | Description |
|--------|------|-------------|
| $\delta_\text{ref}$ | deg | Reference steer angle, $+$ = right |
| $\tau$ | N·m | Total torque on all drivable wheels, $+$ = forward |

### Physical Constants

| Symbol | Value | Unit | Description |
|--------|-------|------|-------------|
| $m$ | 1 500 | kg | Vehicle mass |
| $I_z$ | 2 500 | kg·m² | Yaw moment of inertia about CG |
| $C_f$ | 60 000 | N/rad | Front axle cornering stiffness |
| $C_r$ | 60 000 | N/rad | Rear axle cornering stiffness |
| $\mu$ | 0.85 | — | Peak tyre–road friction coefficient |
| $\Delta t$ | 0.016 | s | Physics timestep (≈ 60 Hz) |
| $g$ | 9.81 | m/s² | Gravitational acceleration |

---

## 2. Vehicle Geometry

Axle positions are specified as a fraction $p \in [0, 1]$ along the frame from front to rear. The signed longitudinal offset from the CG to axle $k$ is:

$$y_k = \left(p_k - 0.5\right) L_\text{frame} \tag{1}$$

Negative $y_k$ means the axle is **ahead** of the CG; positive means **behind**.

The bicycle model uses two effective distances:

$$l_f = \left|\overline{y}_\text{steer}\right|, \qquad l_r = \left|\overline{y}_\text{nonsteer}\right| \tag{2}$$

$$L = l_f + l_r \qquad \text{(effective wheelbase)} \tag{3}$$

where $\overline{y}$ denotes the mean position of the respective axle group.

**Special cases:**
- *All-wheel steer* with no passive axle: $l_f$ and $l_r$ span front to rear steer axle; $L$ is the distance between the two steer groups.
- If no steerable axle is tagged, the frontmost axle is promoted to steerable for model stability.

---

## 3. Longitudinal Dynamics

### 3.1 Torque Distribution

The user supplies a single total torque $\tau$. It is split equally among all $n_\text{drive}$ drivable wheels:

$$\tau_w = \frac{\tau}{n_\text{drive}} \tag{4}$$

### 3.2 Longitudinal Tyre Force per Wheel

Each driven wheel converts torque to a ground force through the rolling radius $r$:

$$F_w = \frac{\tau_w}{r} \tag{5}$$

### 3.3 Normal Load per Wheel

Normal load is distributed uniformly across all $n_\text{total}$ wheels (no longitudinal load transfer):

$$N_w = \frac{mg}{n_\text{total}} \tag{6}$$

### 3.4 Tyre Width Friction Scaling

A wider contact patch supports a higher friction force. Derived from contact-patch area arguments (area $\propto$ width):

$$w_\text{fac} = \min\!\left(1.5,\ \sqrt{\frac{w_\text{tyre}}{0.20}}\right) \tag{7}$$

where $w_\text{tyre}$ is the tyre width in metres and $0.20\ \text{m}$ is the reference width. The factor is clamped at $1.5$, reached at $w_\text{tyre} \approx 0.45\ \text{m}$.

### 3.5 Friction Limit and Traction Saturation (Longitudinal Skid)

The maximum longitudinal force before tyre slip:

$$F_\text{lim} = \mu\, N_w\, w_\text{fac} \tag{8}$$

The delivered force is saturated (clamped):

$$F_\text{act} = \begin{cases} F_w & \text{if } |F_w| \leq F_\text{lim} \\ \operatorname{sign}(F_w)\cdot F_\text{lim} & \text{otherwise} \end{cases} \tag{9}$$

When $|F_w| > F_\text{lim}$ the wheel is **spinning** (drive) or **locking** (brake) — the longitudinal skid condition.

### 3.6 Speed Update

Newton's second law along the longitudinal axis:

$$v \leftarrow v + \frac{F_\text{act}\, n_\text{drive}}{m}\,\Delta t \tag{10}$$

### 3.7 Rolling Resistance

A first-order viscous drag is applied after the torque step:

$$v \leftarrow v\left(1 - k_\text{drag}\,\Delta t\right) \tag{11}$$

$$k_\text{drag} = \begin{cases} 0.50 & |\tau| < 0.1\ \text{N·m} \quad\text{(coasting)} \\ 0.05 & \text{otherwise} \quad\text{(driving)} \end{cases} \tag{12}$$

The $0.50$ value approximates combined tyre rolling resistance and mild aerodynamic drag at low speed; $0.05$ represents residual rolling resistance when the drive torque overcomes most drag.

---

## 4. Lateral Dynamics — Linear Bicycle Model

### 4.1 Model Overview

The vehicle is reduced to a **single-track (bicycle) model**: one equivalent front wheel and one equivalent rear wheel, each at the centroid of their respective axle group. Lateral forces are linear functions of slip angle, saturated at the friction limit.

### 4.2 Steer Angle Distribution

The driver's reference angle $\delta_\text{ref}$ is sign-flipped to the model convention ($+$ = left) and distributed by steering mode:

$$\delta = -\operatorname{radians}(\delta_\text{ref})$$

| Mode | $\delta_f$ | $\delta_r$ |
|------|-----------|-----------|
| `front` | $\delta$ | $0$ |
| `rear` | $0$ | $\delta$ |
| `both` | $\delta$ | $-\delta$ |

For `both` mode the rear axle **counter-steers**, halving the effective wheelbase and approximately doubling yaw authority.

### 4.3 Tyre Slip Angles

The slip angle $\alpha$ is the angle between a wheel's heading and its velocity direction. It is the primary driver of lateral cornering force.

**Front** equivalent wheel at distance $l_f$ ahead of the CG:

$$\alpha_f = \operatorname{sign}(v)\cdot\delta_f - \arctan\!\left(\frac{v_y + \dot\psi\, l_f}{|v|}\right) \tag{13}$$

**Rear** equivalent wheel at distance $l_r$ behind the CG:

$$\alpha_r = \operatorname{sign}(v)\cdot\delta_r - \arctan\!\left(\frac{v_y - \dot\psi\, l_r}{|v|}\right) \tag{14}$$

The $\operatorname{sign}(v)$ factor ensures slip angles reverse correctly during reverse motion, so that cornering forces still oppose lateral slip regardless of direction.

These expressions are evaluated only when $|v| \geq 0.5\ \text{m/s}$. Below this speed a fade damper stabilises the model (Section 4.7).

**Physical interpretation of each term:**

| Term | Meaning |
|------|---------|
| $\operatorname{sign}(v)\cdot\delta$ | Wheel heading offset from the vehicle axis |
| $\arctan(\cdots)$ | Direction of the contact-point velocity in the vehicle frame |

### 4.4 Normal Load — Quasi-static Distribution

$$N_f = \frac{mg\, l_r}{L}, \qquad N_r = \frac{mg\, l_f}{L} \tag{15}$$

No dynamic load transfer is modelled. A heavier rear bias (small $l_r$, large $l_f$) places more normal load on the front axle.

### 4.5 Cornering Force — Linear Model with Friction Saturation

Linear regime:

$$F_{y,f}^\text{lin} = C_f\,\alpha_f, \qquad F_{y,r}^\text{lin} = C_r\,\alpha_r \tag{16}$$

$C_f$ and $C_r$ are **total axle stiffnesses** (N/rad), not per-wheel values.

Saturation at the Coulomb friction limit:

$$F_{y,f} = \operatorname{clamp}\!\left(F_{y,f}^\text{lin},\ \pm\,\mu N_f\right) \tag{17}$$

$$F_{y,r} = \operatorname{clamp}\!\left(F_{y,r}^\text{lin},\ \pm\,\mu N_r\right) \tag{18}$$

When a tyre is saturated the lateral force is limited to $\mu N_\text{axle}$ — the tyre is in a **lateral skid** (sideslip / breakaway).

### 4.6 Equations of Motion

**Lateral translation** (Newton's second law in vehicle-frame $y$):

$$m\dot v_y = F_{y,f} + F_{y,r} - mv\dot\psi \tag{19}$$

**Yaw rotation** (about the CG):

$$I_z\ddot\psi = l_f F_{y,f} - l_r F_{y,r} \tag{20}$$

The term $-mv\dot\psi$ in (19) is the **centripetal coupling term**: when the vehicle rotates, the centripetal acceleration required for circular motion is subtracted from the net lateral force to obtain the lateral acceleration in the vehicle frame.

**Discrete-time Euler integration:**

$$v_y \leftarrow v_y + \left[\frac{F_{y,f} + F_{y,r}}{m} - v\dot\psi\right]\Delta t \tag{21}$$

$$\dot\psi \leftarrow \dot\psi + \frac{l_f F_{y,f} - l_r F_{y,r}}{I_z}\,\Delta t \tag{22}$$

### 4.7 Low-Speed Stabilisation

Below $2\ \text{m/s}$ the linear bicycle model becomes numerically stiff because slip angles diverge as $v \to 0$. An exponential fade is applied:

$$\text{fade} = 1 - \frac{|v|}{2.0} \in [0,\,1] \quad \text{for } |v| < 2\ \text{m/s} \tag{23}$$

$$k = \exp\!\left(-\text{fade}\cdot 10\cdot\Delta t\right) \tag{24}$$

$$v_y \leftarrow v_y\cdot k, \qquad \dot\psi \leftarrow \dot\psi\cdot k \tag{25}$$

At $v = 0$: $k \approx 0.852$ per tick, so $\dot\psi \to 0$ within about 0.1 s.  
At $v = 2\ \text{m/s}$: $k = 1$ (no damping applied).

### 4.8 Lateral Velocity Clamp

A hard clamp prevents unphysically large sideslip at high speed:

$$|v_y| \leq |v|\tan(45°) = |v| \tag{26}$$

This limits the sideslip angle $\beta = \arctan(v_y / v)$ to $\pm 45°$. Deeper sideslip is considered a full spin condition beyond the scope of the linear model.

---

## 5. Understeer, Oversteer, and Neutral Steer

### 5.1 Understeer Gradient

In steady-state circular motion ($\dot v_y = 0$, $\ddot\psi = 0$) the required steer angle deviates from the neutral (kinematic) steer angle $L/R$ by an amount proportional to lateral acceleration. The **understeer gradient** $K_\text{us}$ [rad·s²/m] quantifies this deviation:

$$K_\text{us} = \frac{m}{L}\left(\frac{l_r}{C_r} - \frac{l_f}{C_f}\right) \tag{27}$$

Substituting the model constants ($C_f = C_r = 60\,000\ \text{N/rad}$):

$$K_\text{us} = \frac{1500}{60\,000\, L}(l_r - l_f) = \frac{0.025}{L}(l_r - l_f)$$

For the default symmetric geometry ($l_f = l_r = 1.5\ \text{m}$, $L = 3.0\ \text{m}$):

$$K_\text{us} = 0 \qquad \Rightarrow \quad \textbf{neutral steer}$$

### 5.2 Stability Conditions

| Condition | Behaviour |
|-----------|-----------|
| $K_\text{us} > 0$ | **Understeering** — front saturates first; vehicle plows straight at high speed; self-stabilising |
| $K_\text{us} = 0$ | **Neutral steer** — balanced response at all speeds |
| $K_\text{us} < 0$ | **Oversteering** — rear saturates first; directionally unstable above $v_\text{crit}$ |

### 5.3 Critical Speed (Oversteer)

For $K_\text{us} < 0$ there is a speed at which yaw damping vanishes:

$$v_\text{crit} = \sqrt{\frac{-gL}{K_\text{us}}} \tag{28}$$

Above $v_\text{crit}$ the vehicle diverges in yaw without active corrective steer. For the symmetric default, $K_\text{us} = 0$ and $v_\text{crit} = \infty$.

### 5.4 Runtime Identification via Saturation

The saturation model (eq. 17–18) provides a direct observable at runtime:

- $F_{y,f}$ clamped, $F_{y,r}$ not → **front slides first → understeer** (vehicle pushes wide)
- $F_{y,r}$ clamped, $F_{y,f}$ not → **rear slides first → oversteer** (rear pivots out, large $\dot\psi$ spike)
- Both clamped simultaneously → **four-wheel drift**

---

## 6. World Position Integration

The vehicle velocity in the world frame is a rotation of the body-frame velocity by heading $\psi$. In the Qt screen convention ($y_\text{screen}$ increases downward):

$$\dot x_\text{world} = v\sin\psi - v_y\cos\psi \tag{29}$$

$$\dot y_\text{world} = -v\cos\psi - v_y\sin\psi \tag{30}$$

The minus sign in (30) arises because the forward direction at $\psi = 0$ points screen-up (decreasing $y_\text{screen}$).

Positions are accumulated in **pixel units**:

$$x_\text{px} \leftarrow x_\text{px} + \dot x_\text{world}\cdot\text{PX/M}\cdot\Delta t \tag{31}$$

$$y_\text{px} \leftarrow y_\text{px} + \dot y_\text{world}\cdot\text{PX/M}\cdot\Delta t \tag{32}$$

**Heading update** (clockwise convention):

$$\psi \leftarrow (\psi - \dot\psi\,\Delta t) \bmod 2\pi \tag{33}$$

The minus sign: a positive yaw rate (CCW in physics) decreases $\psi$ (measured clockwise).

---

## 7. Ackermann Steering Geometry

### 7.1 Motivation

For a vehicle to turn without tyre scrub, all wheels must rotate about the same **Instantaneous Centre of Curvature (ICC)**. This requires the inner wheel to steer more sharply than the outer wheel. Ackermann geometry provides the exact per-wheel angles for any number of steer axles.

### 7.2 ICC Radius from the Reference Steer Angle

$$R_\text{ICC} = \frac{L}{\tan|\delta_\text{ref}|}\cdot\operatorname{sign}(\delta_\text{ref}) \tag{34}$$

$R_\text{ICC} > 0$ for a right turn; $R_\text{ICC} < 0$ for a left turn.

### 7.3 Per-Axle Reference Angle

For each steerable axle at longitudinal offset $y_\text{axle}$ from the CG, the distance from the axle to the ICC level is:

$$d_i = y_\text{nonsteer} - y_\text{axle} \tag{35}$$

where $y_\text{nonsteer}$ is the longitudinal position of the non-steerable axle centroid (the ICC level for kinematic steering).

The effective reference angle for this axle is:

$$\delta_\text{axle} = \arctan\!\left(\frac{d_i}{|R_\text{ICC}|}\right)\cdot\operatorname{sign}(R_\text{ICC}) \tag{36}$$

> **Note:** Using $|R_\text{ICC}|$ in the denominator with an explicit sign multiplication is critical. Passing the signed $R_\text{ICC}$ directly to $\text{atan2}$ maps left turns ($R_\text{ICC} < 0$, $d_i > 0$) into the second quadrant ($\approx 171°$) rather than the intended small negative angle — the source of a previously corrected rendering bug.

### 7.4 Inner and Outer Wheel Angles (Ackermann Pair)

Given $\delta_\text{axle}$ and track width $T$, the ICC radius from the axle centreline is:

$$R_\text{ref} = \frac{|d_i|}{\tan|\delta_\text{axle}|} \tag{37}$$

The inner wheel (closer to ICC) steers more sharply than the outer:

$$\delta_\text{inner} = \arctan\!\left(\frac{|d_i|}{R_\text{ref} - T/2}\right) \tag{38}$$

$$\delta_\text{outer} = \arctan\!\left(\frac{|d_i|}{R_\text{ref} + T/2}\right) \tag{39}$$

Assigning left and right with the correct sign:

| Turn direction | Right wheel | Left wheel |
|---------------|-------------|------------|
| Right ($\operatorname{sign} > 0$) | $+\delta_\text{inner}$ (inner) | $+\delta_\text{outer}$ (outer) |
| Left ($\operatorname{sign} < 0$) | $-\delta_\text{outer}$ (outer) | $-\delta_\text{inner}$ (inner) |

### 7.5 Dual-Axle Steer Groups

When multiple axles share a steering group (e.g. tandem steer), each axle computes its own $d_i$ and therefore its own Ackermann pair independently. Axles closer to the ICC require a sharper turn — the geometry is solved per-axle.

---

## 8. Differential Model and Wheel Rotation

### 8.1 Per-Side Ground Speed During a Turn

When the vehicle turns with reference steer angle $\delta_\text{ref}$ and track width $T$:

$$R_t = \frac{L}{\tan|\delta_\text{ref}|} \tag{40}$$

For a right turn ($\operatorname{sign} = +1$), the right side is the inner side:

$$v_R = v\cdot\frac{R_t - T/2}{R_t} \qquad \text{(inner — shorter path)} \tag{41}$$

$$v_L = v\cdot\frac{R_t + T/2}{R_t} \qquad \text{(outer — longer path)} \tag{42}$$

For a left turn the inner/outer assignments flip by sign.

### 8.2 Open Differential

Each side is free to spin at its own speed. The wheel rotation angle accumulated for the tread-stripe animation:

$$\theta_R \leftarrow \left(\theta_R + \frac{v_R\,\Delta t}{r}\right) \bmod 2\pi \tag{43}$$

$$\theta_L \leftarrow \left(\theta_L + \frac{v_L\,\Delta t}{r}\right) \bmod 2\pi \tag{44}$$

### 8.3 Locked Differential

Both sides are constrained to the same rotational speed. The common speed is the average of the ideal individual speeds:

$$v_\text{avg} = \frac{v_R + v_L}{2} = v \tag{45}$$

$$\theta_R = \theta_L \leftarrow \left(\theta + \frac{v_\text{avg}\,\Delta t}{r}\right) \bmod 2\pi \tag{46}$$

**Effect:** during a turn the locked diff forces the inner tyre to spin faster than its ideal speed and the outer tyre slower, creating a net understeer moment and tyre scrub.

### 8.4 Passive (Trailer) Wheels

Trailer axles are non-driven and non-steered. A single shared rotation angle is accumulated at the tractor's forward speed:

$$\theta_\text{trailer} \leftarrow \left(\theta_\text{trailer} + \frac{v\,\Delta t}{r}\right) \bmod 2\pi \tag{47}$$

---

## 9. Fifth-Wheel Kinematic Trailer Model

### 9.1 System Description

An articulating trailer is coupled to the tractor via a fifth-wheel hitch at fractional position $p_\text{hitch}$ along the tractor frame. The kingpin is a pin joint: the trailer's front is constrained to follow the hitch point but is free to rotate about the vertical axis.

| Symbol | Unit | Description |
|--------|------|-------------|
| $\psi_t$ | rad | Trailer heading (same CW-from-North convention) |
| $L_t$ | m | Kingpin distance: hitch to trailer axle centroid |
| $\varphi = \psi_t - \psi$ | rad | Articulation angle |
| $\varphi_\text{max}$ | rad | Maximum articulation angle (user-configurable) |

### 9.2 Kinematic Constraint

The trailer rolls on passive wheels without lateral slip. The **no-lateral-slip constraint** at the trailer axle centroid produces a first-order ODE for the trailer heading:

$$\frac{d\psi_t}{dt} = \frac{v\sin(\psi - \psi_t)}{L_t} \tag{48}$$

**Derivation:** The hitch point moves at speed $v$ along the tractor heading $\psi$. The component of that velocity **perpendicular** to the trailer heading $\psi_t$ is:

$$v_\perp = v\sin(\psi - \psi_t)$$

For the no-slip constraint, the axle must subtend the same perpendicular rate:

$$v_\perp = L_t\,\frac{d\psi_t}{dt} \implies \frac{d\psi_t}{dt} = \frac{v\sin(\psi - \psi_t)}{L_t}$$

**Convergence:** when $\psi_t = \psi$ (trailer aligned with tractor) the right-hand side is zero — stable equilibrium. When $\psi_t \neq \psi$ the trailer is pulled toward the tractor heading at a rate proportional to $v$ and inversely proportional to $L_t$.

### 9.3 Discrete-Time Update

The heading difference, normalised to $(-\pi,\pi]$:

$$\Delta\psi = \bigl((\psi - \psi_t) + \pi\bigr) \bmod 2\pi - \pi \tag{49}$$

First-order Euler step:

$$\psi_t \leftarrow \psi_t + \frac{v\sin(\Delta\psi)}{L_t}\,\Delta t \tag{50}$$

### 9.4 Articulation Angle Clamping

$$\varphi = \bigl((\psi_t - \psi) + \pi\bigr) \bmod 2\pi - \pi \tag{51}$$

$$\varphi_\text{clamped} = \operatorname{clamp}(\varphi,\ -\varphi_\text{max},\ +\varphi_\text{max}) \tag{52}$$

$$\psi_t = \psi + \varphi_\text{clamped} \tag{53}$$

This prevents jack-knifing beyond the mechanical articulation stop.

### 9.5 Hitch Screen Position

In the vehicle-local painter frame (origin at CG, $+y$ rearward), the hitch is at:

$$h_\text{local} = (p_\text{hitch} - 0.5)\cdot L_\text{frame}\cdot\text{PX/M} \quad \text{[px]} \tag{54}$$

After rotation by heading $\psi$, the hitch screen position is:

$$h_x = c_x - h_\text{local}\sin\psi \tag{55}$$

$$h_y = c_y + h_\text{local}\cos\psi \tag{56}$$

The trailer is drawn translated to $(h_x, h_y)$ and rotated by $\psi_t$, with $y = 0$ at the hitch and $y > 0$ rearward along the trailer.

### 9.6 Trailing Behaviour Summary

| Condition | Behaviour |
|-----------|-----------|
| Straight driving | $\psi_t \to \psi$ (trailer aligns with tractor) |
| Steady-state turn | $\varphi_\text{steady} \approx L_{t,\text{eff}} / R_\text{tractor}$ for small angles |
| Jack-knife | Prevented by $\varphi_\text{max}$ clamp (analogous to real articulation stops, typically $45°$–$90°$) |
| Reversing | $v < 0$ reverses sign of $d\psi_t/dt$: trailer diverges (jack-knife risk), correct kinematic behaviour |

---

## 10. Idle Detection and Early Exit

To avoid numerical drift and unnecessary redraws when the vehicle is at rest:

$$\text{idle} \iff |v| < 10^{-4}\ \text{m/s} \;\land\; |v_y| < 0.01\ \text{m/s} \;\land\; |\dot\psi| < 0.01\ \text{rad/s} \;\land\; |\tau| < 0.1\ \text{N·m} \tag{57}$$

When idle, $v_y$ and $\dot\psi$ are zeroed and the tick exits early.

---

## 11. Complete Tick Sequence

Each $\Delta t = 0.016\ \text{s}$ the following steps execute in order:

| Step | Operation | Equations |
|------|-----------|-----------|
| 1 | **Torque chain:** $\tau_w \to F_w \to F_\text{lim} \to F_\text{act} \to v$ | (4) – (10) |
| 2 | **Rolling drag:** $v \leftarrow v(1 - k_\text{drag}\,\Delta t)$ | (11) – (12) |
| 3 | **Idle check** — early return if idle | (57) |
| 4 | **Steer distribution:** $\delta_f,\, \delta_r$ from steering mode | §4.2 |
| 5 | **Slip angles:** $\alpha_f,\, \alpha_r$ | (13) – (14) |
| 6 | **Normal loads:** $N_f,\, N_r$ | (15) |
| 7 | **Cornering forces with saturation:** $F_{y,f},\, F_{y,r}$ | (16) – (18) |
| 8 | **Equations of motion (Euler):** $v_y,\, \dot\psi$ | (21) – (22) |
| 9 | **Lateral velocity clamp** | (26) |
| 10 | **Low-speed fade** (when $|v| < 2\ \text{m/s}$) | (23) – (25) |
| 11 | **World position integration:** $x_\text{px},\, y_\text{px},\, \psi$ | (31) – (33) |
| 12 | **Differential wheel rotation:** $\theta_R,\, \theta_L$ | (43) – (46) |
| 13 | **Fifth-wheel kinematic update** (if enabled): $\psi_t,\, \theta_\text{trailer}$ | (49) – (53), (47) |
| 14 | **Repaint viewport** | — |

---

## 12. Model Limitations and Known Approximations

### 12.1 No Combined-Slip Friction Ellipse
The longitudinal and lateral friction limits are enforced independently. Combined slip (e.g. braking while cornering) does not reduce the lateral friction ceiling. A full model would enforce:

$$\left(\frac{F_x}{F_{x,\text{max}}}\right)^2 + \left(\frac{F_y}{F_{y,\text{max}}}\right)^2 \leq 1$$

### 12.2 No Dynamic Load Transfer
Normal load is static. A full model adds:

$$\Delta N_\text{lat} = \frac{m\, a_y\, h_\text{CG}}{T_\text{track}}, \qquad \Delta N_\text{lon} = \frac{m\, a_x\, h_\text{CG}}{L}$$

### 12.3 Constant Cornering Stiffness
$C_f$ and $C_r$ are fixed. Real tyres follow the **Pacejka Magic Formula** where stiffness varies with slip angle, normal load, and camber. The saturation clamp approximates the Pacejka peak without modelling the drop-off beyond it.

### 12.4 Tyre Width Affects Only Longitudinal Limit
Width scales $F_\text{lim}$ (eq. 7) but not $C_f$ or $C_r$. In reality, a wider contact patch also increases lateral cornering stiffness.

### 12.5 Linear Bicycle Model Valid for Small Slip Angles
The linear force law (eq. 16) is accurate for $|\alpha| \lesssim 5°$–$8°$. At larger slip angles it overpredicts cornering force. The saturation (eq. 17–18) mitigates this but does not reproduce the Pacejka shoulder.

### 12.6 Massless Kinematic Trailer
The trailer has no yaw inertia. In reality, trailer inertia damps high-frequency articulation oscillations and delays jack-knife onset.

### 12.7 Symmetric Cornering Stiffness
$C_f = C_r$ gives $K_\text{us} = 0$ (neutral steer) for symmetric geometry. Real passenger vehicles are typically slightly understeering by design ($K_\text{us} \approx 0.002$–$0.005\ \text{rad·s}^2/\text{m}$).

---

## 13. Notation Summary

| Symbol | Unit | Description |
|--------|------|-------------|
| $v$ | m/s | Longitudinal speed, $+$ = forward |
| $v_y$ | m/s | Lateral speed in vehicle frame, $+$ = left |
| $\psi$ | rad | Heading, CW from North |
| $\dot\psi$ | rad/s | Yaw rate, $+$ = CCW |
| $\delta_\text{ref}$ | deg | Driver reference steer angle, $+$ = right |
| $\delta$ | rad | Steer angle in model, $+$ = left |
| $\delta_f,\,\delta_r$ | rad | Front / rear steer angles |
| $\alpha_f,\,\alpha_r$ | rad | Front / rear tyre slip angles |
| $F_{y,f},\,F_{y,r}$ | N | Lateral cornering force, front / rear |
| $N_f,\,N_r$ | N | Normal load, front / rear |
| $F_w$ | N | Demanded longitudinal force per wheel |
| $F_\text{act}$ | N | Actual (clamped) longitudinal force per wheel |
| $F_\text{lim}$ | N | Friction limit per wheel |
| $\tau$ | N·m | Total input torque |
| $\tau_w$ | N·m | Torque per driven wheel |
| $r$ | m | Tyre rolling radius |
| $L$ | m | Effective wheelbase |
| $l_f,\,l_r$ | m | CG to front / rear equivalent axle |
| $T$ | m | Steer track width |
| $R_\text{ICC}$ | m | ICC radius, $+$ = right turn |
| $R_t$ | m | Turn radius at CG |
| $K_\text{us}$ | rad·s²/m | Understeer gradient |
| $v_\text{crit}$ | m/s | Critical speed (oversteer instability) |
| $m$ | kg | Vehicle mass (1 500) |
| $I_z$ | kg·m² | Yaw moment of inertia (2 500) |
| $C_f,\,C_r$ | N/rad | Cornering stiffness, front / rear (60 000 each) |
| $\mu$ | — | Peak friction coefficient (0.85) |
| $w_\text{fac}$ | — | Tyre width friction scaling factor |
| $\Delta t$ | s | Physics timestep (0.016) |
| $\psi_t$ | rad | Trailer heading |
| $L_t$ | m | Kingpin distance: hitch to trailer axle centroid |
| $\varphi$ | rad | Articulation angle $\psi_t - \psi$ |
| $\varphi_\text{max}$ | rad | Maximum articulation angle |
| $p_\text{hitch}$ | — | Hitch position as fraction of tractor frame $[0,\,\infty)$ |
| PX/M | px/m | Viewport scale (40) |
