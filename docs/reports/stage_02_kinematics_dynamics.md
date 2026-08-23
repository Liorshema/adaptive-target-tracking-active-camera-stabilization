# Stage 02 — Kinematics & Dynamics

## Objective

Develop and validate the nominal kinematic and dynamic models for the mobile rover and the camera arm.

Stage 2 includes:

* Mobile-base kinematics
* Rigid-body dynamics
* Wheel-force relationships
* Actuator relationships
* Controlled wheel-slip modeling
* 4-DOF camera-arm kinematics
* Integrated rover-to-camera pose validation

---

## 1. Mobile Base Kinematics

The rover is modeled using a skid-steer / differential-drive approximation.

### Wheel-to-body velocity

Linear velocity:

```text
v = (r / 2) * (ω_R + ω_L)
```

Angular velocity:

```text
ω = (r / L) * (ω_R - ω_L)
```

Where:

```text
r   = wheel radius
L   = track width
ω_L = left wheel angular velocity
ω_R = right wheel angular velocity
v   = rover forward velocity
ω   = rover yaw rate
```

### Rover planar motion

The rover state is:

```text
state = [x, y, θ]
```

The state derivatives are:

```text
x_dot = v * cos(θ)

y_dot = v * sin(θ)

θ_dot = ω
```

### Numerical integration

Euler integration was used:

```text
x_next = x + x_dot * dt

y_next = y + y_dot * dt

θ_next = θ + θ_dot * dt
```

### Validation

The kinematic model was validated using:

* Straight-line motion
* Circular motion
* Turn-in-place motion

All three tests produced the expected behavior.

---

## 2. Rigid-Body Dynamics

The rover dynamic state was expanded to:

```text
state = [x, y, θ, v, ω]
```

Longitudinal acceleration:

```text
v_dot = F_x / m
```

Yaw acceleration:

```text
ω_dot = τ_z / I_z
```

Where:

```text
F_x = total longitudinal force
m   = rover mass
τ_z = yaw torque
I_z = yaw moment of inertia
```

The complete simplified dynamic model is:

```text
x_dot = v * cos(θ)

y_dot = v * sin(θ)

θ_dot = ω

v_dot = F_x / m

ω_dot = τ_z / I_z
```

The model was validated using constant longitudinal force and constant yaw torque.

---

## 3. Left / Right Force Model

Instead of applying total force and yaw torque directly, the model was extended to use left-side and right-side forces.

Total longitudinal force:

```text
F_x = F_L + F_R
```

Yaw torque:

```text
τ_z = (F_R - F_L) * (L / 2)
```

This creates the physical chain:

```text
Left / Right Forces
        ↓
Total Force + Yaw Torque
        ↓
Linear + Angular Acceleration
        ↓
Rover Motion
```

An asymmetric-force experiment successfully generated simultaneous forward acceleration and rover rotation.

---

## 4. Wheel Model

A simple wheel model converts wheel torque into longitudinal traction force.

```text
F_wheel = τ_wheel / r
```

Where:

```text
τ_wheel = wheel torque
r       = wheel radius
F_wheel = ideal traction force
```

Example validation:

```text
τ_wheel = 3.0 N*m
r       = 0.15 m

F_wheel = 20.0 N
```

The numerical result matched the analytical calculation.

---

## 5. Actuator Model

A simplified DC motor actuator model was implemented.

Motor torque:

```text
τ_motor = K_t * I
```

Where:

```text
K_t = motor torque constant
I   = motor current
```

The gearbox converts motor torque to wheel torque:

```text
τ_wheel = τ_motor * G * η
```

Where:

```text
G = gearbox ratio
η = gearbox efficiency
```

The resulting drivetrain chain is:

```text
Motor Current
     ↓
Motor Torque
     ↓
Gearbox
     ↓
Wheel Torque
     ↓
Ground Force
     ↓
Rover Dynamics
```

A full drivetrain test successfully connected:

```text
I
→ τ_motor
→ τ_wheel
→ F_wheel
→ acceleration
→ velocity
→ rover pose
```

---

## 6. Controlled Wheel Slip

A first-order controlled slip model was implemented as:

```text
F_effective = (1 - s) * F_ideal
```

Where:

```text
s = slip ratio
```

Interpretation:

```text
s = 0.0  → no slip
s = 0.3  → 30% effective-force loss
s = 1.0  → complete loss of traction
```

This is intentionally a simple controlled disturbance model rather than a complete tire-soil interaction model.

### Symmetric Slip

The same slip ratio was applied to both rover sides.

Example:

```text
s_L = 0.30
s_R = 0.30
```

The nominal model predicted greater forward motion than the slip-affected model.

The resulting residuals included:

```text
x residual
v residual
```

while:

```text
y residual     = 0
θ residual     = 0
ω residual     = 0
```

because both sides experienced equal slip.

### Asymmetric Slip

Different slip ratios were then applied:

```text
s_L = 0.10
s_R = 0.40
```

Even though both motors received the same command, the effective left and right forces became different.

Therefore:

```text
F_L ≠ F_R
```

which created a yaw torque and generated errors in:

```text
x
y
θ
v
ω
```

The residual state was represented as:

```text
residual = [e_x, e_y, e_θ, e_v, e_ω]
```

This experiment demonstrated how terrain-dependent wheel slip can create model mismatch between nominal rover physics and actual rover motion.

---

## 7. 4-DOF Camera Arm

The camera arm was upgraded from a planar 2-DOF structure to a spatial 4-DOF structure.

The selected architecture is:

```text
Base Yaw
    ↓
Shoulder Pitch
    ↓
Elbow Pitch
    ↓
Wrist Pitch
    ↓
Camera
```

The joint coordinates are:

```text
q = [q0, q1, q2, q3]
```

Where:

```text
q0 = base yaw
q1 = shoulder pitch
q2 = elbow pitch
q3 = wrist pitch
```

The current arm dimensions used in the model are:

```text
link 1 length = 0.40 m
link 2 length = 0.36 m
wrist length  = 0.10 m
```

---

## 8. Camera-Arm Forward Kinematics

The arm forward-kinematics model computes the camera position relative to the rover.

Horizontal radial reach:

```text
ρ =
l1 * cos(q1)
+ l2 * cos(q1 + q2)
+ l3 * cos(q1 + q2 + q3)
```

Camera height:

```text
z_camera =
l1 * sin(q1)
+ l2 * sin(q1 + q2)
+ l3 * sin(q1 + q2 + q3)
```

The base-yaw joint rotates the arm plane around the vertical axis:

```text
x_camera = ρ * cos(q0)

y_camera = ρ * sin(q0)
```

Camera pitch:

```text
camera_pitch = q1 + q2 + q3
```

Therefore the arm model maps:

```text
[q0, q1, q2, q3]
        ↓
[x_camera, y_camera, z_camera, camera_pitch]
```

### Validation

The arm was validated using several configurations:

* All joints at zero
* Base yaw of 90 degrees
* Vertical shoulder configuration
* General 3D pose
* Wrist-pitch camera-angle adjustment

The tests produced the expected camera positions and orientations.

---

## 9. Rover-to-World Camera Transformation

The arm forward kinematics initially produces the camera position relative to the rover.

A frame transformation was implemented to calculate the camera pose in the world frame.

Camera world position:

```text
x_camera_world =
rover_x
+ cos(rover_yaw) * arm_x
- sin(rover_yaw) * arm_y
```

```text
y_camera_world =
rover_y
+ sin(rover_yaw) * arm_x
+ cos(rover_yaw) * arm_y
```

```text
z_camera_world =
rover_base_height + arm_z
```

Camera yaw in the world frame:

```text
camera_yaw_world =
rover_yaw + q0
```

Camera pitch:

```text
camera_pitch_world =
q1 + q2 + q3
```

The final camera pose currently modeled is:

```text
camera_pose =
[x, y, z, yaw, pitch]
```

Roll is not independently controlled in the current 4-DOF architecture.

---

## 10. Full Stage 2 Integration Test

A final integrated test connected the complete modeled chain:

```text
Motor Current
    ↓
Actuator Model
    ↓
Wheel Torque
    ↓
Wheel Force
    ↓
Rover Dynamics
    ↓
Rover Position / Orientation
    ↓
4-DOF Arm Forward Kinematics
    ↓
Camera Pose in Rover Frame
    ↓
World-Frame Transformation
    ↓
Camera World Pose
```

The test used asymmetric motor currents so that the rover translated and rotated simultaneously.

The resulting rover state was physically consistent, and the calculated camera orientation satisfied:

```text
camera_yaw_world =
rover_yaw + base_yaw
```

and:

```text
camera_pitch =
shoulder_pitch
+ elbow_pitch
+ wrist_pitch
```

The final integrated test completed successfully.

---

## 11. Implemented Files

The main Stage 2 implementation includes:

```text
rover_dynamics/
├── kinematic_model.py
├── dynamic_model.py
├── wheel_model.py
├── actuator_model.py
├── slip_model.py
├── arm_kinematics.py
└── camera_world_pose.py
```

Validation scripts include tests for:

```text
kinematics
rigid-body dynamics
wheel model
actuator model
wheel + dynamics integration
full drivetrain
symmetric slip
asymmetric slip
camera-arm kinematics
camera world pose
full Stage 2 integration
```

---

## 12. Stage 2 Completion

Completed:

* Mobile-base kinematics
* Numerical trajectory integration
* Straight / circular / turn-in-place validation
* Simplified rigid-body dynamics
* Left / right force relationships
* Wheel torque-to-force model
* DC motor / gearbox actuator model
* Full drivetrain model
* Controlled symmetric wheel slip
* Controlled asymmetric wheel slip
* Residual-error demonstration
* 4-DOF camera-arm architecture
* Camera-arm forward kinematics
* Camera position and orientation
* Rover-to-world camera transformation
* Full integrated Stage 2 validation

The project-stage completion criterion requires the mobile-base and arm equations to be documented, implemented, and validated on simple trajectories, including a controlled wheel-slip disturbance. Those requirements have been satisfied.

**Stage 2 Status: COMPLETE**
