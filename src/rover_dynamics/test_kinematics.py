import math

from rover_dynamics.kinematic_model import RoverKinematicModel


model = RoverKinematicModel(
    wheel_radius=0.15,
    track_width=0.60
)


# --------------------------------------------------
# Test 1 - Straight line
# --------------------------------------------------

state = [0.0, 0.0, 0.0]

omega_left = 2.0
omega_right = 2.0

dt = 0.1
simulation_time = 10.0
steps = int(simulation_time / dt)

for _ in range(steps):
    state = model.step(
        state,
        omega_left,
        omega_right,
        dt
    )

print("Test 1 - Straight line")
print("x =", state[0])
print("y =", state[1])
print("theta =", state[2])
print()


# --------------------------------------------------
# Test 2 - Circular motion
# --------------------------------------------------

state = [0.0, 0.0, 0.0]

omega_left = 1.0
omega_right = 2.0

for _ in range(steps):
    state = model.step(
        state,
        omega_left,
        omega_right,
        dt
    )

print("Test 2 - Circular motion")
print("x =", state[0])
print("y =", state[1])
print("theta =", state[2])
print()


# --------------------------------------------------
# Test 3 - Turn in place
# --------------------------------------------------

state = [0.0, 0.0, 0.0]

omega_left = -2.0
omega_right = 2.0

for _ in range(steps):
    state = model.step(
        state,
        omega_left,
        omega_right,
        dt
    )

print("Test 3 - Turn in place")
print("x =", state[0])
print("y =", state[1])
print("theta =", state[2])