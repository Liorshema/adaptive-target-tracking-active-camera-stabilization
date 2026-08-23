import math

from rover_dynamics.arm_kinematics import ArmKinematics


arm = ArmKinematics(
    link1_length=0.40,
    link2_length=0.36,
    wrist_length=0.10
)


# Test 1
q0 = 0.0
q1 = 0.0
q2 = 0.0
q3 = 0.0

x, y, z, pitch = arm.forward_kinematics(
    q0,
    q1,
    q2,
    q3
)

print("Test 1 - Straight forward")
print("x =", x, "m")
print("y =", y, "m")
print("z =", z, "m")
print("camera pitch =", pitch, "rad")
print()


# Test 2
q0 = math.pi / 2
q1 = 0.0
q2 = 0.0
q3 = 0.0

x, y, z, pitch = arm.forward_kinematics(
    q0,
    q1,
    q2,
    q3
)

print("Test 2 - Base yaw 90 deg")
print("x =", x, "m")
print("y =", y, "m")
print("z =", z, "m")
print("camera pitch =", pitch, "rad")
print()


# Test 3
q0 = 0.0
q1 = math.pi / 2
q2 = 0.0
q3 = 0.0

x, y, z, pitch = arm.forward_kinematics(
    q0,
    q1,
    q2,
    q3
)

print("Test 3 - Arm vertical")
print("x =", x, "m")
print("y =", y, "m")
print("z =", z, "m")
print("camera pitch =", pitch, "rad")
print()


# Test 4
q0 = 0.0
q1 = math.radians(30)
q2 = math.radians(-30)
q3 = math.radians(-45)

x, y, z, pitch = arm.forward_kinematics(
    q0,
    q1,
    q2,
    q3
)

print("Test 4 - Wrist pitch")
print("x =", x, "m")
print("y =", y, "m")
print("z =", z, "m")
print("camera pitch =", math.degrees(pitch), "deg")