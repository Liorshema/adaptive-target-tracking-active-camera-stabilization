import math

from rover_dynamics.arm_kinematics import ArmKinematics
from rover_dynamics.camera_world_pose import camera_pose_in_world


arm = ArmKinematics(
    link1_length=0.40,
    link2_length=0.36,
    wrist_length=0.10
)


# Rover pose
rover_x = 2.0
rover_y = 1.0
rover_yaw = math.radians(30)


# Arm configuration
q0 = math.radians(45)    # base yaw
q1 = math.radians(20)    # shoulder
q2 = math.radians(-30)   # elbow
q3 = math.radians(15)    # wrist


# Arm FK
arm_x, arm_y, arm_z, camera_pitch = arm.forward_kinematics(
    q0,
    q1,
    q2,
    q3
)


# Camera pose in world
(
    camera_x,
    camera_y,
    camera_z,
    camera_yaw,
    camera_pitch_world
) = camera_pose_in_world(
    rover_x,
    rover_y,
    rover_yaw,
    arm_x,
    arm_y,
    arm_z,
    q0,
    camera_pitch
)


print("----- ROVER -----")
print("x =", rover_x)
print("y =", rover_y)
print("yaw =", math.degrees(rover_yaw), "deg")
print()


print("----- ARM -----")
print("base yaw =", math.degrees(q0), "deg")
print("shoulder =", math.degrees(q1), "deg")
print("elbow =", math.degrees(q2), "deg")
print("wrist =", math.degrees(q3), "deg")
print()


print("----- CAMERA WORLD POSE -----")
print("x =", camera_x, "m")
print("y =", camera_y, "m")
print("z =", camera_z, "m")

print(
    "yaw =",
    math.degrees(camera_yaw),
    "deg"
)

print(
    "pitch =",
    math.degrees(camera_pitch_world),
    "deg"
)