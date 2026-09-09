import math

from robot_dynamics.actuator_model import ActuatorModel
from robot_dynamics.wheel_model import WheelModel
from robot_dynamics.dynamic_model import RobotDynamicModel
from robot_dynamics.arm_kinematics import ArmKinematics
from robot_dynamics.camera_world_pose import camera_pose_in_world


# --------------------------------------------------
# Robot physical parameters
# --------------------------------------------------

wheel_radius = 0.15
track_width = 0.68
mass = 20.0
yaw_inertia = 2.0


# --------------------------------------------------
# Actuator parameters
# --------------------------------------------------

torque_constant = 0.02
gear_ratio = 50.0
efficiency = 0.80


# --------------------------------------------------
# Create models
# --------------------------------------------------

actuator = ActuatorModel(
    torque_constant=torque_constant,
    gear_ratio=gear_ratio,
    efficiency=efficiency
)

wheel = WheelModel(
    wheel_radius=wheel_radius
)

robot = RobotDynamicModel(
    mass=mass,
    yaw_inertia=yaw_inertia,
    track_width=track_width
)

arm = ArmKinematics(
    link1_length=0.40,
    link2_length=0.36,
    wrist_length=0.12
)


# --------------------------------------------------
# Initial robot state
# [x, y, theta, v, omega]
# --------------------------------------------------

state = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
]


# --------------------------------------------------
# Motor commands
# Slightly asymmetric -> robot turns
# --------------------------------------------------

current_left = 2.0
current_right = 2.5


# Current -> wheel torque
torque_left = actuator.current_to_wheel_torque(
    current_left
)

torque_right = actuator.current_to_wheel_torque(
    current_right
)


# Wheel torque -> ground force
force_left = wheel.torque_to_force(
    torque_left
)

force_right = wheel.torque_to_force(
    torque_right
)


# --------------------------------------------------
# Simulate robot
# --------------------------------------------------

dt = 0.1
simulation_time = 2.0

steps = int(simulation_time / dt)

for i in range(steps):

    state = robot.step(
        state,
        force_left,
        force_right,
        dt
    )


# --------------------------------------------------
# Final robot pose
# --------------------------------------------------

robot_x = state[0]
robot_y = state[1]
robot_yaw = state[2]


# --------------------------------------------------
# Arm configuration
# --------------------------------------------------

q0 = math.radians(30)     # base yaw
q1 = math.radians(25)     # shoulder pitch
q2 = math.radians(-20)    # elbow pitch
q3 = math.radians(-15)    # wrist pitch


# Arm forward kinematics
arm_x, arm_y, arm_z, camera_pitch = arm.forward_kinematics(
    q0,
    q1,
    q2,
    q3
)


# --------------------------------------------------
# Camera pose in world
# --------------------------------------------------

(
    camera_x,
    camera_y,
    camera_z,
    camera_yaw,
    camera_pitch_world
) = camera_pose_in_world(
    robot_x,
    robot_y,
    robot_yaw,
    arm_x,
    arm_y,
    arm_z,
    q0,
    camera_pitch
)


# --------------------------------------------------
# Results
# --------------------------------------------------

print("========== STAGE 2 INTEGRATION TEST ==========")
print()

print("----- MOTOR / WHEEL -----")
print("left current =", current_left, "A")
print("right current =", current_right, "A")
print("left wheel torque =", torque_left, "N*m")
print("right wheel torque =", torque_right, "N*m")
print("left force =", force_left, "N")
print("right force =", force_right, "N")
print()

print("----- ROBOT FINAL STATE -----")
print("x =", robot_x, "m")
print("y =", robot_y, "m")
print("yaw =", math.degrees(robot_yaw), "deg")
print("v =", state[3], "m/s")
print("omega =", state[4], "rad/s")
print()

print("----- ARM CONFIGURATION -----")
print("base yaw =", math.degrees(q0), "deg")
print("shoulder =", math.degrees(q1), "deg")
print("elbow =", math.degrees(q2), "deg")
print("wrist =", math.degrees(q3), "deg")
print()

print("----- CAMERA WORLD POSE -----")
print("x =", camera_x, "m")
print("y =", camera_y, "m")
print("z =", camera_z, "m")
print("yaw =", math.degrees(camera_yaw), "deg")
print("pitch =", math.degrees(camera_pitch_world), "deg")