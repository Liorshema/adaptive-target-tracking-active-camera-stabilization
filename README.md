Adaptive Target Tracking & Active Camera Stabilization

ROS 2 / Gazebo project for adaptive 3D target tracking using a mobile manipulator with a wheeled base, robotic arm, and camera.

The system estimates the robot and target states, predicts future target motion using classical and learned models, and coordinates the mobile base and arm to keep the target smoothly framed inside the camera field of view.

Main Features
ROS 2 Jazzy + Gazebo simulation
Mobile base, robotic arm, and onboard camera
Robot and target state estimation
Classical target motion prediction
Sequential machine learning prediction
Adaptive classical-ML fusion
Lightweight online adaptation
Whole-body base-arm coordination
Camera tracking and stabilization
Joint, velocity, actuator, and field-of-view constraints
Reproducible tracking experiments and evaluation
System Architecture
Perception
    ↓
State Estimation
    ↓
Target Prediction
    ↓
Robot Models
    ↓
Whole-Body Control
    ↓
Low-Level Control
    ↓
Gazebo / Robot

The robot model combines the motion of the mobile base and robotic arm to determine the camera pose and velocity. The controller then coordinates both subsystems to follow the predicted target trajectory while respecting physical and visibility constraints.

Packages
robot_description   Robot geometry, URDF/Xacro, sensors and physical parameters
robot_simulation    Gazebo worlds, target motion and disturbances
robot_models        Kinematics, dynamics, geometry and constraints
robot_perception    Camera-based target measurements
robot_estimation    Robot and target state estimation
robot_prediction    Classical, learned and hybrid prediction
robot_learning      Offline ML training and evaluation
robot_control       Whole-body and low-level controllers
robot_evaluation    Tracking, prediction and control metrics
robot_experiments   Experiment and benchmark management
robot_interface     Interactive experiment interface
robot_bringup       System launch and configuration
Current Development

Current focus is the robot_models package, including:

common
mobile
arm
whole_body
camera
target
reference
task_space
constraints

The project is developed incrementally, beginning with a complete classical baseline and later adding learned prediction, adaptive fusion, online adaptation, and controller comparison.

Environment
Ubuntu 24.04
ROS 2 Jazzy
Gazebo
Python 3
NumPy
Goal

The main goal is to study how target-motion prediction and adaptation affect closed-loop visual tracking, and how coordinated base-arm control can use those predictions to maintain smooth and stable camera tracking.