import math


def camera_pose_in_world(
    robot_x,
    robot_y,
    robot_yaw,
    arm_x,
    arm_y,
    arm_z,
    arm_base_yaw,
    camera_pitch,
    robot_base_height=0.0
):
    """
    Transform camera pose from robot frame
    into world frame.

    Returns:
    camera_x_world
    camera_y_world
    camera_z_world
    camera_yaw_world
    camera_pitch_world
    """

    camera_x_world = (
        robot_x
        + math.cos(robot_yaw) * arm_x
        - math.sin(robot_yaw) * arm_y
    )

    camera_y_world = (
        robot_y
        + math.sin(robot_yaw) * arm_x
        + math.cos(robot_yaw) * arm_y
    )

    camera_z_world = (
        robot_base_height
        + arm_z
    )

    camera_yaw_world = (
        robot_yaw
        + arm_base_yaw
    )

    camera_pitch_world = camera_pitch

    return (
        camera_x_world,
        camera_y_world,
        camera_z_world,
        camera_yaw_world,
        camera_pitch_world
    )