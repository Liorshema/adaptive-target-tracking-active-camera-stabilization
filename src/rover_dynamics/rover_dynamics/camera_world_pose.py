import math


def camera_pose_in_world(
    rover_x,
    rover_y,
    rover_yaw,
    arm_x,
    arm_y,
    arm_z,
    arm_base_yaw,
    camera_pitch,
    rover_base_height=0.0
):
    """
    Transform camera pose from rover frame
    into world frame.

    Returns:
    camera_x_world
    camera_y_world
    camera_z_world
    camera_yaw_world
    camera_pitch_world
    """

    camera_x_world = (
        rover_x
        + math.cos(rover_yaw) * arm_x
        - math.sin(rover_yaw) * arm_y
    )

    camera_y_world = (
        rover_y
        + math.sin(rover_yaw) * arm_x
        + math.cos(rover_yaw) * arm_y
    )

    camera_z_world = (
        rover_base_height
        + arm_z
    )

    camera_yaw_world = (
        rover_yaw
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