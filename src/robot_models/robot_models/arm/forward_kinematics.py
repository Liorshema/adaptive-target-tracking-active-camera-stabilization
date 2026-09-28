"""Forward kinematics for the robotic arm."""

import numpy as np

from robot_models.common.rotations import (
    axis_angle_rotation,
    rotation_x,
    rotation_z,
)
from robot_models.common.transforms import (
    make_transform,
)


class ArmForwardKinematics:
    """Compute the camera optical-frame pose in the robot base frame."""

    DOF = 4

    def __init__(
        self,
        joint_axes: np.ndarray,
        body_height: float,
        arm_base_height: float,
        link_1_length: float,
        link_2_length: float,
        wrist_length: float,
    ) -> None:
        joint_axes = np.asarray(
            joint_axes,
            dtype=float,
        )

        if joint_axes.shape != (self.DOF, 3):
            raise ValueError(
                f"joint_axes must have shape ({self.DOF}, 3)"
            )

        axis_norms = np.linalg.norm(
            joint_axes,
            axis=1,
        )

        if np.any(axis_norms == 0.0):
            raise ValueError(
                "joint axes must be non-zero"
            )

        self.joint_axes = (
            joint_axes
            / axis_norms[:, None]
        )

        geometry = np.array(
            [
                body_height,
                arm_base_height,
                link_1_length,
                link_2_length,
                wrist_length,
            ],
            dtype=float,
        )

        if np.any(geometry <= 0.0):
            raise ValueError(
                "all geometry parameters must be positive"
            )

        self.body_height = float(body_height)
        self.arm_base_height = float(arm_base_height)
        self.link_1_length = float(link_1_length)
        self.link_2_length = float(link_2_length)
        self.wrist_length = float(wrist_length)

    def compute(
        self,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """Return T_B_O for the camera optical frame.

        B:
            Robot base frame.

        O:
            Camera optical frame.
        """

        joint_positions = np.asarray(
            joint_positions,
            dtype=float,
        )

        if joint_positions.shape != (self.DOF,):
            raise ValueError(
                f"joint_positions must have shape ({self.DOF},)"
            )

        base_mount_height = (
            self.body_height / 2.0
            + self.arm_base_height / 2.0
        )

        shoulder_height = (
            self.arm_base_height / 2.0
        )

        joint_offsets = np.array(
            [
                [0.0, 0.0, base_mount_height],
                [0.0, 0.0, shoulder_height],
                [self.link_1_length, 0.0, 0.0],
                [self.link_2_length, 0.0, 0.0],
            ],
            dtype=float,
        )

        transform_base_current = np.eye(4)

        for axis, angle, offset in zip(
            self.joint_axes,
            joint_positions,
            joint_offsets,
        ):
            joint_transform = make_transform(
                rotation=axis_angle_rotation(
                    axis,
                    angle,
                ),
                position=offset,
            )

            transform_base_current = (
                transform_base_current
                @ joint_transform
            )

        # wrist_link -> camera_link
        transform_wrist_camera = make_transform(
            rotation=np.eye(3),
            position=np.array(
                [
                    self.wrist_length,
                    0.0,
                    0.0,
                ]
            ),
        )

        transform_base_camera = (
            transform_base_current
            @ transform_wrist_camera
        )

        # camera_link -> camera_optical_frame
        #
        # Matches URDF:
        # rpy="-pi/2 0 -pi/2"
        #
        # URDF RPY convention:
        # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        transform_camera_optical = make_transform(
            rotation=(
                rotation_z(-np.pi / 2.0)
                @ rotation_x(-np.pi / 2.0)
            ),
            position=np.zeros(3),
        )

        return (
            transform_base_camera
            @ transform_camera_optical
        )