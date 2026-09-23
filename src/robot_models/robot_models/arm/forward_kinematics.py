"""Forward kinematics for the 4-DOF camera arm."""

import numpy as np

from robot_models.common.rotations import axis_angle_rotation
from robot_models.common.transforms import make_transform


class ArmForwardKinematics:
    """Forward kinematics for the camera arm."""

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
        joint_axes = np.asarray(joint_axes, dtype=float)

        if joint_axes.shape != (self.DOF, 3):
            raise ValueError(
                "joint_axes must have shape (4, 3)"
            )

        axis_norms = np.linalg.norm(
            joint_axes,
            axis=1,
        )

        if np.any(np.isclose(axis_norms, 0.0)):
            raise ValueError(
                "joint axes must be non-zero"
            )

        geometric_parameters = np.array(
            [
                body_height,
                arm_base_height,
                link_1_length,
                link_2_length,
                wrist_length,
            ],
            dtype=float,
        )

        if np.any(geometric_parameters <= 0.0):
            raise ValueError(
                "all geometric parameters must be positive"
            )

        self.joint_axes = (
            joint_axes
            / axis_norms[:, None]
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
        """Compute the camera transform T_B_E."""

        joint_positions = np.asarray(
            joint_positions,
            dtype=float,
        )

        if joint_positions.shape != (self.DOF,):
            raise ValueError(
                "joint_positions must have shape (4,)"
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

        transform = np.eye(4, dtype=float)

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

            transform = (
                transform @ joint_transform
            )

        camera_transform = make_transform(
            rotation=np.eye(3),
            position=np.array(
                [
                    self.wrist_length,
                    0.0,
                    0.0,
                ],
                dtype=float,
            ),
        )

        return transform @ camera_transform