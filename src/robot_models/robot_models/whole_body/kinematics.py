"""Whole-body kinematics for the mobile manipulator."""

import numpy as np


class WholeBodyKinematics:
    """Combine mobile-base and arm kinematics."""

    @staticmethod
    def camera_transform_world(
        transform_world_base: np.ndarray,
        transform_base_camera: np.ndarray,
    ) -> np.ndarray:
        """Compute the camera pose in the world frame.

        T_W_E = T_W_B @ T_B_E
        """

        transform_world_base = np.asarray(
            transform_world_base,
            dtype=float,
        )

        transform_base_camera = np.asarray(
            transform_base_camera,
            dtype=float,
        )

        if transform_world_base.shape != (4, 4):
            raise ValueError(
                "transform_world_base must have shape (4, 4)"
            )

        if transform_base_camera.shape != (4, 4):
            raise ValueError(
                "transform_base_camera must have shape (4, 4)"
            )

        return (
            transform_world_base
            @ transform_base_camera
        )