"""Desired camera-pose construction utilities."""

import numpy as np

from robot_models.common.transforms import make_transform


class PoseReference:
    """Build desired camera pose references."""

    @staticmethod
    def compute(
        desired_position_world: np.ndarray,
        desired_rotation_world_camera: np.ndarray,
    ) -> np.ndarray:
        """Return desired camera transform T_W_E_d."""

        desired_position_world = np.asarray(
            desired_position_world,
            dtype=float,
        )

        desired_rotation_world_camera = np.asarray(
            desired_rotation_world_camera,
            dtype=float,
        )

        if desired_position_world.shape != (3,):
            raise ValueError(
                "desired_position_world must have shape (3,)"
            )

        if desired_rotation_world_camera.shape != (3, 3):
            raise ValueError(
                "desired_rotation_world_camera must have shape (3, 3)"
            )

        return make_transform(
            rotation=desired_rotation_world_camera,
            position=desired_position_world,
        )