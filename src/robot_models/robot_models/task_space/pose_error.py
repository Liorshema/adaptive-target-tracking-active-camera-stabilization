"""Task-space pose error utilities."""

import numpy as np

from robot_models.common.rotations import (
    rotation_log_vector,
)


class PoseError:
    """Compute camera pose error in the world frame."""

    @staticmethod
    def compute(
        transform_world_camera: np.ndarray,
        transform_world_camera_desired: np.ndarray,
    ) -> np.ndarray:
        """Return 6D pose error [position_error, rotation_error]."""
        transform_world_camera = np.asarray(
            transform_world_camera,
            dtype=float,
        )

        transform_world_camera_desired = np.asarray(
            transform_world_camera_desired,
            dtype=float,
        )

        if transform_world_camera.shape != (4, 4):
            raise ValueError(
                'transform_world_camera must have shape (4, 4)'
            )

        if transform_world_camera_desired.shape != (4, 4):
            raise ValueError(
                'transform_world_camera_desired must have shape (4, 4)'
            )

        position_world_camera = (
            transform_world_camera[:3, 3]
        )

        position_world_camera_desired = (
            transform_world_camera_desired[:3, 3]
        )

        position_error_world = (
            position_world_camera_desired
            - position_world_camera
        )

        rotation_world_camera = (
            transform_world_camera[:3, :3]
        )

        rotation_world_camera_desired = (
            transform_world_camera_desired[:3, :3]
        )

        rotation_camera_desired = (
            rotation_world_camera.T
            @ rotation_world_camera_desired
        )

        rotation_error_camera = (
            rotation_log_vector(
                rotation_camera_desired
            )
        )

        rotation_error_world = (
            rotation_world_camera
            @ rotation_error_camera
        )

        return np.concatenate(
            (
                position_error_world,
                rotation_error_world,
            )
        )
