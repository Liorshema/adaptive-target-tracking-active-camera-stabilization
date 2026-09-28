"""Camera visibility constraint utilities."""


import numpy as np

from robot_models.camera.fov import CameraFOV
from robot_models.camera.geometry import CameraGeometry
from robot_models.camera.projection import CameraProjection


class VisibilityConstraint:
    """Represent target visibility constraints in the camera image."""

    def __init__(
        self,
        projection: CameraProjection,
        fov: CameraFOV,
    ) -> None:
        self.projection = projection
        self.fov = fov

    def pixel(
        self,
        transform_world_optical: np.ndarray,
        target_position_world: np.ndarray,
    ) -> np.ndarray:
        """Project target world position into image coordinates."""
        target_position_optical = (
            CameraGeometry.target_position_optical(
                transform_world_optical=transform_world_optical,
                target_position_world=target_position_world,
            )
        )

        return self.projection.project(
            target_position_optical
        )

    def residual(
        self,
        transform_world_optical: np.ndarray,
        target_position_world: np.ndarray,
    ) -> np.ndarray:
        """
        Return image-boundary visibility residuals.

        Positive values indicate satisfied constraints.
        """
        pixel = self.pixel(
            transform_world_optical,
            target_position_world,
        )

        u, v = pixel

        u_min, u_max, v_min, v_max = (
            self.fov.bounds
        )

        return np.array(
            [
                u - u_min,
                u_max - u,
                v - v_min,
                v_max - v,
            ],
            dtype=float,
        )

    def contains(
        self,
        transform_world_optical: np.ndarray,
        target_position_world: np.ndarray,
    ) -> bool:
        """Return True when target is visible in the image."""
        try:
            residual = self.residual(
                transform_world_optical,
                target_position_world,
            )
        except ValueError:
            return False

        return bool(
            np.all(residual >= 0.0)
        )
