"""Pinhole camera projection utilities."""

import numpy as np


class CameraProjection:
    """Project 3D points from the optical frame to image coordinates."""

    def __init__(
        self,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> None:
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError(
                "fx and fy must be positive"
            )

        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)

    @property
    def intrinsic_matrix(self) -> np.ndarray:
        """Return the 3x3 intrinsic camera matrix."""

        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def project(
        self,
        point_optical: np.ndarray,
    ) -> np.ndarray:
        """Project a 3D point to image coordinates [u, v].

        The input point must be expressed in the camera optical frame:
            x -> right
            y -> down
            z -> forward
        """

        point_optical = np.asarray(
            point_optical,
            dtype=float,
        )

        if point_optical.shape != (3,):
            raise ValueError(
                "point_optical must have shape (3,)"
            )

        x, y, z = point_optical

        if z <= 0.0:
            raise ValueError(
                "point must lie in front of the camera (z > 0)"
            )

        normalized_point = np.array(
            [
                x / z,
                y / z,
                1.0,
            ],
            dtype=float,
        )

        pixel_homogeneous = (
            self.intrinsic_matrix
            @ normalized_point
        )

        return pixel_homogeneous[:2]