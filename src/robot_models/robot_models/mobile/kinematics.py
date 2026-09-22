"""Kinematic model for a differential-drive mobile base."""

import numpy as np


class MobileKinematics:
    """Differential-drive kinematics using vector/matrix representations."""

    def __init__(
        self,
        wheel_radius: float,
        track_width: float,
    ) -> None:
        if wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be positive")

        if track_width <= 0.0:
            raise ValueError("track_width must be positive")

        self.wheel_radius = float(wheel_radius)
        self.track_width = float(track_width)

    @property
    def wheel_to_body_matrix(self) -> np.ndarray:
        """Return mapping from wheel rates to planar body velocity."""

        r = self.wheel_radius
        L = self.track_width

        return np.array(
            [
                [r / 2.0, r / 2.0],
                [-r / L, r / L],
            ],
            dtype=float,
        )

    @property
    def body_to_wheel_matrix(self) -> np.ndarray:
        """Return mapping from planar body velocity to wheel rates."""

        r = self.wheel_radius
        L = self.track_width

        return np.array(
            [
                [1.0 / r, -L / (2.0 * r)],
                [1.0 / r, L / (2.0 * r)],
            ],
            dtype=float,
        )

    def wheel_rates_to_body_velocity(
        self,
        wheel_rates: np.ndarray,
    ) -> np.ndarray:
        """Map [omega_left, omega_right] to [v, omega]."""

        wheel_rates = np.asarray(wheel_rates, dtype=float)

        if wheel_rates.shape != (2,):
            raise ValueError("wheel_rates must have shape (2,)")

        return self.wheel_to_body_matrix @ wheel_rates

    def body_velocity_to_wheel_rates(
        self,
        body_velocity: np.ndarray,
    ) -> np.ndarray:
        """Map [v, omega] to [omega_left, omega_right]."""

        body_velocity = np.asarray(body_velocity, dtype=float)

        if body_velocity.shape != (2,):
            raise ValueError("body_velocity must have shape (2,)")

        return self.body_to_wheel_matrix @ body_velocity

    @staticmethod
    def planar_pose_derivative(
        pose: np.ndarray,
        body_velocity: np.ndarray,
    ) -> np.ndarray:
        """Compute planar pose derivative from body velocity."""

        pose = np.asarray(pose, dtype=float)
        body_velocity = np.asarray(body_velocity, dtype=float)

        if pose.shape != (3,):
            raise ValueError("pose must have shape (3,)")

        if body_velocity.shape != (2,):
            raise ValueError("body_velocity must have shape (2,)")

        theta = pose[2]

        body_to_world = np.array(
            [
                [np.cos(theta), 0.0],
                [np.sin(theta), 0.0],
                [0.0, 1.0],
            ],
            dtype=float,
        )

        return body_to_world @ body_velocity