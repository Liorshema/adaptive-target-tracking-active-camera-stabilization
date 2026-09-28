"""Differential-drive mobile-base kinematics."""

import numpy as np

from robot_models.base.differential_drive.jacobian import (
    DifferentialDriveJacobian,
)
from robot_models.interfaces.base_model import (
    BaseModel,
)


class DifferentialDriveKinematics(BaseModel):
    """Differential-drive base kinematic model."""

    def __init__(
        self,
        wheel_radius: float,
        track_width: float,
    ) -> None:
        if wheel_radius <= 0.0:
            raise ValueError(
                'wheel_radius must be positive'
            )

        if track_width <= 0.0:
            raise ValueError(
                'track_width must be positive'
            )

        self.wheel_radius = float(
            wheel_radius
        )

        self.track_width = float(
            track_width
        )

    @property
    def velocity_dimension(self) -> int:
        """Return base generalized velocity dimension."""
        return 2

    @property
    def wheel_to_body_matrix(
        self,
    ) -> np.ndarray:
        """Map wheel rates to [v_B, omega_B]."""
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
    def body_to_wheel_matrix(
        self,
    ) -> np.ndarray:
        """Map [v_B, omega_B] to wheel rates."""
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
        """Convert wheel angular rates to base velocity."""
        wheel_rates = np.asarray(
            wheel_rates,
            dtype=float,
        )

        if wheel_rates.shape != (2,):
            raise ValueError(
                'wheel_rates must have shape (2,)'
            )

        return (
            self.wheel_to_body_matrix
            @ wheel_rates
        )

    def body_velocity_to_wheel_rates(
        self,
        body_velocity: np.ndarray,
    ) -> np.ndarray:
        """Convert base velocity to wheel angular rates."""
        body_velocity = np.asarray(
            body_velocity,
            dtype=float,
        )

        if body_velocity.shape != (2,):
            raise ValueError(
                'body_velocity must have shape (2,)'
            )

        return (
            self.body_to_wheel_matrix
            @ body_velocity
        )

    def pose_derivative(
        self,
        pose: np.ndarray,
        generalized_velocity: np.ndarray,
    ) -> np.ndarray:
        """Return planar base pose derivative."""
        pose = np.asarray(
            pose,
            dtype=float,
        )

        generalized_velocity = np.asarray(
            generalized_velocity,
            dtype=float,
        )

        if pose.shape != (3,):
            raise ValueError(
                'pose must have shape (3,)'
            )

        if generalized_velocity.shape != (2,):
            raise ValueError(
                'generalized_velocity must have shape (2,)'
            )

        theta = pose[2]

        mapping = np.array(
            [
                [np.cos(theta), 0.0],
                [np.sin(theta), 0.0],
                [0.0, 1.0],
            ],
            dtype=float,
        )

        return (
            mapping
            @ generalized_velocity
        )

    def spatial_jacobian(
        self,
        point_base: np.ndarray,
    ) -> np.ndarray:
        """Return the 6x2 base Jacobian."""
        return DifferentialDriveJacobian.compute(
            point_base
        )
