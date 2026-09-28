"""Differential-drive base Jacobian utilities."""

import numpy as np


class DifferentialDriveJacobian:
    """Compute the base contribution to a spatial twist."""

    VELOCITY_DIMENSION = 2

    @staticmethod
    def compute(
        point_base: np.ndarray,
    ) -> np.ndarray:
        """
        Return the 6x2 Jacobian in the base frame.

        Generalized base velocity:

            nu_B = [v_B, omega_B]^T

        where:
            v_B     = forward linear velocity
            omega_B = yaw rate
        """
        point_base = np.asarray(
            point_base,
            dtype=float,
        )

        if point_base.shape != (3,):
            raise ValueError(
                'point_base must have shape (3,)'
            )

        forward_axis_base = np.array(
            [1.0, 0.0, 0.0]
        )

        yaw_axis_base = np.array(
            [0.0, 0.0, 1.0]
        )

        linear_from_translation = (
            forward_axis_base
        )

        linear_from_rotation = np.cross(
            yaw_axis_base,
            point_base,
        )

        angular_from_translation = (
            np.zeros(3)
        )

        angular_from_rotation = (
            yaw_axis_base
        )

        return np.column_stack(
            (
                np.concatenate(
                    (
                        linear_from_translation,
                        angular_from_translation,
                    )
                ),
                np.concatenate(
                    (
                        linear_from_rotation,
                        angular_from_rotation,
                    )
                ),
            )
        )
