"""Dynamic state model for a planar differential-drive mobile base."""

import numpy as np


class MobileDynamics:
    """Continuous-time state-space model for the mobile base."""

    STATE_DIM = 5
    INPUT_DIM = 2

    @staticmethod
    def state_derivative(
        state: np.ndarray,
        control: np.ndarray,
    ) -> np.ndarray:
        """Compute x_dot = f(x, u).

        State:
            [x, y, theta, v, omega]

        Control:
            [a, alpha]
        """

        state = np.asarray(state, dtype=float)
        control = np.asarray(control, dtype=float)

        if state.shape != (MobileDynamics.STATE_DIM,):
            raise ValueError("state must have shape (5,)")

        if control.shape != (MobileDynamics.INPUT_DIM,):
            raise ValueError("control must have shape (2,)")

        theta = state[2]
        v = state[3]
        omega = state[4]

        a = control[0]
        alpha = control[1]

        kinematic_matrix = np.array(
            [
                [np.cos(theta), 0.0],
                [np.sin(theta), 0.0],
                [0.0, 1.0],
            ],
            dtype=float,
        )

        generalized_velocity = np.array(
            [v, omega],
            dtype=float,
        )

        pose_derivative = (
            kinematic_matrix @ generalized_velocity
        )

        velocity_derivative = np.array(
            [a, alpha],
            dtype=float,
        )

        return np.concatenate(
            (
                pose_derivative,
                velocity_derivative,
            )
        )