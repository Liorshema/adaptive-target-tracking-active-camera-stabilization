import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class RobotProcessModel:
    """
    Process model for a differential-drive mobile manipulator.

    State
    -----
    x_R = [
        x_B,
        y_B,
        theta_B,
        v_B,
        omega_B,
        q,
        q_dot
    ]^T

    where

    q, q_dot in R^n

    and the complete state dimension is

        5 + 2n.
    """

    BASE_STATE_DIMENSION = 5

    def __init__(
        self,
        joint_count: int,
        linear_acceleration_noise_std: float = 1.0,
        angular_acceleration_noise_std: float = 1.0,
        joint_acceleration_noise_std: float = 1.0,
    ) -> None:
        if joint_count <= 0:
            raise ValueError(
                "joint_count must be positive."
            )

        if linear_acceleration_noise_std < 0.0:
            raise ValueError(
                "linear_acceleration_noise_std must be non-negative."
            )

        if angular_acceleration_noise_std < 0.0:
            raise ValueError(
                "angular_acceleration_noise_std must be non-negative."
            )

        if joint_acceleration_noise_std < 0.0:
            raise ValueError(
                "joint_acceleration_noise_std must be non-negative."
            )

        self._joint_count = joint_count

        self._linear_acceleration_noise_std = (
            linear_acceleration_noise_std
        )

        self._angular_acceleration_noise_std = (
            angular_acceleration_noise_std
        )

        self._joint_acceleration_noise_std = (
            joint_acceleration_noise_std
        )

    @property
    def joint_count(self) -> int:
        """Return the number of manipulator joints."""
        return self._joint_count

    @property
    def state_dimension(self) -> int:
        """Return the complete robot-state dimension."""
        return (
            self.BASE_STATE_DIMENSION
            + 2 * self._joint_count
        )

    def propagate(
        self,
        state: Vector,
        dt: float,
    ) -> Vector:
        """
        Propagate the robot state forward by dt.

        Base model
        ----------
        x_dot     = v cos(theta)
        y_dot     = v sin(theta)
        theta_dot = omega

        Joint model
        -----------
        q_dot is assumed constant over the prediction interval.
        """
        state = self._validate_state(state)
        self._validate_dt(dt)

        next_state = state.copy()

        theta = state[2]
        linear_velocity = state[3]
        angular_velocity = state[4]

        joint_position_start = self.BASE_STATE_DIMENSION
        joint_velocity_start = (
            joint_position_start + self._joint_count
        )

        joint_positions = state[
            joint_position_start:joint_velocity_start
        ]

        joint_velocities = state[
            joint_velocity_start:
        ]

        next_state[0] += (
            dt
            * linear_velocity
            * np.cos(theta)
        )

        next_state[1] += (
            dt
            * linear_velocity
            * np.sin(theta)
        )

        next_state[2] += (
            dt
            * angular_velocity
        )

        next_state[
            joint_position_start:joint_velocity_start
        ] = (
            joint_positions
            + dt * joint_velocities
        )

        return next_state

    def jacobian(
        self,
        state: Vector,
        dt: float,
    ) -> Matrix:
        """
        Return the Jacobian of the process model with respect to state.
        """
        state = self._validate_state(state)
        self._validate_dt(dt)

        F = np.eye(
            self.state_dimension
        )

        theta = state[2]
        linear_velocity = state[3]

        F[0, 2] = (
            -dt
            * linear_velocity
            * np.sin(theta)
        )

        F[0, 3] = (
            dt
            * np.cos(theta)
        )

        F[1, 2] = (
            dt
            * linear_velocity
            * np.cos(theta)
        )

        F[1, 3] = (
            dt
            * np.sin(theta)
        )

        F[2, 4] = dt

        joint_position_start = self.BASE_STATE_DIMENSION
        joint_velocity_start = (
            joint_position_start + self._joint_count
        )

        F[
            joint_position_start:joint_velocity_start,
            joint_velocity_start:
        ] = (
            dt * np.eye(self._joint_count)
        )

        return F

    def process_covariance(
        self,
        dt: float,
    ) -> Matrix:
        """
        Return the process-noise covariance Q(dt).

        Independent acceleration disturbances are assumed for:

        - base linear motion,
        - base angular motion,
        - manipulator joints.

        Noise parameters are provided externally through the
        constructor.
        """
        self._validate_dt(dt)

        n = self.state_dimension

        Q = np.zeros(
            (n, n)
        )

        sigma_v_squared = (
            self._linear_acceleration_noise_std ** 2
        )

        sigma_omega_squared = (
            self._angular_acceleration_noise_std ** 2
        )

        sigma_joint_squared = (
            self._joint_acceleration_noise_std ** 2
        )

        # Base linear velocity uncertainty.
        Q[3, 3] = (
            dt ** 2
            * sigma_v_squared
        )

        # Base angular velocity uncertainty.
        Q[4, 4] = (
            dt ** 2
            * sigma_omega_squared
        )

        joint_position_start = self.BASE_STATE_DIMENSION
        joint_velocity_start = (
            joint_position_start + self._joint_count
        )

        identity = np.eye(
            self._joint_count
        )

        q_pp = (
            dt ** 4 / 4.0
        ) * identity

        q_pv = (
            dt ** 3 / 2.0
        ) * identity

        q_vv = (
            dt ** 2
        ) * identity

        joint_covariance = (
            sigma_joint_squared
            * np.block([
                [q_pp, q_pv],
                [q_pv, q_vv],
            ])
        )

        Q[
            joint_position_start:,
            joint_position_start:
        ] = joint_covariance

        return Q

    def _validate_state(
        self,
        state: Vector,
    ) -> Vector:
        """Validate and return the robot state vector."""
        state = np.asarray(
            state,
            dtype=float,
        )

        if state.shape != (
            self.state_dimension,
        ):
            raise ValueError(
                "Robot state must have shape "
                f"({self.state_dimension},), "
                f"got {state.shape}."
            )

        return state

    @staticmethod
    def _validate_dt(
        dt: float,
    ) -> None:
        """Validate the prediction time step."""
        if dt <= 0.0:
            raise ValueError(
                "dt must be positive."
            )
        """
        Validate the prediction time step.
        """
        if dt <= 0.0:
            raise ValueError(
                "dt must be positive."
            )