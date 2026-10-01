import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class ConstantVelocityProcessModel:
    """
    Constant-velocity process model for a target moving in N dimensions.

    State
    -----
    x = [p, v]^T

    where

    p in R^d
        Position vector.

    v in R^d
        Velocity vector.

    The process noise is modeled as zero-mean white acceleration noise.
    """

    def __init__(
        self,
        spatial_dimension: int = 3,
        acceleration_noise_std: float = 1.0,
    ) -> None:
        if spatial_dimension <= 0:
            raise ValueError("spatial_dimension must be positive.")

        if acceleration_noise_std < 0.0:
            raise ValueError(
                "acceleration_noise_std must be non-negative."
            )

        self._spatial_dimension = spatial_dimension
        self._acceleration_noise_std = acceleration_noise_std

    @property
    def spatial_dimension(self) -> int:
        """Return the dimension of physical space."""
        return self._spatial_dimension

    @property
    def state_dimension(self) -> int:
        """Return the total state dimension."""
        return 2 * self._spatial_dimension

    @property
    def acceleration_noise_std(self) -> float:
        """Return the acceleration-noise standard deviation."""
        return self._acceleration_noise_std

    def state_transition_matrix(
        self,
        dt: float,
    ) -> Matrix:
        """
        Return the discrete constant-velocity state transition matrix.

        x_k+1 = A(dt) x_k
        """
        self._validate_dt(dt)

        d = self._spatial_dimension

        identity = np.eye(d)
        zeros = np.zeros((d, d))

        return np.block([
            [identity, dt * identity],
            [zeros, identity],
        ])

    def process_covariance(
        self,
        dt: float,
    ) -> Matrix:
        """
        Return the discrete process-noise covariance matrix.

        The model assumes white acceleration noise with standard
        deviation sigma_a.
        """
        self._validate_dt(dt)

        d = self._spatial_dimension
        identity = np.eye(d)

        sigma_a_squared = self._acceleration_noise_std ** 2

        q_pp = (dt ** 4 / 4.0) * identity
        q_pv = (dt ** 3 / 2.0) * identity
        q_vv = (dt ** 2) * identity

        return sigma_a_squared * np.block([
            [q_pp, q_pv],
            [q_pv, q_vv],
        ])

    def propagate(
        self,
        state: Vector,
        dt: float,
    ) -> Vector:
        """
        Propagate the state forward by dt.
        """
        state = self._validate_state(state)

        A = self.state_transition_matrix(dt)

        return A @ state

    def jacobian(
        self,
        state: Vector,
        dt: float,
    ) -> Matrix:
        """
        Return the process Jacobian.

        For the constant-velocity model, the process model is linear,
        so the Jacobian equals the state transition matrix.
        """
        self._validate_state(state)

        return self.state_transition_matrix(dt)

    def _validate_state(
        self,
        state: Vector,
    ) -> Vector:
        """
        Validate and return the state as a NumPy vector.
        """
        state = np.asarray(state, dtype=float)

        if state.shape != (self.state_dimension,):
            raise ValueError(
                f"State must have shape ({self.state_dimension},), "
                f"got {state.shape}."
            )

        return state

    @staticmethod
    def _validate_dt(dt: float) -> None:
        """
        Validate the prediction time step.
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive.")