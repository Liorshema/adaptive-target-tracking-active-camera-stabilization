import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class PositionMeasurementModel:
    """
    Position-only measurement model for a target state.

    State
    -----
    x = [p, v]^T

    Measurement
    -----------
    z = p

    where p in R^d.
    """

    def __init__(self, spatial_dimension: int = 3) -> None:
        if spatial_dimension <= 0:
            raise ValueError("spatial_dimension must be positive.")

        self._spatial_dimension = spatial_dimension

    @property
    def spatial_dimension(self) -> int:
        """Return the measurement-space dimension."""
        return self._spatial_dimension

    @property
    def state_dimension(self) -> int:
        """Return the expected state dimension."""
        return 2 * self._spatial_dimension

    @property
    def measurement_dimension(self) -> int:
        """Return the measurement dimension."""
        return self._spatial_dimension

    def measurement_matrix(self) -> Matrix:
        """
        Return the linear measurement matrix H.

        z = H x
        """
        d = self._spatial_dimension

        identity = np.eye(d)
        zeros = np.zeros((d, d))

        return np.hstack((identity, zeros))

    def predict_measurement(
        self,
        state: Vector,
    ) -> Vector:
        """
        Predict the measurement corresponding to the given state.
        """
        state = np.asarray(state, dtype=float)

        if state.shape != (self.state_dimension,):
            raise ValueError(
                f"State must have shape ({self.state_dimension},), "
                f"got {state.shape}."
            )

        return self.measurement_matrix() @ state

    def jacobian(
        self,
        state: Vector,
    ) -> Matrix:
        """
        Return the measurement Jacobian.

        Since the measurement model is linear, the Jacobian is equal
        to the measurement matrix H.
        """
        state = np.asarray(state, dtype=float)

        if state.shape != (self.state_dimension,):
            raise ValueError(
                f"State must have shape ({self.state_dimension},), "
                f"got {state.shape}."
            )

        return self.measurement_matrix()