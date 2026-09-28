"""Abstract interface for mobile-base models."""

from abc import ABC, abstractmethod

import numpy as np


class BaseModel(ABC):
    """Common interface for mobile-base kinematic models."""

    @property
    @abstractmethod
    def velocity_dimension(self) -> int:
        """Return the dimension of the base generalized velocity."""
    @abstractmethod
    def pose_derivative(
        self,
        pose: np.ndarray,
        generalized_velocity: np.ndarray,
    ) -> np.ndarray:
        """Return the base pose derivative."""
    @abstractmethod
    def spatial_jacobian(
        self,
        point_base: np.ndarray,
    ) -> np.ndarray:
        """
        Return the base Jacobian for a point fixed to the base.

        The Jacobian maps base generalized velocity to a 6D twist
        expressed in the base frame.
        """
