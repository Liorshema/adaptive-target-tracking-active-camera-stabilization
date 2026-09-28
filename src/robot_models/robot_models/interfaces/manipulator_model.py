"""Abstract interface for manipulator models."""

from abc import ABC, abstractmethod

import numpy as np


class ManipulatorModel(ABC):
    """Common interface for manipulator kinematic models."""

    @property
    @abstractmethod
    def dof(self) -> int:
        """Return the number of manipulator degrees of freedom."""
    @abstractmethod
    def forward_kinematics(
        self,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """Return end-effector transform relative to the base."""
    @abstractmethod
    def jacobian(
        self,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """Return the 6 x n manipulator Jacobian in the base frame."""
