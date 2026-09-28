"""Common interface for complete robot models."""

from abc import ABC, abstractmethod

from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel


class RobotModel(ABC):
    """Interface for a robot composed of a base and manipulator."""

    @property
    @abstractmethod
    def base(self) -> BaseModel:
        """Return the robot base model."""
    @property
    @abstractmethod
    def manipulator(self) -> ManipulatorModel:
        """Return the robot manipulator model."""
    @property
    def generalized_velocity_dimension(self) -> int:
        """Return the whole-body generalized velocity dimension."""
        return (
            self.base.velocity_dimension
            + self.manipulator.dof
        )
