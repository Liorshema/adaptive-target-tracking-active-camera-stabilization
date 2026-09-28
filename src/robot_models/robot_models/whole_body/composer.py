"""Whole-body robot model composition."""

from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.interfaces.robot_model import RobotModel


class WholeBodyComposer(RobotModel):
    """Compose a base model and manipulator model into one robot model."""

    def __init__(
        self,
        base: BaseModel,
        manipulator: ManipulatorModel,
    ) -> None:
        self._base = base
        self._manipulator = manipulator

    @property
    def base(self) -> BaseModel:
        """Return the base model."""
        return self._base

    @property
    def manipulator(self) -> ManipulatorModel:
        """Return the manipulator model."""
        return self._manipulator
