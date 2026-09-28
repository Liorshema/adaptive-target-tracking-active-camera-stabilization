"""Whole-body forward kinematics."""

import numpy as np

from robot_models.interfaces.robot_model import RobotModel


class WholeBodyKinematics:
    """Compute end-effector pose for a composed robot model."""

    @staticmethod
    def compute(
        robot_model: RobotModel,
        transform_world_base: np.ndarray,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """
        Return the end-effector transform in the world frame.

        T_W_E = T_W_B @ T_B_E(q)
        """
        transform_world_base = np.asarray(
            transform_world_base,
            dtype=float,
        )

        joint_positions = np.asarray(
            joint_positions,
            dtype=float,
        )

        if transform_world_base.shape != (4, 4):
            raise ValueError(
                'transform_world_base must have shape (4, 4)'
            )

        if joint_positions.shape != (
            robot_model.manipulator.dof,
        ):
            raise ValueError(
                'joint_positions must match manipulator DOF'
            )

        transform_base_end_effector = (
            robot_model.manipulator.forward_kinematics(
                joint_positions
            )
        )

        return (
            transform_world_base
            @ transform_base_end_effector
        )
