"""Whole-body Jacobian composition."""

import numpy as np

from robot_models.interfaces.robot_model import RobotModel


class WholeBodyJacobian:
    """Compose base and manipulator Jacobians."""

    @staticmethod
    def compute(
        robot_model: RobotModel,
        transform_world_base: np.ndarray,
        joint_positions: np.ndarray,
    ) -> np.ndarray:
        """
        Return the whole-body Jacobian expressed in the world frame.

        The generalized velocity is

            nu = [nu_B, q_dot]^T

        and

            V_E = J_WB @ nu.
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

        rotation_world_base = (
            transform_world_base[:3, :3]
        )

        transform_base_end_effector = (
            robot_model.manipulator.forward_kinematics(
                joint_positions
            )
        )

        end_effector_position_base = (
            transform_base_end_effector[:3, 3]
        )

        base_jacobian_base = (
            robot_model.base.spatial_jacobian(
                end_effector_position_base
            )
        )

        manipulator_jacobian_base = (
            robot_model.manipulator.jacobian(
                joint_positions
            )
        )

        whole_body_jacobian_base = np.hstack(
            (
                base_jacobian_base,
                manipulator_jacobian_base,
            )
        )

        rotation_spatial_world_base = np.block(
            [
                [
                    rotation_world_base,
                    np.zeros((3, 3)),
                ],
                [
                    np.zeros((3, 3)),
                    rotation_world_base,
                ],
            ]
        )

        return (
            rotation_spatial_world_base
            @ whole_body_jacobian_base
        )
