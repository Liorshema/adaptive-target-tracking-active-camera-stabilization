"""Whole-body Jacobian for the mobile manipulator."""

import numpy as np


class WholeBodyJacobian:
    """Combine mobile-base and arm Jacobians."""

    @staticmethod
    def compute(
        transform_world_base: np.ndarray,
        camera_position_base: np.ndarray,
        arm_jacobian_base: np.ndarray,
    ) -> np.ndarray:
        """Return the 6x(2+n) whole-body Jacobian in world frame.

        Generalized velocity:
            [v_B, omega_B, q_dot_1, ..., q_dot_n]

        The mobile base is assumed to be planar:
            - v_B: forward velocity along base x-axis
            - omega_B: yaw rate about base z-axis
        """

        transform_world_base = np.asarray(
            transform_world_base,
            dtype=float,
        )

        camera_position_base = np.asarray(
            camera_position_base,
            dtype=float,
        )

        arm_jacobian_base = np.asarray(
            arm_jacobian_base,
            dtype=float,
        )

        if transform_world_base.shape != (4, 4):
            raise ValueError(
                "transform_world_base must have shape (4, 4)"
            )

        if camera_position_base.shape != (3,):
            raise ValueError(
                "camera_position_base must have shape (3,)"
            )

        if arm_jacobian_base.ndim != 2:
            raise ValueError(
                "arm_jacobian_base must be a 2D matrix"
            )

        if arm_jacobian_base.shape[0] != 6:
            raise ValueError(
                "arm_jacobian_base must have 6 rows"
            )

        rotation_world_base = (
            transform_world_base[:3, :3]
        )

        forward_axis_world = (
            rotation_world_base
            @ np.array(
                [1.0, 0.0, 0.0],
                dtype=float,
            )
        )

        yaw_axis_world = (
            rotation_world_base
            @ np.array(
                [0.0, 0.0, 1.0],
                dtype=float,
            )
        )

        camera_position_world_relative = (
            rotation_world_base
            @ camera_position_base
        )

        base_linear_column = np.zeros(
            6,
            dtype=float,
        )

        base_linear_column[:3] = (
            forward_axis_world
        )

        base_angular_column = np.zeros(
            6,
            dtype=float,
        )

        base_angular_column[:3] = np.cross(
            yaw_axis_world,
            camera_position_world_relative,
        )

        base_angular_column[3:] = (
            yaw_axis_world
        )

        arm_jacobian_world = np.zeros_like(
            arm_jacobian_base
        )

        arm_jacobian_world[:3, :] = (
            rotation_world_base
            @ arm_jacobian_base[:3, :]
        )

        arm_jacobian_world[3:, :] = (
            rotation_world_base
            @ arm_jacobian_base[3:, :]
        )

        return np.column_stack(
            (
                base_linear_column,
                base_angular_column,
                arm_jacobian_world,
            )
        )