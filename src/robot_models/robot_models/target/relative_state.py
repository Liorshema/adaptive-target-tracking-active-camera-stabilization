"""Relative target-state utilities."""

import numpy as np


class RelativeTargetState:
    """Compute target state relative to the end effector."""

    @staticmethod
    def compute(
        target_position_world: np.ndarray,
        target_velocity_world: np.ndarray,
        end_effector_position_world: np.ndarray,
        end_effector_velocity_world: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute relative target position and velocity.

        All vectors must be expressed in the same frame.
        """
        target_position_world = np.asarray(
            target_position_world,
            dtype=float,
        )
        target_velocity_world = np.asarray(
            target_velocity_world,
            dtype=float,
        )
        end_effector_position_world = np.asarray(
            end_effector_position_world,
            dtype=float,
        )
        end_effector_velocity_world = np.asarray(
            end_effector_velocity_world,
            dtype=float,
        )

        vectors = (
            target_position_world,
            target_velocity_world,
            end_effector_position_world,
            end_effector_velocity_world,
        )

        if any(vector.shape != (3,) for vector in vectors):
            raise ValueError(
                'All position and velocity vectors must have shape (3,).'
            )

        relative_position_world = (
            target_position_world
            - end_effector_position_world
        )

        relative_velocity_world = (
            target_velocity_world
            - end_effector_velocity_world
        )

        return relative_position_world, relative_velocity_world
