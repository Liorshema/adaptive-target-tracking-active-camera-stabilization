"""Rigid-body dynamics utilities for the robotic arm."""

import numpy as np


class ArmDynamics:
    """Forward dynamics for an n-DOF manipulator."""

    @staticmethod
    def forward_dynamics(
        mass_matrix: np.ndarray,
        coriolis_matrix: np.ndarray,
        gravity_vector: np.ndarray,
        joint_velocities: np.ndarray,
        joint_torques: np.ndarray,
    ) -> np.ndarray:
        """Compute joint accelerations from manipulator dynamics.

        M(q) q_ddot + C(q, q_dot) q_dot + g(q) = tau
        """

        mass_matrix = np.asarray(
            mass_matrix,
            dtype=float,
        )

        coriolis_matrix = np.asarray(
            coriolis_matrix,
            dtype=float,
        )

        gravity_vector = np.asarray(
            gravity_vector,
            dtype=float,
        )

        joint_velocities = np.asarray(
            joint_velocities,
            dtype=float,
        )

        joint_torques = np.asarray(
            joint_torques,
            dtype=float,
        )

        if mass_matrix.ndim != 2:
            raise ValueError(
                "mass_matrix must be two-dimensional"
            )

        n = mass_matrix.shape[0]

        if mass_matrix.shape != (n, n):
            raise ValueError(
                "mass_matrix must be square"
            )

        if coriolis_matrix.shape != (n, n):
            raise ValueError(
                "coriolis_matrix must match mass_matrix"
            )

        for name, vector in (
            ("gravity_vector", gravity_vector),
            ("joint_velocities", joint_velocities),
            ("joint_torques", joint_torques),
        ):
            if vector.shape != (n,):
                raise ValueError(
                    f"{name} must have shape ({n},)"
                )

        generalized_force = (
            joint_torques
            - coriolis_matrix @ joint_velocities
            - gravity_vector
        )

        return np.linalg.solve(
            mass_matrix,
            generalized_force,
        )