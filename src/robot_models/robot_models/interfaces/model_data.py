"""Controller-facing aggregation of robot model quantities."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class ModelData:
    """Controller-ready mathematical model data."""

    camera_transform_world: np.ndarray
    camera_twist_world: np.ndarray

    desired_camera_transform_world: np.ndarray
    desired_camera_twist_world: np.ndarray

    whole_body_jacobian_world: np.ndarray

    joint_constraint_residual: Optional[np.ndarray] = None
    velocity_constraint_residual: Optional[np.ndarray] = None
    acceleration_constraint_residual: Optional[np.ndarray] = None
    actuator_constraint_residual: Optional[np.ndarray] = None
    visibility_constraint_residual: Optional[np.ndarray] = None

    def __post_init__(self) -> None:
        self.camera_transform_world = np.asarray(
            self.camera_transform_world,
            dtype=float,
        )

        self.camera_twist_world = np.asarray(
            self.camera_twist_world,
            dtype=float,
        )

        self.desired_camera_transform_world = np.asarray(
            self.desired_camera_transform_world,
            dtype=float,
        )

        self.desired_camera_twist_world = np.asarray(
            self.desired_camera_twist_world,
            dtype=float,
        )

        self.whole_body_jacobian_world = np.asarray(
            self.whole_body_jacobian_world,
            dtype=float,
        )

        if self.camera_transform_world.shape != (4, 4):
            raise ValueError(
                "camera_transform_world must have shape (4, 4)"
            )

        if self.camera_twist_world.shape != (6,):
            raise ValueError(
                "camera_twist_world must have shape (6,)"
            )

        if self.desired_camera_transform_world.shape != (4, 4):
            raise ValueError(
                "desired_camera_transform_world must have shape (4, 4)"
            )

        if self.desired_camera_twist_world.shape != (6,):
            raise ValueError(
                "desired_camera_twist_world must have shape (6,)"
            )

        if self.whole_body_jacobian_world.ndim != 2:
            raise ValueError(
                "whole_body_jacobian_world must be a 2D matrix"
            )

        if self.whole_body_jacobian_world.shape[0] != 6:
            raise ValueError(
                "whole_body_jacobian_world must have 6 rows"
            )

        optional_vectors = {
            "joint_constraint_residual":
                self.joint_constraint_residual,
            "velocity_constraint_residual":
                self.velocity_constraint_residual,
            "acceleration_constraint_residual":
                self.acceleration_constraint_residual,
            "actuator_constraint_residual":
                self.actuator_constraint_residual,
            "visibility_constraint_residual":
                self.visibility_constraint_residual,
        }

        for name, vector in optional_vectors.items():
            if vector is None:
                continue

            vector = np.asarray(
                vector,
                dtype=float,
            )

            if vector.ndim != 1:
                raise ValueError(
                    f"{name} must be a 1D vector"
                )

            setattr(
                self,
                name,
                vector,
            )