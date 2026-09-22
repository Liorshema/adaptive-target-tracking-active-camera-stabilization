"""Shared data types for the robot mathematical model layer."""

from dataclasses import dataclass

import numpy as np


@dataclass
class Pose:
    """Rigid-body pose represented by position and rotation."""

    position: np.ndarray
    rotation: np.ndarray

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)
        self.rotation = np.asarray(self.rotation, dtype=float)

        if self.position.shape != (3,):
            raise ValueError("position must have shape (3,)")

        if self.rotation.shape != (3, 3):
            raise ValueError("rotation must have shape (3, 3)")


@dataclass
class Twist:
    """Spatial velocity represented by linear and angular velocity."""

    linear: np.ndarray
    angular: np.ndarray

    def __post_init__(self) -> None:
        self.linear = np.asarray(self.linear, dtype=float)
        self.angular = np.asarray(self.angular, dtype=float)

        if self.linear.shape != (3,):
            raise ValueError("linear must have shape (3,)")

        if self.angular.shape != (3,):
            raise ValueError("angular must have shape (3,)")

    @property
    def vector(self) -> np.ndarray:
        """Return the 6D twist vector [v, omega]."""

        return np.concatenate(
            (
                self.linear,
                self.angular,
            )
        )


@dataclass
class RobotState:
    """Estimated state of the mobile manipulator."""

    base_pose: Pose
    base_twist: Twist
    joint_positions: np.ndarray
    joint_velocities: np.ndarray

    def __post_init__(self) -> None:
        self.joint_positions = np.asarray(
            self.joint_positions,
            dtype=float,
        )
        self.joint_velocities = np.asarray(
            self.joint_velocities,
            dtype=float,
        )

        if self.joint_positions.ndim != 1:
            raise ValueError(
                "joint_positions must be a 1D vector"
            )

        if self.joint_velocities.shape != self.joint_positions.shape:
            raise ValueError(
                "joint_velocities must match joint_positions shape"
            )


@dataclass
class TargetState:
    """Estimated target state in Cartesian space."""

    position: np.ndarray
    velocity: np.ndarray

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)

        if self.position.shape != (3,):
            raise ValueError("position must have shape (3,)")

        if self.velocity.shape != (3,):
            raise ValueError("velocity must have shape (3,)")


@dataclass
class ConstraintSet:
    """Linear inequality constraint representation."""

    matrix: np.ndarray
    lower: np.ndarray
    upper: np.ndarray

    def __post_init__(self) -> None:
        self.matrix = np.asarray(self.matrix, dtype=float)
        self.lower = np.asarray(self.lower, dtype=float)
        self.upper = np.asarray(self.upper, dtype=float)

        if self.matrix.ndim != 2:
            raise ValueError("matrix must be two-dimensional")

        if self.lower.ndim != 1 or self.upper.ndim != 1:
            raise ValueError(
                "lower and upper must be 1D vectors"
            )

        if self.matrix.shape[0] != self.lower.size:
            raise ValueError(
                "matrix row count must match lower size"
            )

        if self.lower.shape != self.upper.shape:
            raise ValueError(
                "lower and upper must have the same shape"
            )