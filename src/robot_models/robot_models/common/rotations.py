"""Rotation utilities for SO(3)."""

import numpy as np


def skew(vector: np.ndarray) -> np.ndarray:
    """Return the skew-symmetric matrix of a 3D vector."""

    vector = np.asarray(vector, dtype=float)

    if vector.shape != (3,):
        raise ValueError("vector must have shape (3,)")

    x, y, z = vector

    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def axis_angle_rotation(
    axis: np.ndarray,
    angle: float,
) -> np.ndarray:
    """Return a rotation matrix from axis-angle using Rodrigues' formula."""

    axis = np.asarray(axis, dtype=float)

    if axis.shape != (3,):
        raise ValueError("axis must have shape (3,)")

    norm = np.linalg.norm(axis)

    if np.isclose(norm, 0.0):
        raise ValueError("axis must be non-zero")

    axis = axis / norm

    axis_skew = skew(axis)

    return (
        np.eye(3)
        + np.sin(angle) * axis_skew
        + (1.0 - np.cos(angle))
        * (axis_skew @ axis_skew)
    )


def is_rotation_matrix(
    rotation: np.ndarray,
    atol: float = 1e-8,
) -> bool:
    """Check whether a matrix is a valid rotation matrix."""

    rotation = np.asarray(rotation, dtype=float)

    if rotation.shape != (3, 3):
        return False

    orthogonality = rotation.T @ rotation
    determinant = np.linalg.det(rotation)

    return (
        np.allclose(
            orthogonality,
            np.eye(3),
            atol=atol,
        )
        and np.isclose(
            determinant,
            1.0,
            atol=atol,
        )
    )


def rotation_x(angle: float) -> np.ndarray:
    """Return rotation matrix about the x-axis."""

    return axis_angle_rotation(
        axis=np.array([1.0, 0.0, 0.0]),
        angle=angle,
    )


def rotation_y(angle: float) -> np.ndarray:
    """Return rotation matrix about the y-axis."""

    return axis_angle_rotation(
        axis=np.array([0.0, 1.0, 0.0]),
        angle=angle,
    )


def rotation_z(angle: float) -> np.ndarray:
    """Return rotation matrix about the z-axis."""

    return axis_angle_rotation(
        axis=np.array([0.0, 0.0, 1.0]),
        angle=angle,
    )