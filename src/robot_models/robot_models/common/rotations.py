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

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def rotation_y(angle: float) -> np.ndarray:
    """Return rotation matrix about the y-axis."""

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )


def rotation_z(angle: float) -> np.ndarray:
    """Return rotation matrix about the z-axis."""

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )