"""Tests for mobile-base models."""

import numpy as np

from robot_models.base.differential_drive.kinematics import (
    DifferentialDriveKinematics,
)


def test_differential_drive_velocity_mapping() -> None:
    """Verify wheel-rate to body-velocity mapping."""
    model = DifferentialDriveKinematics(
        wheel_radius=0.15,
        track_width=0.60,
    )

    wheel_rates = np.array([2.0, 2.0])

    body_velocity = model.wheel_rates_to_body_velocity(
        wheel_rates
    )

    expected = np.array([0.30, 0.0])

    assert np.allclose(body_velocity, expected)


def test_differential_drive_pose_derivative() -> None:
    """Verify planar pose derivative."""
    model = DifferentialDriveKinematics(
        wheel_radius=0.15,
        track_width=0.60,
    )

    pose = np.array([
        1.0,
        2.0,
        np.pi / 2.0,
    ])

    generalized_velocity = np.array([
        0.30,
        0.20,
    ])

    derivative = model.pose_derivative(
        pose,
        generalized_velocity,
    )

    expected = np.array([
        0.0,
        0.30,
        0.20,
    ])

    assert np.allclose(
        derivative,
        expected,
        atol=1e-12,
    )


def test_differential_drive_spatial_jacobian() -> None:
    """Verify base twist Jacobian."""
    model = DifferentialDriveKinematics(
        wheel_radius=0.15,
        track_width=0.60,
    )

    point_base = np.array([
        0.88,
        0.0,
        0.18,
    ])

    jacobian = model.spatial_jacobian(
        point_base
    )

    expected = np.array([
        [1.0, 0.0],
        [0.0, 0.88],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 1.0],
    ])

    assert jacobian.shape == (6, 2)
    assert np.allclose(jacobian, expected)
