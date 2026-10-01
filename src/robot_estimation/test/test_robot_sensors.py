import numpy as np
import pytest

from robot_estimation.robot.sensors.imu import ImuSensor
from robot_estimation.robot.sensors.joint_encoder import JointEncoderSensor
from robot_estimation.robot.sensors.wheel_odometry import (
    WheelOdometrySensor,
)


def test_imu_measurement_creation() -> None:
    sensor = ImuSensor(
        spatial_dimension=3,
        covariance=np.eye(6),
    )

    raw_value = np.array([
        0.1,
        -0.2,
        9.81,
        0.01,
        -0.02,
        0.03,
    ])

    measurement = sensor.create_measurement(
        raw_value=raw_value,
        timestamp=1.5,
    )

    assert sensor.measurement_dimension == 6
    assert np.allclose(measurement.value, raw_value)
    assert np.allclose(measurement.covariance, np.eye(6))
    assert measurement.timestamp == 1.5


def test_imu_invalid_measurement_dimension_raises_error() -> None:
    sensor = ImuSensor(
        spatial_dimension=3,
        covariance=np.eye(6),
    )

    with pytest.raises(ValueError):
        sensor.create_measurement(
            raw_value=np.zeros(5),
            timestamp=0.0,
        )


def test_imu_invalid_covariance_dimension_raises_error() -> None:
    with pytest.raises(ValueError):
        ImuSensor(
            spatial_dimension=3,
            covariance=np.eye(3),
        )


def test_wheel_odometry_measurement_creation() -> None:
    covariance = np.diag([
        0.02,
        0.01,
    ])

    sensor = WheelOdometrySensor(
        covariance=covariance,
    )

    raw_value = np.array([
        1.2,
        0.15,
    ])

    measurement = sensor.create_measurement(
        raw_value=raw_value,
        timestamp=2.0,
    )

    assert sensor.measurement_dimension == 2
    assert np.allclose(measurement.value, raw_value)
    assert np.allclose(measurement.covariance, covariance)
    assert measurement.timestamp == 2.0


def test_wheel_odometry_invalid_measurement_dimension_raises_error() -> None:
    sensor = WheelOdometrySensor(
        covariance=np.eye(2),
    )

    with pytest.raises(ValueError):
        sensor.create_measurement(
            raw_value=np.zeros(3),
            timestamp=0.0,
        )


def test_joint_encoder_measurement_creation() -> None:
    joint_count = 4

    sensor = JointEncoderSensor(
        joint_count=joint_count,
        covariance=np.eye(2 * joint_count),
    )

    raw_value = np.array([
        0.1,
        0.2,
        -0.1,
        0.3,
        0.01,
        0.02,
        -0.01,
        0.03,
    ])

    measurement = sensor.create_measurement(
        raw_value=raw_value,
        timestamp=3.0,
    )

    assert sensor.joint_count == 4
    assert sensor.measurement_dimension == 8
    assert np.allclose(measurement.value, raw_value)
    assert measurement.timestamp == 3.0


def test_joint_encoder_invalid_measurement_dimension_raises_error() -> None:
    sensor = JointEncoderSensor(
        joint_count=4,
        covariance=np.eye(8),
    )

    with pytest.raises(ValueError):
        sensor.create_measurement(
            raw_value=np.zeros(7),
            timestamp=0.0,
        )


def test_joint_encoder_invalid_joint_count_raises_error() -> None:
    with pytest.raises(ValueError):
        JointEncoderSensor(
            joint_count=0,
            covariance=np.eye(2),
        )