"""Load mathematical robot-model parameters from robot description."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class DifferentialDriveParameters:
    """Parameters required by a differential-drive model."""

    wheel_radius: float
    track_width: float


@dataclass(frozen=True)
class SerialArmParameters:
    """Parameters required by a serial-arm model."""

    joint_axes: np.ndarray
    body_height: float
    arm_base_height: float
    link_1_length: float
    link_2_length: float
    wrist_length: float


@dataclass(frozen=True)
class JointLimitParameters:
    """Manipulator joint limits and actuator-related bounds."""

    lower: np.ndarray
    upper: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray
    damping: np.ndarray


@dataclass(frozen=True)
class RobotModelParameters:
    """Parameters required to construct the mathematical robot model."""

    base: DifferentialDriveParameters
    manipulator: SerialArmParameters
    joint_limits: JointLimitParameters


def load_robot_model_parameters(
    config_path: str | Path | None = None,
) -> RobotModelParameters:
    """
    Load and validate robot-model parameters.

    When no path is supplied, load robot.yaml from the installed
    robot_description package.
    """
    path = (
        Path(config_path)
        if config_path is not None
        else _default_config_path()
    )

    with path.open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError('Robot configuration must be a mapping.')

    robot = _require_mapping(data, 'robot')
    body = _require_mapping(robot, 'body')
    wheels = _require_mapping(robot, 'wheels')
    arm = _require_mapping(robot, 'arm')
    arm_base = _require_mapping(arm, 'base')
    link_1 = _require_mapping(arm, 'link1')
    link_2 = _require_mapping(arm, 'link2')
    wrist = _require_mapping(arm, 'wrist')
    joints = _require_mapping(arm, 'joints')

    joint_names = (
        'base_yaw',
        'shoulder',
        'elbow',
        'wrist',
    )

    joint_configs = [
        _require_mapping(joints, name)
        for name in joint_names
    ]

    joint_axes = np.asarray(
        [
            config['axis']
            for config in joint_configs
        ],
        dtype=float,
    )

    if joint_axes.shape != (4, 3):
        raise ValueError(
            'Manipulator joint axes must have shape (4, 3).'
        )

    axis_norms = np.linalg.norm(
        joint_axes,
        axis=1,
    )

    if np.any(axis_norms <= 0.0):
        raise ValueError(
            'Manipulator joint axes must be non-zero.'
        )

    joint_axes = (
        joint_axes
        / axis_norms[:, np.newaxis]
    )

    wheel_radius = _positive_float(
        wheels,
        'radius',
    )

    wheel_y_offset = _positive_float(
        wheels,
        'y_offset',
    )

    # Distance between left and right wheel centerlines.
    track_width = 2.0 * wheel_y_offset

    base_parameters = DifferentialDriveParameters(
        wheel_radius=wheel_radius,
        track_width=track_width,
    )

    manipulator_parameters = SerialArmParameters(
        joint_axes=joint_axes,
        body_height=_positive_float(
            body,
            'height',
        ),
        arm_base_height=_positive_float(
            arm_base,
            'height',
        ),
        link_1_length=_positive_float(
            link_1,
            'length',
        ),
        link_2_length=_positive_float(
            link_2,
            'length',
        ),
        wrist_length=_positive_float(
            wrist,
            'length',
        ),
    )

    joint_limit_parameters = JointLimitParameters(
        lower=_joint_vector(
            joint_configs,
            'lower',
        ),
        upper=_joint_vector(
            joint_configs,
            'upper',
        ),
        velocity=_joint_vector(
            joint_configs,
            'velocity',
        ),
        effort=_joint_vector(
            joint_configs,
            'effort',
        ),
        damping=_joint_vector(
            joint_configs,
            'damping',
        ),
    )

    if np.any(
        joint_limit_parameters.lower
        > joint_limit_parameters.upper
    ):
        raise ValueError(
            'Joint lower limits cannot exceed upper limits.'
        )

    return RobotModelParameters(
        base=base_parameters,
        manipulator=manipulator_parameters,
        joint_limits=joint_limit_parameters,
    )


def _default_config_path() -> Path:
    """Return the installed robot.yaml path."""
    package_share = Path(
        get_package_share_directory(
            'robot_description'
        )
    )

    return (
        package_share
        / 'config'
        / 'robot'
        / 'robot.yaml'
    )


def _require_mapping(
    mapping: dict[str, Any],
    key: str,
) -> dict[str, Any]:
    """Return a required nested mapping."""
    value = mapping.get(key)

    if not isinstance(value, dict):
        raise ValueError(
            f'Missing or invalid mapping: {key}'
        )

    return value


def _positive_float(
    mapping: dict[str, Any],
    key: str,
) -> float:
    """Return a required positive scalar parameter."""
    if key not in mapping:
        raise ValueError(
            f'Missing parameter: {key}'
        )

    value = float(mapping[key])

    if value <= 0.0:
        raise ValueError(
            f'Parameter {key} must be positive.'
        )

    return value


def _joint_vector(
    joint_configs: list[dict[str, Any]],
    key: str,
) -> np.ndarray:
    """Collect one joint parameter into a vector."""
    try:
        vector = np.asarray(
            [
                config[key]
                for config in joint_configs
            ],
            dtype=float,
        )
    except KeyError as error:
        raise ValueError(
            f'Missing joint parameter: {key}'
        ) from error

    if vector.shape != (4,):
        raise ValueError(
            f'Joint parameter {key} must have shape (4,).'
        )

    return vector