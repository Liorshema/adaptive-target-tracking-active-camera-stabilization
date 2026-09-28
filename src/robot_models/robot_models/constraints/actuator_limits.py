"""Actuator constraint utilities."""

import numpy as np


class ActuatorLimits:
    """Represent box constraints on actuator commands."""

    def __init__(
        self,
        lower: np.ndarray,
        upper: np.ndarray,
    ) -> None:
        lower = np.asarray(
            lower,
            dtype=float,
        )

        upper = np.asarray(
            upper,
            dtype=float,
        )

        if lower.ndim != 1:
            raise ValueError(
                "lower must be a 1D vector"
            )

        if upper.shape != lower.shape:
            raise ValueError(
                "upper must match lower shape"
            )

        if np.any(lower > upper):
            raise ValueError(
                "lower limits must not exceed upper limits"
            )

        self.lower = lower
        self.upper = upper

    def residual(
        self,
        command: np.ndarray,
    ) -> np.ndarray:
        """Return signed distance from actuator boundaries."""

        command = np.asarray(
            command,
            dtype=float,
        )

        if command.shape != self.lower.shape:
            raise ValueError(
                "command must match limit shape"
            )

        lower_residual = (
            command - self.lower
        )

        upper_residual = (
            self.upper - command
        )

        return np.concatenate(
            (
                lower_residual,
                upper_residual,
            )
        )

    def contains(
        self,
        command: np.ndarray,
    ) -> bool:
        """Return True when all actuator limits are satisfied."""

        return bool(
            np.all(
                self.residual(command) >= 0.0
            )
        )