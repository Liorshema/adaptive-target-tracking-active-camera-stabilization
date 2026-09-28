"""Whole-body acceleration constraint utilities."""

import numpy as np


class AccelerationLimits:
    """Represent box constraints on generalized acceleration."""

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

    def acceleration(
        self,
        generalized_velocity_current: np.ndarray,
        generalized_velocity_previous: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Compute finite-difference generalized acceleration."""

        generalized_velocity_current = np.asarray(
            generalized_velocity_current,
            dtype=float,
        )

        generalized_velocity_previous = np.asarray(
            generalized_velocity_previous,
            dtype=float,
        )

        if generalized_velocity_current.shape != self.lower.shape:
            raise ValueError(
                "current velocity must match limit shape"
            )

        if generalized_velocity_previous.shape != self.lower.shape:
            raise ValueError(
                "previous velocity must match limit shape"
            )

        if dt <= 0.0:
            raise ValueError(
                "dt must be positive"
            )

        return (
            generalized_velocity_current
            - generalized_velocity_previous
        ) / dt

    def residual(
        self,
        generalized_velocity_current: np.ndarray,
        generalized_velocity_previous: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Return signed distance from acceleration boundaries."""

        generalized_acceleration = self.acceleration(
            generalized_velocity_current,
            generalized_velocity_previous,
            dt,
        )

        lower_residual = (
            generalized_acceleration
            - self.lower
        )

        upper_residual = (
            self.upper
            - generalized_acceleration
        )

        return np.concatenate(
            (
                lower_residual,
                upper_residual,
            )
        )

    def contains(
        self,
        generalized_velocity_current: np.ndarray,
        generalized_velocity_previous: np.ndarray,
        dt: float,
    ) -> bool:
        """Return True when all acceleration limits are satisfied."""

        return bool(
            np.all(
                self.residual(
                    generalized_velocity_current,
                    generalized_velocity_previous,
                    dt,
                ) >= 0.0
            )
        )