"""Whole-body velocity constraint utilities."""

import numpy as np


class VelocityLimits:
    """Represent box constraints on generalized velocity."""

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
                'lower must be a 1D vector'
            )

        if upper.shape != lower.shape:
            raise ValueError(
                'upper must match lower shape'
            )

        if np.any(lower > upper):
            raise ValueError(
                'lower limits must not exceed upper limits'
            )

        self.lower = lower
        self.upper = upper

    def residual(
        self,
        generalized_velocity: np.ndarray,
    ) -> np.ndarray:
        """Return signed distance from velocity boundaries."""
        generalized_velocity = np.asarray(
            generalized_velocity,
            dtype=float,
        )

        if generalized_velocity.shape != self.lower.shape:
            raise ValueError(
                'generalized_velocity must match limit shape'
            )

        lower_residual = (
            generalized_velocity
            - self.lower
        )

        upper_residual = (
            self.upper
            - generalized_velocity
        )

        return np.concatenate(
            (
                lower_residual,
                upper_residual,
            )
        )

    def contains(
        self,
        generalized_velocity: np.ndarray,
    ) -> bool:
        """Return True when all velocity limits are satisfied."""
        return bool(
            np.all(
                self.residual(
                    generalized_velocity
                ) >= 0.0
            )
        )
