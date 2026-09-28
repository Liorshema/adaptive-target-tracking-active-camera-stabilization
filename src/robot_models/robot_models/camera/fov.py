"""Camera field-of-view utilities."""

import numpy as np


class CameraFOV:
    """Check image visibility and distance from image boundaries."""

    def __init__(
        self,
        image_width: int,
        image_height: int,
    ) -> None:
        if image_width <= 0:
            raise ValueError(
                'image_width must be positive'
            )

        if image_height <= 0:
            raise ValueError(
                'image_height must be positive'
            )

        self.image_width = int(image_width)
        self.image_height = int(image_height)

    @property
    def bounds(self) -> np.ndarray:
        """Return image bounds [u_min, u_max, v_min, v_max]."""
        return np.array(
            [
                0.0,
                float(self.image_width - 1),
                0.0,
                float(self.image_height - 1),
            ],
            dtype=float,
        )

    def contains(
        self,
        pixel: np.ndarray,
    ) -> bool:
        """Return True if pixel lies inside the image."""
        pixel = np.asarray(
            pixel,
            dtype=float,
        )

        if pixel.shape != (2,):
            raise ValueError(
                'pixel must have shape (2,)'
            )

        u, v = pixel

        u_min, u_max, v_min, v_max = (
            self.bounds
        )

        return bool(
            u_min <= u <= u_max
            and v_min <= v <= v_max
        )

    def margin(
        self,
        pixel: np.ndarray,
    ) -> float:
        """
        Return signed distance to the nearest image boundary.

        Positive:
            pixel is inside the image.

        Zero:
            pixel lies on the boundary.

        Negative:
            pixel is outside the image.
        """
        pixel = np.asarray(
            pixel,
            dtype=float,
        )

        if pixel.shape != (2,):
            raise ValueError(
                'pixel must have shape (2,)'
            )

        u, v = pixel

        u_min, u_max, v_min, v_max = (
            self.bounds
        )

        boundary_distances = np.array(
            [
                u - u_min,
                u_max - u,
                v - v_min,
                v_max - v,
            ],
            dtype=float,
        )

        return float(
            np.min(boundary_distances)
        )
