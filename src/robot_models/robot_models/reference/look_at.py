"""Look-at orientation generation for the camera."""

import numpy as np

from robot_models.common.rotations import is_rotation_matrix


class LookAtReference:
    """Generate a desired camera orientation toward a target."""

    @staticmethod
    def compute(
        camera_position_world: np.ndarray,
        target_position_world: np.ndarray,
        world_up: np.ndarray = np.array(
            [0.0, 0.0, 1.0],
            dtype=float,
        ),
    ) -> np.ndarray:
        """Return desired camera rotation R_W_E.

        Camera optical-frame convention:
            x -> right
            y -> down
            z -> forward
        """

        camera_position_world = np.asarray(
            camera_position_world,
            dtype=float,
        )

        target_position_world = np.asarray(
            target_position_world,
            dtype=float,
        )

        world_up = np.asarray(
            world_up,
            dtype=float,
        )

        if camera_position_world.shape != (3,):
            raise ValueError(
                "camera_position_world must have shape (3,)"
            )

        if target_position_world.shape != (3,):
            raise ValueError(
                "target_position_world must have shape (3,)"
            )

        if world_up.shape != (3,):
            raise ValueError(
                "world_up must have shape (3,)"
            )

        direction = (
            target_position_world
            - camera_position_world
        )

        direction_norm = np.linalg.norm(direction)

        if np.isclose(direction_norm, 0.0):
            raise ValueError(
                "camera and target positions must be different"
            )

        z_axis_world = (
            direction / direction_norm
        )

        world_up_norm = np.linalg.norm(world_up)

        if np.isclose(world_up_norm, 0.0):
            raise ValueError(
                "world_up must be non-zero"
            )

        up_axis_world = (
            world_up / world_up_norm
        )

        x_axis_world = np.cross(
            z_axis_world,
            up_axis_world,
        )

        if np.isclose(
            np.linalg.norm(x_axis_world),
            0.0,
        ):
            fallback_up = np.array(
                [0.0, 1.0, 0.0],
                dtype=float,
            )

            x_axis_world = np.cross(
                z_axis_world,
                fallback_up,
            )

        x_axis_world /= np.linalg.norm(
            x_axis_world
        )

        y_axis_world = np.cross(
            z_axis_world,
            x_axis_world,
        )

        y_axis_world /= np.linalg.norm(
            y_axis_world
        )

        rotation_world_camera = np.column_stack(
            (
                x_axis_world,
                y_axis_world,
                z_axis_world,
            )
        )

        if not is_rotation_matrix(
            rotation_world_camera
        ):
            raise RuntimeError(
                "failed to construct a valid rotation matrix"
            )

        return rotation_world_camera