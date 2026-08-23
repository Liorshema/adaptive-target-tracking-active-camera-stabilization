import math


class ArmKinematics:
    def __init__(
        self,
        link1_length,
        link2_length,
        wrist_length
    ):
        self.l1 = link1_length
        self.l2 = link2_length
        self.l3 = wrist_length

    def forward_kinematics(
        self,
        q0,
        q1,
        q2,
        q3
    ):
        """
        4-DOF camera arm forward kinematics.

        q0 = base yaw [rad]
        q1 = shoulder pitch [rad]
        q2 = elbow pitch [rad]
        q3 = wrist pitch [rad]

        Returns:
        x_camera
        y_camera
        z_camera
        camera_pitch
        """

        # Position of camera in the vertical arm plane
        rho = (
            self.l1 * math.cos(q1)
            + self.l2 * math.cos(q1 + q2)
            + self.l3 * math.cos(q1 + q2 + q3)
        )

        z_camera = (
            self.l1 * math.sin(q1)
            + self.l2 * math.sin(q1 + q2)
            + self.l3 * math.sin(q1 + q2 + q3)
        )

        # Rotate the whole arm plane using base yaw
        x_camera = rho * math.cos(q0)
        y_camera = rho * math.sin(q0)

        # Camera pitch orientation
        camera_pitch = q1 + q2 + q3

        return (
            x_camera,
            y_camera,
            z_camera,
            camera_pitch
        )