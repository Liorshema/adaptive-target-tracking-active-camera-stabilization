class WheelModel:
    def __init__(self, wheel_radius):
        self.r = wheel_radius

    def torque_to_force(self, wheel_torque):
        """
        Convert wheel torque [N*m]
        to longitudinal traction force [N]
        """

        force = wheel_torque / self.r

        return force