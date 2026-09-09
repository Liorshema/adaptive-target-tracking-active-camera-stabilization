class ActuatorModel:
    def __init__(self, torque_constant, gear_ratio, efficiency):
        self.Kt = torque_constant
        self.G = gear_ratio
        self.eta = efficiency

    def current_to_motor_torque(self, current):
        """
        Convert motor current [A]
        to motor torque [N*m]
        """

        motor_torque = self.Kt * current

        return motor_torque

    def motor_to_wheel_torque(self, motor_torque):
        """
        Convert motor torque [N*m]
        to wheel torque after gearbox [N*m]
        """

        wheel_torque = motor_torque * self.G * self.eta

        return wheel_torque

    def current_to_wheel_torque(self, current):
        """
        Full actuator chain:
        current -> motor torque -> wheel torque
        """

        motor_torque = self.current_to_motor_torque(current)

        wheel_torque = self.motor_to_wheel_torque(
            motor_torque
        )

        return wheel_torque