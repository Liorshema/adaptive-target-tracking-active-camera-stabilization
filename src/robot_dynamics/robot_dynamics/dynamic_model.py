import math


class RobotDynamicModel:
    def __init__(self, mass, yaw_inertia, track_width):
        self.m = mass
        self.Iz = yaw_inertia
        self.L = track_width

    def wheel_forces_to_body(self, force_left, force_right):
        """
        Convert left/right longitudinal forces
        into total body force and yaw torque.
        """

        force_x = force_left + force_right

        torque_z = (
            force_right - force_left
        ) * (self.L / 2.0)

        return force_x, torque_z

    def state_derivative(self, state, force_left, force_right):
        """
        state = [x, y, theta, v, omega]

        force_left  = total longitudinal force from left side [N]
        force_right = total longitudinal force from right side [N]
        """

        x, y, theta, v, omega = state

        force_x, torque_z = self.wheel_forces_to_body(
            force_left,
            force_right
        )

        # Kinematics
        x_dot = v * math.cos(theta)
        y_dot = v * math.sin(theta)
        theta_dot = omega

        # Dynamics
        v_dot = force_x / self.m
        omega_dot = torque_z / self.Iz

        return [
            x_dot,
            y_dot,
            theta_dot,
            v_dot,
            omega_dot
        ]

    def step(self, state, force_left, force_right, dt):
        derivatives = self.state_derivative(
            state,
            force_left,
            force_right
        )

        next_state = [
            state[i] + derivatives[i] * dt
            for i in range(len(state))
        ]

        return next_state