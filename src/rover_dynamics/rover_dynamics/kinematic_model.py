import math


class RoverKinematicModel:
    def __init__(self, wheel_radius, track_width):
        self.r = wheel_radius
        self.L = track_width

    def body_to_wheel_velocity(self, v, omega):
        """
        Convert rover body velocity to wheel angular velocities.
        """

        v_left = v - omega * self.L / 2.0
        v_right = v + omega * self.L / 2.0

        omega_left = v_left / self.r
        omega_right = v_right / self.r

        return omega_left, omega_right

    def wheel_to_body_velocity(self, omega_left, omega_right):
        """
        Convert wheel angular velocities to rover body velocity.
        """

        v_left = self.r * omega_left
        v_right = self.r * omega_right

        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.L

        return v, omega

    def state_derivative(self, state, omega_left, omega_right):
        x, y, theta = state

        v, omega = self.wheel_to_body_velocity(
            omega_left,
            omega_right
        )

        x_dot = v * math.cos(theta)
        y_dot = v * math.sin(theta)
        theta_dot = omega

        return x_dot, y_dot, theta_dot

    def step(self, state, omega_left, omega_right, dt):
        x_dot, y_dot, theta_dot = self.state_derivative(
            state,
            omega_left,
            omega_right
        )

        x, y, theta = state

        x_next = x + x_dot * dt
        y_next = y + y_dot * dt
        theta_next = theta + theta_dot * dt

        return [x_next, y_next, theta_next]