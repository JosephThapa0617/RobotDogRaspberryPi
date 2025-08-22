from collections import deque


class PIDLegStabilizer:
    def __init__(
        self,
        kp_pitch=0.0,
        ki_pitch=0.0,
        kd_pitch=0.0,
        kp_roll=0.0,
        ki_roll=0.0,
        kd_roll=0.0,
        integral_window=100,
    ):
        self.kp_pitch = kp_pitch
        self.ki_pitch = ki_pitch
        self.kd_pitch = kd_pitch
        self.kp_roll = kp_roll
        self.ki_roll = ki_roll
        self.kd_roll = kd_roll

        self.integral_window = integral_window
        self.reset()

        self.pitch_setpoint = 0.0
        self.roll_setpoint = 0.0

    def set_setpoint(self, pitch_setpoint, roll_setpoint):
        self.pitch_setpoint = pitch_setpoint
        self.roll_setpoint = roll_setpoint

    def reset(self):
        """Reset the internal state of the PID controller."""
        self.pitch_error_history = deque(maxlen=self.integral_window)
        self.roll_error_history = deque(maxlen=self.integral_window)
        self.prev_pitch_error = 0.0
        self.prev_roll_error = 0.0

    def compute_leg_deltas(self, pitch, roll, dt):
        pitch_error = self.pitch_setpoint - pitch
        roll_error = self.roll_setpoint - roll

        self.pitch_error_history.append(pitch_error * dt)
        self.roll_error_history.append(roll_error * dt)

        pitch_integral = sum(self.pitch_error_history)
        roll_integral = sum(self.roll_error_history)

        pitch_derivative = (pitch_error - self.prev_pitch_error) / dt
        roll_derivative = (roll_error - self.prev_roll_error) / dt

        pitch_output = (
            self.kp_pitch * pitch_error
            + self.ki_pitch * pitch_integral
            + self.kd_pitch * pitch_derivative
        )
        roll_output = (
            self.kp_roll * roll_error
            + self.ki_roll * roll_integral
            + self.kd_roll * roll_derivative
        )

        self.prev_pitch_error = pitch_error
        self.prev_roll_error = roll_error

        delta_y_rf = -pitch_output - roll_output
        delta_y_lf = -pitch_output + roll_output
        delta_y_rb = +pitch_output - roll_output
        delta_y_lb = +pitch_output + roll_output

        return [delta_y_lf, delta_y_rf, delta_y_lb, delta_y_rb]
