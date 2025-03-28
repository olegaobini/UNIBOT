# cascaded_controller.py

from pid import PID

class CascadedController:
    """
    A generic 3-loop cascaded PID controller:
      - Inner Loop:  controls angular acceleration (IMU data)
      - Middle Loop: controls angular position (via integrated acceleration or angle measurement)
      - Outer Loop:  controls wheel speed (via encoder measurement)

    This same class can be used for either reaction wheels (target speed = 0)
    or the main drive wheel (target speed from user input).
    """
    def __init__(
        self,
        kp_inner, ki_inner, kd_inner, limit_inner,
        kp_middle, ki_middle, kd_middle, limit_middle,
        kp_outer, ki_outer, kd_outer, limit_outer,
        dt
    ):
        self.inner_pid  = PID(kp_inner,  ki_inner,  kd_inner,  dt, limit_inner)
        self.middle_pid = PID(kp_middle, ki_middle, kd_middle, dt, limit_middle)
        self.outer_pid  = PID(kp_outer,  ki_outer,  kd_outer,  dt, limit_outer)

        # You might keep track of setpoints, e.g.:
        self.accel_setpoint  = 0.0
        self.angle_setpoint  = 0.0
        self.speed_setpoint  = 0.0

        self.dt = dt

    def update(self, accel_measurement, angle_measurement, speed_measurement):
        """
        1. Outer loop:   speed error = (speed_setpoint - speed_measurement)
           => Middle setpoint is angle setpoint = outer_pid.update(speed_error)
        2. Middle loop:  angle error = (angle_setpoint - angle_measurement)
           => Inner setpoint = angle_acceleration_setpoint = middle_pid.update(angle_error)
        3. Inner loop:   accel error = (accel_setpoint - accel_measurement)
           => final output = inner_pid.update(accel error)

        Return final control signal, e.g. duty cycle command.
        """

        # Outer loop -> sets angle setpoint
        speed_error  = self.speed_setpoint - speed_measurement
        angle_sp_cmd = self.outer_pid.update(speed_error)

        # Middle loop -> sets acceleration setpoint
        angle_error  = angle_sp_cmd - angle_measurement
        accel_sp_cmd = self.middle_pid.update(angle_error)

        # Inner loop
        accel_error = accel_sp_cmd - accel_measurement
        motor_output = self.inner_pid.update(accel_error)

        return motor_output

    def reset_all(self):
        self.inner_pid.reset()
        self.middle_pid.reset()
        self.outer_pid.reset()

    # Optionally, add methods to set the setpoints
    def set_speed_setpoint(self, sp):
        self.speed_setpoint = sp

    def set_angle_setpoint(self, sp):
        self.angle_setpoint = sp  # if you want a direct angle setpoint
    # etc.

