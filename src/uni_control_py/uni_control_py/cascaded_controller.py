#cascaded_controller.py

from pid import PID

class CascadedController:
    """
    A 3-loop cascaded PID controller:
      - Inner Loop:  controls angular acceleration (IMU data)
      - Middle Loop: controls angular position (via integrated acceleration)
      - Outer Loop:  controls wheel speed (via encoder measurement)
    """
    def __init__(
        self,
        kp_inner, ki_inner, kd_inner, limit_inner,
        kp_middle, ki_middle, kd_middle, limit_middle,
        kp_outer, ki_outer, kd_outer, limit_outer,
        dt, wheel_speed_setpoint,
    ):

        self.dt = dt
        self.wheel_speed_setpoint = wheel_speed_setpoint # Control the robots forward speed

        self.inner_pid  = PID(kp_inner,  ki_inner,  kd_inner,  dt, limit_inner)
        self.middle_pid = PID(kp_middle, ki_middle, kd_middle, dt, limit_middle)
        self.outer_pid  = PID(kp_outer,  ki_outer,  kd_outer,  dt, limit_outer)
        

    def update(self, angle_vel_measurement, angle_pos_measurement, wheel_speed_measurement):
        """
        1. Outer loop:   speed error = (speed_setpoint - speed_measurement)
           => Middle setpoint is angle setpoint = outer_pid.update(speed_error)

        2. Middle loop:  angle pos error = (angle_pos_setpoint - angle_pos_measurement)
           => Inner setpoint = middle_pid.update(angle_pos_error)

        3. Inner loop:   angular vel error = (angular_vel_setpoint - angular_vel_measurement)
           => final output = inner_pid.update(angular_vel_error)

        Return final control signal(e.g. duty cycle command).
        """

        # Outer loop
        wheel_speed_error  = self.wheel_speed_setpoint - wheel_speed_measurement
        angle_sp_cmd = self.outer_pid.update(wheel_speed_error)

        # Middle loop 
        angle_pos_error  = angle_sp_cmd - angle_pos_measurement
        accel_sp_cmd = self.middle_pid.update(angle_pos_error)

        # Inner loop
        angle_pos_error = accel_sp_cmd - angle_vel_measurement
        motor_output = self.inner_pid.update(angle_pos_error)

        return motor_output


    def reset_all(self):
        self.inner_pid.reset()
        self.middle_pid.reset()
        self.outer_pid.reset()

    #method to set the setpoints
    def set_speed_setpoint(self, sp):
        self.speed_setpoint = sp