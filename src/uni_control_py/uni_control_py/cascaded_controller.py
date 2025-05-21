#cascaded_controller.py

from pid import PID

class CascadedController:
    """
    A 3-loop cascaded PID controller:
      - Inner Loop:  controls angular acceleration (IMU data)
      - Middle Loop: controls angular position (via integrated acceleration)
      - Outer Loop:  controls wheel speed/robot velocity (via encoder measurement/IMU velocity data)
    """
    def __init__(
        self,
        kp_inner, ki_inner, kd_inner, limit_inner,
        kp_middle, ki_middle, kd_middle, limit_middle,
        kp_outer, ki_outer, kd_outer, limit_outer,
        dt, outer_loop_setpoint,
    ):

        self.dt = dt
        self.outer_loop_setpoint = outer_loop_setpoint # Control the robots forward speed

        self.inner_pid  = PID(kp_inner,  ki_inner,  kd_inner,  dt, limit_inner)
        self.middle_pid = PID(kp_middle, ki_middle, kd_middle, dt, limit_middle)
        self.outer_pid  = PID(kp_outer,  ki_outer,  kd_outer,  dt, limit_outer)
        

    def update(self, inner_loop_measurement, middle_loop_measurement, outer_loop_measurement):
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
        outer_loop_error  = self.outer_loop_setpoint - outer_loop_measurement
        outer_loop_output = self.outer_pid.update(outer_loop_error)

        # Middle loop 
        middle_loop_error  = outer_loop_output - middle_loop_measurement
        middle_loop_output = self.middle_pid.update(middle_loop_error)

        # Inner loop
        inner_loop_error = middle_loop_output - inner_loop_measurement
        output = self.inner_pid.update(inner_loop_error)

        return output


    def reset_all(self):
        self.inner_pid.reset()
        self.middle_pid.reset()
        self.outer_pid.reset()

    #method to set the setpoints
    def set_outer_loop_setpoint(self, sp):
        self.outer_loop_setpoint = sp