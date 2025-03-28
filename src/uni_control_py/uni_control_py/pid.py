class PID:
    def __init__(self, kp, ki, kd, dt_sec, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt_sec
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if output > self.output_limit:
            output = self.output_limit
        elif output < -self.output_limit:
            output = -self.output_limit

        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0