#!/usr/bin/env python3

import time

from cascaded_controller import CascadedController
from motor_control import MotorController
from mpu6050 import MPU6050


def main():
    """
    reaction motor 1: gpio pin 18
    reaction motor 2: gpio pin 12
    main motor: gpio pin 13
    """

    dt = 0.01 # period of loop
    
    sensor = MPU6050() # Your IMU sensor class
    motor1 = MotorController(pwm_pin=18, frequency=1000, initial_duty=0.0)
    motor2 = MotorController(pwm_pin=19, frequency=1000, initial_duty=0.0) 
    motor_main = MotorController(pwm_pin=13, frequency=1000, initial_duty=0.0) 

    reaction_wheel_control = CascadedController(
        kp_inner=1.0, ki_inner=0.1, kd_inner=0.01, limit_inner=1.0,
        kp_middle=2.0, ki_middle=0.2, kd_middle=0.02, limit_middle=1.0,
        kp_outer=3.0, ki_outer=0.3, kd_outer=0.03, limit_outer=1.0,
        dt=dt
    )

    main_wheel_control = CascadedController(
        kp_inner=1.0, ki_inner=0.1, kd_inner=0.01, limit_inner=1.0,
        kp_middle=2.0, ki_middle=0.2, kd_middle=0.02, limit_middle=1.0,
        kp_outer=3.0, ki_outer=0.3, kd_outer=0.03, limit_outer=1.0,
        dt=dt
    )

    # Main Control Loop
    try:
        while True:
            loop_start = time.time()

            # 2.1) Read sensors
            Ax, Ay, Az = sensor.read_accel()  
            # For balancing, you might need gyro data or angle estimation:
            # Gx, Gy, Gz = sensor.read_gyro() or use a filter to get orientation
            angle_measurement = compute_angle_from_accel_gyro(Ax, Ay, Az)  
            speed_measurement = read_encoder_speed()  
            accel_measurement = Ax  # or however you define "accel" for your inner loop

            # 2.2) Update the cascaded PID
            # Optionally set a speed setpoint if needed
            # wheel_controller.set_speed_setpoint(desired_speed)
            motor_command = reaction_wheel_control.update(
                accel_measurement, 
                angle_measurement,
                speed_measurement
            )

            # 2.3) Send command to motor(s)
            # "motor_command" might be in [-1.0, +1.0], so convert to duty cycle
            duty_cycle = (motor_command + 1.0) / 2.0  # e.g. shift range
            duty_cycle = min(max(duty_cycle, 0.0), 1.0)  # clamp to [0..1]
            motor_left.set_duty_cycle(duty_cycle)

            # 2.4) Wait until next loop iteration
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're behind schedule, skip sleeping or log a warning
                pass

    except KeyboardInterrupt:
        # Graceful shutdown
        motor1.set_duty_cycle(0.0)
        motor2.set_duty_cycle(0.0)
        motor_main.set_duty_cycle(0.0)

def compute_angle_from_accel_gyro(Ax, Ay, Az):
    # Placeholder function
    # Typically you'd do a complementary or Kalman filter here.
    # For quick demos: approximate pitch from Ay, roll from Ax, etc.
    return 0.0

def read_encoder_speed():
    # Placeholder function
    return 0.0

if __name__ == "__main__":
    main()
