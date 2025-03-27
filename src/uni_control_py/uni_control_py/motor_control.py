#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from gpiozero import PWMOutputDevice
from time import sleep

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Set up a PWM output on GPIO pin 18 at 1 kHz
        self.pwm_pin = 13
        self.frequency = 1000  # 1 kHz
        self.motor = PWMOutputDevice(pin=self.pwm_pin, frequency=self.frequency)

        # For demonstration, set a 50% duty cycle:
        self.motor.value = 0.5  # value range: 0.0 (off) -> 1.0 (fully on)
        self.get_logger().info(
            f"MotorController started. Software PWM at {self.frequency} Hz on GPIO {self.pwm_pin} at 50% duty."
        )

        # Optionally, use a ROS timer to vary the duty cycle
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.duty_direction = 1
        self.current_duty = 0.5

    def timer_callback(self):
        """
        Example: ramp duty cycle up and down between 0.0 and 1.0 over time.
        This is just to demonstrate changing PWM in software.
        """
        step = 0.1 * self.duty_direction
        self.current_duty += step

        if self.current_duty >= 1.0:
            self.current_duty = 1.0
            self.duty_direction = -1
        elif self.current_duty <= 0.0:
            self.current_duty = 0.0
            self.duty_direction = 1

        self.motor.value = self.current_duty
        self.get_logger().info(f"Current duty cycle set to {self.current_duty:.2f}")

    def destroy_node(self):
        # Turn off the motor
        self.motor.value = 0.0
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

