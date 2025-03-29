#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice

class MotorController(Node):
    def __init__(self,
                 node_name='motor_controller',
                 pwm_pin=13,
                 frequency=1000,
                 initial_duty=0.0):
        """
        :param node_name:     ROS 2 node name
        :param pwm_pin:       GPIO pin number to use for PWM
        :param frequency:     PWM frequency (Hz)
        :param initial_duty:  Initial duty cycle (0.0 to 1.0)
        """
        super().__init__(node_name)
        
        self.pwm_pin = pwm_pin
        self.frequency = frequency

        # Create a PWM output on the specified pin with gpiozero
        self.motor = PWMOutputDevice(pin=self.pwm_pin, frequency=self.frequency)

        # Set initial duty cycle
        self.set_duty_cycle(initial_duty)

        self.get_logger().info(
            f"MotorController started on GPIO {self.pwm_pin} at {self.frequency} Hz with duty {initial_duty:.2f}."
        )

    def set_duty_cycle(self, duty):
        """
        Set the PWM duty cycle [0.0, 1.0].
        """
        duty = max(0.0, min(1.0, duty))  # clamp between 0 and 1
        self.motor.value = duty
        self.get_logger().info(f"Duty cycle set to {duty:.2f}")

    def destroy_node(self):
        """
        Properly stop PWM when node is destroyed.
        """
        self.set_duty_cycle(0.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # Example usage: GPIO 18, 1kHz, initial duty = 0.5
    node = MotorController(node_name='motor_controller',
                           pwm_pin=18,
                           frequency=1000,
                           initial_duty=0.8)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# cd ~/unibot_ws/src/uni_control_py/uni_control_py