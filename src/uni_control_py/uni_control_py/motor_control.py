#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice

class MotorController(Node):
    '''
    duty cycle is negative edged (0.0 is fully on, 1.0 is fully off)
    '''
    def __init__(self,
                 pwm_pin,
                 node_name='motor_controller',
                 frequency=1000,
                 duty=1.0): #initial duty cycle is 1.0 (off)
        """
        :param node_name:     ROS 2 node name
        :param pwm_pin:       GPIO pin number to use for PWM
        :param frequency:     PWM frequency (Hz)
        :param duty:  duty cycle (0.0 (fully on) to 1.0 (fully off))
        """
        super().__init__(node_name)
        
        self.pwm_pin = pwm_pin
        self.frequency = frequency

        # Create a PWM output on the specified pin with gpiozero
        self.motor = PWMOutputDevice(pin=self.pwm_pin, frequency=self.frequency)

        # Set initial duty cycle
        self.set_duty_cycle(duty) # the motors are expecting the negative edge duty cycle, but that's stupid

        self.get_logger().info(
            f"MotorController started on GPIO {self.pwm_pin} at {self.frequency} Hz with duty {duty:.2f}."
        )
    
    # Set the PWM duty cycle [0.0, 1.0].
    def set_duty_cycle(self, duty):
        duty = max(0.0, min(1.0, duty))  # clamp between 0 and 1
        self.motor.value = duty
        self.get_logger().info(f"Duty cycle set to {duty:.2f}")

    def read_encoder_speed(self):
        # Placeholder function
        return 0.0

    def destroy_node(self):
        """
        Properly stop PWM when node is destroyed.
        """
        self.set_duty_cycle(0.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # Example usage: GPIO 18, 1kHz, initial duty = 0.5
    node = MotorController(pwm_pin=12, 
                           node_name='motor_controller',
                           frequency=1000,
                           duty=0.8)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()