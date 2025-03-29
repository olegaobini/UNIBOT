#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice, DigitalInputDevice

class MotorController(Node):
    '''
    duty cycle is negative edged (0.0 is fully on, 1.0 is fully off)
    '''
    def __init__(self,
                 pwm_pin,
                 channel_a,
                 channel_b,
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

        # Encoder setup
        self.encoder_a = DigitalInputDevice(channel_a)
        self.encoder_b = DigitalInputDevice(channel_b)
        self.pulse_count = 0
        self.last_time = time.time()

        # Attach interrupts to encoder pins
        self.encoder_a.when_activated = self._increment_pulse
        self.encoder_b.when_activated = self._increment_pulse

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


    def _increment_pulse(self):
        """
        Increment or decrement the pulse count based on the direction.
        """
        if self.encoder_a.value == self.encoder_b.value:
            self.pulse_count += 1  # Forward
        else:
            self.pulse_count -= 1  # Reverse

    def read_encoder_speed(self):
        """
        Calculate the speed of the motor based on encoder pulses.
        :return: Speed in revolutions per second (RPS)
        """
        current_time = time.time()
        time_interval = current_time - self.last_time

        # Calculate speed
        pulses_per_revolution = 360  # Replace with your encoder's PPR
        speed = (self.pulse_count / pulses_per_revolution) / time_interval

        # Reset pulse count and time
        self.pulse_count = 0
        self.last_time = current_time

        return speed            

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