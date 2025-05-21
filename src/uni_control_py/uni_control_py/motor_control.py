#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice, DigitalInputDevice, DigitalOutputDevice

class MotorController(Node):
    '''
    duty cycle is negative edged (0.0 is fully on, 1.0 is fully off)
    the quadrature encoding is the sketchiest part of this entire code base.
    '''
    def __init__(self,
                 pwm_pin,
                 dir_pin,
                 channel_a_pin,
                 channel_b_pin,
                 pwm_frequency=1000,
                 initial_duty=0.0, #initial duty cycle is 1.0 (off)
                 node_name='motor_controller',):
        """
        :param node_name:     ROS 2 node name
        :param pwm_pin:       GPIO pin number to use for PWM
        :param frequency:     PWM frequency (Hz)
        :param duty:  duty cycle (0.0 (fully on) to 1.0 (fully off))
        """
        super().__init__(node_name)
        
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.pwm_frequency = pwm_frequency

        # Create a PWM output on the specified pin with gpiozero
        self.motor = PWMOutputDevice(pin=self.pwm_pin, frequency=self.pwm_frequency)
        self.direction = DigitalOutputDevice(pin=self.dir_pin)

        # Qaudrature Encoder setup
        #self.encoder_a = DigitalInputDevice(channel_a_pin)
        #self.encoder_b = DigitalInputDevice(channel_b_pin)
        #self.pulse_count = 0
        #self.last_time = time.time()

        # Attach interrupts to encoder pins
        #self.encoder_a.when_activated = self._increment_pulse
        #self.encoder_b.when_activated = self._increment_pulse

        # Set initial duty cycle
        self.set_duty_cycle(initial_duty) # the motors are expecting the negative edge duty cycle, but that's stupid

        # self.get_logger().info(
        #     f"MotorController started on GPIO {self.pwm_pin} at {self.pwm_frequency} Hz with duty {initial_duty:.2f}."
        # )

    def set_duty_cycle(self, duty):
        """
        Set the PWM duty cycle and control motor direction.
        :param duty: Duty cycle (-1.0 to 1.0). Negative values reverse the motor.
        """
        # Clamp the duty cycle between -1.0 and 1.0
        duty = max(-1.0, min(1.0, duty))

        # Determine direction based on the sign of the duty cycle
        if duty < 0:
            self.direction.off()  # Ground the direction pin for reverse
            duty = -duty  # Make the duty cycle positive for PWM
        else:
            self.direction.on()  # Set the direction pin high for forward

        # Clamp the duty cycle to [0.0, 1.0] and invert for negative-edged PWM
        inverted_duty = 1.0 - duty
        self.motor.value = inverted_duty

        #self.get_logger().info(
            #f"Duty cycle set to {duty:.2f} (inverted: {inverted_duty:.2f}), direction: {'Reverse' if duty < 0 else 'Forward'}"
        #)
    
    # Set the PWM duty cycle [0.0, 1.0].
    # def set_duty_cycle(self, duty):
    #     """
    #     Set the PWM duty cycle. Adjusted so 0.0 is fully off and 1.0 is fully on.
    #     """
    #     duty = max(-1.0, min(1.0, duty))  # Clamp between -1 and 1
    #     inverted_duty = 1.0 - duty       # Invert the duty cycle
    #     self.motor.value = inverted_duty
    #     self.get_logger().info(f"Duty cycle set to {inverted_duty:.2f}")


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
        pulses_per_revolution = 360  #This PPR is a placeholder, find the actual value for  encoder
        speed = (self.pulse_count / pulses_per_revolution) / time_interval

        # Reset pulse count and time
        self.pulse_count = 0
        self.last_time = current_time

        return speed            

    def destroy_node(self):
        """
        Properly stop PWM when node is destroyed.
        """
        self.set_duty_cycle(1.0)
        super().destroy_node()



# Example usage if this script is run directly
def main(args=None):
    rclpy.init(args=args)

    # side motors are 12, 13 
    # main motor is pin 18
    node = MotorController(pwm_pin=18,
                           dir_pin=15,
                           channel_a_pin=0,
                           channel_b_pin=0,
                           initial_duty=0.1)
    
    time.sleep(1)  # Allow time for the motor to initialize
    node.set_duty_cycle(-0.1)  # Set initial duty cycle to 50%

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
