#!/usr/bin/env python3 import time

import os
import yaml
import time
from ament_index_python.packages import get_package_share_directory

from cascaded_controller import CascadedController
from motor_control import MotorController
from mpu6050 import MPU6050

FREQUENCY = 1000 #PWM Frequency in Hz
DT = 0.1 #period of loop

def main():
    '''
    TODO which axis corresponds to roll and pitch? send to motor command
    - There's a lot of assumptions in the get_angle_pos function. 
      test which directions actually correspond to roll and pitch
    
    - Is there any way to calibrate the angles so we don't have to start
      the robot in a specific orientation?
    '''

    sensor = MPU6050() # Your IMU sensor class

    pins = load_yaml('header_pins.yaml')
    motor1 = MotorController(pwm_pin=pins['motor1']['pwm_pin'], 
                             frequency=FREQUENCY, 
                             initial_duty=1.0, 
                             speed_setpoint=0.0)
    motor2 = MotorController(pwm_pin=['motor2']['pwm_pin'], 
                             frequency=FREQUENCY, 
                             initial_duty=1.0, 
                             speed_setpoint=0.0) 
    motor_main = MotorController(pwm_pin=['motor3']['pwm_pin'], 
                                 frequency=FREQUENCY, 
                                 initial_duty=1.0, 
                                 speed_setpoint=0.0)  

    gains = load_yaml('pid_gains.yaml')
    reaction_wheel_cascade = CascadedController(
        **gains['reaction_wheel_gains'],
        dt=DT
    )

    main_wheel_cascade = CascadedController(
        **gains['main_wheel_gains'],
        dt=DT
    )

    # Main Control Loop
    try:
        #initialize angles - 
        prev_angles = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        while True:
            loop_start = time.time()

            #Ax, Ay, Az = sensor.read_accel()  
            Gx, Gy, Gz = sensor.read_gyro()  


            prev_angles = get_angle_pos(Gx, Gy, DT, prev_angles)
            roll_angle = prev_angles['roll']
            pitch_angle = prev_angles['pitch']

            motor1_speed = motor1.read_encoder_speed()
            motor2_speed = motor2.read_encoder_speed()
            reaction_wheel_speed =  (motor1_speed + motor2_speed) / 2.0 # get the average speed of both motors

            main_wheel_speed = motor_main.read_encoder_speed()


            # Updates the cascaded PID
            reaction_wheel_cmd = reaction_wheel_cascade.update(
                Gx, #THIS MIGHT BE WRONG, Check For Which Axis Correspons to Roll
                roll_angle,
                reaction_wheel_speed
            )

            main_wheel_cmd = main_wheel_cascade.update(
                Gy, #THIS MIGHT BE WRONG, Check For Which Axis Correspons to Pitch
                pitch_angle,
                main_wheel_speed
            )

            # Send command to motor(s)
            # "motor_command" might be in [-1.0, +1.0], so convert to duty cycle
            motor1.set_duty_cycle(reaction_wheel_cmd)
            motor2.set_duty_cycle(-reaction_wheel_cmd)
            motor_main.set_duty_cycle(main_wheel_cmd)

            # 2.4) Wait until next loop iteration
            elapsed = time.time() - loop_start
            sleep_time = DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're behind schedule, skip sleeping or log a warning
                print(f"[WARNING] Loop took {elapsed} seconds. Expected {DT} seconds.")
                pass

    except KeyboardInterrupt:
        # Graceful shutdown
        motor2.set_duty_cycle(0.0)
        motor2.set_duty_cycle(0.0)
        motor_main.set_duty_cycle(0.0)

def get_angle_pos(Gx, Gy, dt, prev_angles):
    """
    Integrate gyroscope data to estimate angular position.
    
    :param Gx: Angular velocity around the X-axis (degrees/sec)
    :param Gy: Angular velocity around the Y-axis (degrees/sec)
    :param Gz: Angular velocity around the Z-axis (degrees/sec)
    :param dt: Time step (seconds)
    :param prev_angles: Dictionary containing previous angles for roll, pitch, and yaw
    :return: Updated angles (roll, pitch). Yaw is not needed here
    """
    roll_angle = prev_angles['roll'] + Gx * dt
    pitch_angle = prev_angles['pitch'] + Gy * dt

    return {'roll': roll_angle, 'pitch': pitch_angle} 

def load_yaml(file_name):
    # Get the path to the config directory
    package_share_directory = get_package_share_directory('uni_control_py')
    config_file_path = os.path.join(package_share_directory, 'config', file_name)
    
    # Load the YAML file
    with open(config_file_path, 'r') as file:
        return yaml.safe_load(file)
    

if __name__ == "__main__":
    main()
