#!/usr/bin/env python3 import time

import os
import yaml
import time
import rclpy
from ament_index_python.packages import get_package_share_directory

from cascaded_controller import CascadedController
from motor_control import MotorController
from mpu6050 import MPU6050

PWM_FREQUENCY = 1000 #Hz
DT = 0.005 #period of loop 

def main():
    '''
    TODO 
    - There's a lot of assumptions in the get_angle_pos function. 
      test which directions actually correspond to roll and pitch
    
    - Is there any way to calibrate the angles so we don't have to start
      the robot in a specific orientation?
    '''
    # Initialize ROS 2
    rclpy.init()

    sensor = MPU6050()
    pins = load_yaml('header_pins.yaml')
    motor1 = MotorController(pwm_pin=pins['motor1']['pwm_pin'], 
                             dir_pin=pins['motor1']['dir_pin'],
                             channel_a_pin=pins['motor1']['enc_a_pin'],
                             channel_b_pin=pins['motor1']['enc_b_pin'],
                             pwm_frequency=PWM_FREQUENCY)
    motor2 = MotorController(pwm_pin=pins['motor2']['pwm_pin'], 
                             dir_pin=pins['motor2']['dir_pin'],
                             channel_a_pin=pins['motor2']['enc_a_pin'],
                             channel_b_pin=pins['motor2']['enc_b_pin'],
                             pwm_frequency=PWM_FREQUENCY)
    motor_main = MotorController(pwm_pin=pins['motor3']['pwm_pin'], 
                                 dir_pin=pins['motor3']['dir_pin'],
                                 channel_a_pin=pins['motor3']['enc_a_pin'],
                                 channel_b_pin=pins['motor3']['enc_b_pin'],
                                 pwm_frequency=PWM_FREQUENCY)

    gains = load_yaml('pid_gains.yaml')
    print(f"kp_inner type: {type(gains['main_wheel_gains']['kp_inner'])}")  # Check type
    print(f"kp_inner type: {type(gains['main_wheel_gains']['kp_inner'])}")

    reaction_wheel_cascade = CascadedController(
        **gains['reaction_wheel_gains'],
        dt=DT, 
        wheel_speed_setpoint=1.0
    )

    main_wheel_cascade = CascadedController(
        **gains['main_wheel_gains'],
        dt=DT,
        wheel_speed_setpoint=1.0
    )

    # Initialize angles
    prev_angles = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    zero_angles = calibrate_imu(sensor)

    # Main Control Loop
    try:
        while True:
            loop_start = time.time()

            # Ax, Ay, Az = sensor.read_accel()  
            Gx, Gy, Gz = sensor.read_gyro()  

            prev_angles = get_angle_pos(Gx - zero_angles['roll'], Gy - zero_angles['pitch'], DT, prev_angles)
            #prev_angles = get_angle_pos(Gx, Gy, DT, prev_angles)
            roll_angle = prev_angles['roll']
            pitch_angle = prev_angles['pitch']

            # THIS IS TEMP
            main_wheel_speed = 0
            motor1_speed = 0
            motor2_speed = 0
            reaction_wheel_speed = (motor1_speed + motor2_speed) / 2.0  # Average speed of both motors

            # Updates the cascaded PID
            reaction_wheel_cmd = reaction_wheel_cascade.update(
                Gx,  # THIS MIGHT BE WRONG, Check For Which Axis Corresponds to Roll
                roll_angle,
                reaction_wheel_speed
            )

            main_wheel_cmd = main_wheel_cascade.update(
                Gy,  # THIS MIGHT BE WRONG, Check For Which Axis Corresponds to Pitch
                0*pitch_angle,
                main_wheel_speed
            )

            # Send command to motor(s)
            #motor_main.set_duty_cycle(-main_wheel_cmd)

            motor1.set_duty_cycle(reaction_wheel_cmd)
            motor2.set_duty_cycle(-reaction_wheel_cmd)
            print(f'cmd:{reaction_wheel_cmd:.2f}')

            #debugging text output
            #print(f'cmd:{main_wheel_cmd:.2f}, Gx:{Gx:.2f}, Gy:{Gy:.2f}, Gz:{Gz:.2f}, roll_angle:{roll_angle:.2f}, pitch_angle:{pitch_angle:.2f}')

            # Wait until next loop iteration
            elapsed = time.time() - loop_start
            sleep_time = DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                print(f"[WARNING] Loop took {elapsed} seconds. Expected {DT} seconds.")

    except KeyboardInterrupt:
        # Graceful shutdown
        motor1.set_duty_cycle(0.0)
        motor2.set_duty_cycle(0.0)
        motor_main.set_duty_cycle(0.0)
    finally:
        # Shutdown ROS 2
        motor1.destroy_node()
        motor2.destroy_node()
        motor_main.destroy_node()
        rclpy.shutdown()

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

    return {'roll': float(roll_angle), 'pitch': float(pitch_angle)} 

def load_yaml(file_name):
    # Get the path to the config directory
    #package_share_directory = get_package_share_directory('uni_control_py')
    #config_file_path = os.path.join(package_share_directory, 'config', file_name)
    config_file_path = os.path.join('/home/uni/unibot_ws/src/uni_control_py/config/', file_name)
    
    # Load the YAML file
    with open(config_file_path, 'r') as file:
        data = yaml.safe_load(file)
        return data
    
def calibrate_imu(sensor):
    """
    Calibrate the IMU by setting the current angles as the zero reference.

    :param sensor: The IMU sensor object (e.g., MPU6050)
    :return: A dictionary with the calibrated zero angles for roll and pitch
    """
    # Read the current gyro values
    Gy, Gx, Gz = sensor.read_gyro()

    # Initialize angles to zero
    zero_angles = {'roll': 0.0, 'pitch': 0.0}

    # Use the current gyro readings to set the zero reference
    zero_angles['roll'] = Gx
    zero_angles['pitch'] = Gy

    print(f"IMU calibrated. Zero angles set to: roll={zero_angles['roll']:.2f}, pitch={zero_angles['pitch']:.2f}")
    return zero_angles

if __name__ == "__main__":
    main()