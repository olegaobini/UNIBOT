#!/usr/bin/env python3

import os
import time
import smbus
from ament_index_python.packages import get_package_share_directory

# These numbers convert raw accelerometer and gyroscope data to physical units
ACCEL_SCALE = 16384.0 # ±2g → Scale Factor = 16384.0 (LSB/g)
GYRO_SCALE = 131.0 # ±250 dps → Scale Factor = 131.0 (LSB/(degrees/sec))

class MPU6050:
    # MPU6050 registers
    MPU6050_ADDR     = 0x68
    PWR_MGMT_1       = 0x6B

    #accelerometer registers
    ACCEL_XOUT     = 0x3D
    ACCEL_YOUT     = 0x3B
    ACCEL_ZOUT     = 0x3F

    #gyroscope registers
    GYRO_XOUT      = 0x45
    GYRO_YOUT      = 0x43
    GYRO_ZOUT      = 0x47

    # Configuration registers 

    def __init__(self, bus_num=1, address=MPU6050_ADDR):
        self.bus = smbus.SMBus(bus_num)
        self.address = address
        self.DLPF_CFG = 6 
        self.CONFIG_REG = 0x1A

        try:
            time.sleep(0.1)  # <-- Add slight delay before first write
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
            self.bus.write_byte_data(self.address, self.CONFIG_REG, self.DLPF_CFG)
            time.sleep(0.1)  # <-- Let the sensor wake up
        except OSError as e:
            print(f"[MPU6050 ERROR] Could not initialize MPU6050: {e}")
            raise

    def _read_raw_data(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        return value - 65536 if value > 32767 else value

    def read_accel(self):
        acc_x = self._read_raw_data(self.ACCEL_XOUT)
        acc_y = self._read_raw_data(self.ACCEL_YOUT)
        acc_z = self._read_raw_data(self.ACCEL_ZOUT)

        Ax = acc_x / ACCEL_SCALE # 16384 LSB/g
        Ay = acc_y / ACCEL_SCALE
        Az = acc_z / ACCEL_SCALE

        return Ax, Ay, Az

    def read_gyro(self):
        gyro_x = self._read_raw_data(self.GYRO_XOUT)
        gyro_y = self._read_raw_data(self.GYRO_YOUT)
        gyro_z = self._read_raw_data(self.GYRO_ZOUT)

        roundNum = 0
        Gx = round((gyro_x / GYRO_SCALE), roundNum) # 131 LSB/(degrees/sec)
        Gy = round((gyro_y / GYRO_SCALE), roundNum)
        Gz = round((gyro_z / GYRO_SCALE), roundNum)

        return Gx, Gy, Gz

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

# Standalone test
if __name__ == "__main__":
    sensor = MPU6050()
    print("Reading MPU6050 data...")
    while True:
        Ax, Ay, Az = sensor.read_accel()
        print(f"Ax: {Ax:.2f} g, Ay: {Ay:.2f} g, Az: {Az:.2f} g")

        Gx, Gy, Gz = sensor.read_gyro()
        print(f"Gx: {Gx:.2f} deg/s, Gy: {Gy:.2f} deg/s, Gz: {Gz:.2f} deg/s")
        time.sleep(0.5)