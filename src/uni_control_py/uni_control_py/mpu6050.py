#!/usr/bin/env python3

import smbus
import time

class MPU6050:
    # Constants
    MPU6050_ADDR     = 0x68
    PWR_MGMT_1       = 0x6B
    ACCEL_XOUT_H     = 0x3B
    ACCEL_YOUT_H     = 0x3D
    ACCEL_ZOUT_H     = 0x3F

    def __init__(self, bus_num=1, address=MPU6050_ADDR):
        self.bus = smbus.SMBus(bus_num)
        self.address = address

        try:
            time.sleep(0.1)  # <-- Add slight delay before first write
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
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
        acc_x = self._read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self._read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self._read_raw_data(self.ACCEL_ZOUT_H)

        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0

        return Ax, Ay, Az

# Standalone test
if __name__ == "__main__":
    sensor = MPU6050()
    print("Reading MPU6050 data...")
    while True:
        Ax, Ay, Az = sensor.read_accel()
        print(f"Ax: {Ax:.2f} g, Ay: {Ay:.2f} g, Az: {Az:.2f} g")
        time.sleep(0.5)

