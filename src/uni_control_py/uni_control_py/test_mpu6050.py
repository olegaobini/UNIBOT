import smbus
import time

# Bus 1 for modern Raspberry Pi
bus = smbus.SMBus(1)
MPU6050_ADDR = 0x68

# MPU6050 registers
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

def read_raw_data(reg):
    # Read two bytes, combine them
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low  = bus.read_byte_data(MPU6050_ADDR, reg+1)
    value = (high << 8) | low

    # Convert to signed 16-bit
    if value > 32767:
        value -= 65536
    return value

def initialize():
    # Wake the MPU6050 (clear sleep bit)
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def main():
    initialize()
    while True:
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        # Assume default +/- 2g range; 1g = 16384 LSB
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0

        print(f"Ax: {Ax:.3f} g, Ay: {Ay:.3f} g, Az: {Az:.3f} g")
        time.sleep(0.5)

if __name__ == "__main__":
    main()
