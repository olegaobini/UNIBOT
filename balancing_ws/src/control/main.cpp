#include "main_wheel.hpp"

// Setup
main_wheel balance(0.001f); // 1 kHz loop

void setup() {
  Serial.begin(115200);
  // setup IMU, encoders, PWM
}

void loop() {
  static uint32_t last = micros();
  if (micros() - last >= 1000) {
    last += 1000;

    float wheel_speed = read_encoder_speed();
    float pitch = imu_get_pitch();
    float angular_velocity = imu_get_gyro_pitch();

    float target_velocity = get_target_velocity(); // from ROS/RC

    float pwm_output = balance.update(target_velocity, wheel_speed, pitch, angular_velocity);

    apply_motor_pwm(pwm_output);
  }
}
