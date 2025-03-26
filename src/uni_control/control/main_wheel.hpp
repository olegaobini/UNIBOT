#pragma once
#include "pid.hpp"

class main_wheel {
public:
  PID outer; // wheel speed PID
  PID middle; // tilt angle PID
  PID inner; // angular velocity PID

  main_wheel(float dt)
    : outer(1.0, 0.0, 0.0, dt, 20.0),
      middle(3.0, 0.2, 0.01, dt, 30.0),
      inner(5.0, 0.1, 0.01, dt, 255.0) {} // output could be mapped to PWM

  float update(float desired_velocity, float measured_velocity,
               float measured_angle, float measured_ang_velocity) {

    float vel_error = desired_velocity - measured_velocity;
    float target_angle = outer.update(vel_error);

    float angle_error = target_angle - measured_angle;
    float target_angular_velocity = middle.update(angle_error);

    float ang_vel_error = target_angular_velocity - measured_ang_velocity;
    float motor_command = inner.update(ang_vel_error);

    return motor_command;
  }

  void reset() {
    outer.reset();
    middle.reset();
    inner.reset();
  }
};
