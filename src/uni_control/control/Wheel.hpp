#pragma once
#include "PID.hpp"

class Wheel {
public:
  PID outer; // wheel speed PID
  PID middle; // tilt angle PID
  PID inner; // angular velocity PID

  float kp_inner, ki_inner, kd_inner;
  float kp_outer, ki_outer, kd_outer;
  float kp_middle, ki_middle, kd_middle;

  float inner_limit;
  float outer_limit;
  float middle_limit;

  // kp(p), ki(i), kd(d), dt(dt_sec), output_limit(limit) {}
  Wheel(float dt)
    : outer(kp_outer,ki_outer ,kd_outer, dt, outer_limit),
      inner(kp_inner,ki_inner ,kd_inner, dt, inner_limit),
      middle(kp_middle,ki_middle,kd_middle, dt, middle_limit) {} // output could be mapped to PWM

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
