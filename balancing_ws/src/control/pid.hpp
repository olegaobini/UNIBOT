#pragma once

class PID {
public:
  float kp, ki, kd;
  float integral = 0.0f;
  float prev_error = 0.0f;
  float dt, output_limit;

  PID(float p, float i, float d, float dt_sec, float limit)
    : kp(p), ki(i), kd(d), dt(dt_sec), output_limit(limit) {}

  float update(float error) {
    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    float output = kp * error + ki * integral + kd * derivative;

    if (output > output_limit) output = output_limit;
    if (output < -output_limit) output = -output_limit;

    return output;
  }

  void reset() {
    integral = 0.0f;
    prev_error = 0.0f;
  }
};
