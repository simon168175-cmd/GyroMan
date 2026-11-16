// Minimal 1D angle/bias Kalman filter for MPU6050 pitch estimation.

#pragma once

class Kalman1D {
 public:
  void setAngle(float angle) { angle_ = angle; }

  float update(float measuredAngle, float measuredRate, float dt) {
    rate_ = measuredRate - bias_;
    angle_ += dt * rate_;

    p00_ += dt * (dt * p11_ - p01_ - p10_ + qAngle_);
    p01_ -= dt * p11_;
    p10_ -= dt * p11_;
    p11_ += qBias_ * dt;

    const float innovation = measuredAngle - angle_;
    const float s = p00_ + rMeasure_;
    const float k0 = p00_ / s;
    const float k1 = p10_ / s;

    angle_ += k0 * innovation;
    bias_ += k1 * innovation;

    const float oldP00 = p00_;
    const float oldP01 = p01_;
    p00_ -= k0 * oldP00;
    p01_ -= k0 * oldP01;
    p10_ -= k1 * oldP00;
    p11_ -= k1 * oldP01;
    return angle_;
  }

  float rate() const { return rate_; }

 private:
  float qAngle_ = 0.001f;
  float qBias_ = 0.003f;
  float rMeasure_ = 0.03f;
  float angle_ = 0.0f;
  float bias_ = 0.0f;
  float rate_ = 0.0f;
  float p00_ = 0.0f;
  float p01_ = 0.0f;
  float p10_ = 0.0f;
  float p11_ = 0.0f;
};
