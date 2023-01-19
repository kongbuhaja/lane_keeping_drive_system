#include "alcoholdriving/pid_controller.h"
#include "ros/ros.h"

namespace alcoholdriving {
PID::PID(float p_gain, float i_gain, float d_gain)
    : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain) {
  p_error_ = 0.0f;
  i_error_ = 0.0f;
  d_error_ = 0.0f;
}

float PID::getControlOutput(float error) {
  float float_type_error = (float)error;
  d_error_ = float_type_error - p_error_;
  p_error_ = float_type_error;
  i_error_ += float_type_error;
  return p_gain_ * p_error_ + i_gain_ * i_error_+ d_gain_ * d_error_;
}
}  // namespace alcoholdriving