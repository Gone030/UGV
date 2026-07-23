#include "ugv_mcu/velocity_controller.hpp"

#include <algorithm>
#include <cmath>

namespace ugv_mcu {
VelocityController::VelocityController(config::PidGains gains) : gains_(gains) {}
void VelocityController::reset() { integral_ = 0.0F; previous_error_ = 0.0F; have_previous_ = false; }
float VelocityController::update(float target, float measured, float dt) {
  if (!std::isfinite(target) || !std::isfinite(measured) || !(dt > 0.0F)) { reset(); return 0.0F; }
  if (target == 0.0F) { reset(); return 0.0F; }
  const float error = target - measured;
  integral_ += error * dt;
  const float limit = std::max(0.0F, gains_.integral_limit);
  integral_ = std::clamp(integral_, -limit, limit); // Anti-windup clamp.
  const float derivative = have_previous_ ? (error - previous_error_) / dt : 0.0F;
  previous_error_ = error; have_previous_ = true;
  const float raw = gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
  return std::clamp(raw, -1.0F, 1.0F);
}
}  // namespace ugv_mcu
