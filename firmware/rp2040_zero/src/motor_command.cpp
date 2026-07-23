#include "ugv_mcu/motor_driver.hpp"

#include <algorithm>
#include <cmath>

namespace ugv_mcu {
float clamp_motor_command(float command) {
  if (!std::isfinite(command)) return 0.0F;
  return std::clamp(command, -1.0F, 1.0F);
}

MotorCommandPlan plan_motor_command(float command, bool enabled, bool inverted) {
  if (!std::isfinite(command) || command < -1.0F || command > 1.0F)
    return {false, false, 0.0F, 0.0F};
  if (!enabled) return {true, false, 0.0F, 0.0F};
  bool forward = command >= 0.0F;
  if (inverted) forward = !forward;
  return {true, forward, std::fabs(command), command};
}
}  // namespace ugv_mcu
