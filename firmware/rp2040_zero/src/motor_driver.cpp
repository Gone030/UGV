#include "ugv_mcu/motor_driver.hpp"

#include <algorithm>
#include <cmath>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

namespace ugv_mcu {
MotorDriver::MotorDriver(config::MotorPins pins, bool inverted) : pins_(pins), inverted_(inverted) {}

bool MotorDriver::initialize() {
  initialized_ = true; enabled_ = false; commanded_output_ = 0.0F;
  hardware_active_ = config::kHardwareOutputEnabled && config::gpio_valid(pins_.pwm) &&
                     config::gpio_valid(pins_.dir) && config::kPwmFrequencyHz > 0;
  if (!hardware_active_) return false;  // Deliberate safe stub; no GPIO touched.
  gpio_init(static_cast<unsigned>(pins_.dir));
  gpio_set_dir(static_cast<unsigned>(pins_.dir), GPIO_OUT);
  gpio_put(static_cast<unsigned>(pins_.dir), false);
  gpio_set_function(static_cast<unsigned>(pins_.pwm), GPIO_FUNC_PWM);
  const unsigned slice = pwm_gpio_to_slice_num(static_cast<unsigned>(pins_.pwm));
  const float system_hz = static_cast<float>(clock_get_hz(clk_sys));
  float divider = system_hz / (static_cast<float>(config::kPwmFrequencyHz) * 65536.0F);
  divider = std::clamp(divider, 1.0F, 255.0F);
  const float wrap = system_hz / (static_cast<float>(config::kPwmFrequencyHz) * divider) - 1.0F;
  if (!(wrap >= 1.0F && wrap <= 65535.0F)) { hardware_active_ = false; return true; }
  pwm_wrap_ = static_cast<std::uint16_t>(wrap);
  pwm_set_clkdiv(slice, divider);
  pwm_set_wrap(slice, pwm_wrap_);
  pwm_set_gpio_level(static_cast<unsigned>(pins_.pwm), 0);
  pwm_set_enabled(slice, true);
  return true;
}

void MotorDriver::set_enabled(bool enabled) { enabled_ = enabled && initialized_ && hardware_active_; if (!enabled_) stop(); }

void MotorDriver::set_output(float command) { static_cast<void>(try_set_output(command)); }

bool MotorDriver::try_set_output(float command) {
  const auto plan = plan_motor_command(command, enabled_ && hardware_active_, inverted_);
  if (!plan.accepted || !enabled_ || !hardware_active_) { stop(); return false; }
  commanded_output_ = plan.signed_output;
  gpio_put(static_cast<unsigned>(pins_.dir), plan.direction_forward);
  pwm_set_gpio_level(static_cast<unsigned>(pins_.pwm),
                     static_cast<std::uint16_t>(plan.duty * pwm_wrap_));
  return true;
}

void MotorDriver::stop() {
  commanded_output_ = 0.0F;
  if (hardware_active_) pwm_set_gpio_level(static_cast<unsigned>(pins_.pwm), 0);
  apply_stop_policy(config::kMotorStopMode);
}

void MotorDriver::apply_stop_policy(config::MotorStopMode mode) {
  // Coast/brake electrical semantics are TBD. Until then every mode only enforces duty zero.
  static_cast<void>(mode);
}
}  // namespace ugv_mcu
