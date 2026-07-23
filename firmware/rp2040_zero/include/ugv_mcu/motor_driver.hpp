#pragma once

#include <cstdint>
#include "ugv_mcu/config.hpp"

namespace ugv_mcu {

struct MotorCommandPlan {
  bool accepted{};
  bool direction_forward{};
  float duty{};
  float signed_output{};
};

float clamp_motor_command(float command);
MotorCommandPlan plan_motor_command(float command, bool enabled, bool inverted);

class MotorDriver {
 public:
  MotorDriver(config::MotorPins pins, bool inverted);
  MotorDriver(config::MotorPins pins, bool inverted, bool hardware_output_enabled,
              std::uint32_t pwm_frequency_hz, config::MotorStopMode stop_mode);
  bool initialize();
  void set_enabled(bool enabled);
  void set_output(float signed_command);
  bool try_set_output(float signed_command);
  void stop();
  bool hardware_active() const { return hardware_active_; }
  float commanded_output() const { return commanded_output_; }
  bool enabled() const { return enabled_; }

 private:
  config::MotorPins pins_;
  bool inverted_{};
  bool hardware_output_enabled_{};
  std::uint32_t pwm_frequency_hz_{};
  config::MotorStopMode stop_mode_{config::MotorStopMode::kTbd};
  bool initialized_{};
  bool enabled_{};
  bool hardware_active_{};
  std::uint16_t pwm_wrap_{65535};
  float commanded_output_{};
  void apply_stop_policy(config::MotorStopMode mode);
};

}  // namespace ugv_mcu
