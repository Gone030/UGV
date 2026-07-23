#pragma once

#include "ugv_mcu/config.hpp"

namespace ugv_mcu {

class VelocityController {
 public:
  explicit VelocityController(config::PidGains gains);
  void reset();
  float update(float target_rad_s, float measured_rad_s, float dt_s);

 private:
  config::PidGains gains_;
  float integral_{};
  float previous_error_{};
  bool have_previous_{};
};

}  // namespace ugv_mcu
