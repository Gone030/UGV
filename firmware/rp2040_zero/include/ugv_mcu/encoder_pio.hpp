#pragma once

#include <cstdint>

#include "hardware/pio.h"
#include "ugv_mcu/config.hpp"
#include "ugv_mcu/encoder_backend.hpp"

namespace ugv_mcu {

class EncoderPioBackend final : public EncoderBackend {
 public:
  explicit EncoderPioBackend(config::EncoderPins pins, PIO pio = pio0);
  bool initialize() override;
  std::int64_t count() const override;
  bool hardware_active() const override { return hardware_active_; }

 private:
  config::EncoderPins pins_;
  PIO pio_;
  int state_machine_{-1};
  unsigned program_offset_{};
  bool hardware_active_{};
  mutable bool have_raw_count_{};
  mutable std::uint32_t previous_raw_count_{};
  mutable std::int64_t extended_count_{};
};

}  // namespace ugv_mcu
