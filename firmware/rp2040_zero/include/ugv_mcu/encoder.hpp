#pragma once

#include <cstdint>
#include "ugv_mcu/encoder_backend.hpp"

namespace ugv_mcu {

class Encoder {
 public:
  Encoder(EncoderBackend& backend, bool inverted);
  bool initialize();
  float update_velocity(std::uint64_t now_us, std::uint32_t counts_per_rev);
  std::int64_t count() const;
  float velocity_rad_s() const { return velocity_rad_s_; }

 private:
  EncoderBackend& backend_;
  bool inverted_{};
  bool have_sample_{};
  std::int64_t previous_count_{};
  std::uint64_t previous_time_us_{};
  float velocity_rad_s_{};
};

}  // namespace ugv_mcu
