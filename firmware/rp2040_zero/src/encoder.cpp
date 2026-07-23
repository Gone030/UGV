#include "ugv_mcu/encoder.hpp"

namespace ugv_mcu {
namespace { constexpr double kTwoPi = 6.28318530717958647692; }

void QuadratureDecoder::reset(std::uint8_t initial_state) {
  previous_state_ = initial_state & 0x3U;
  count_ = 0;
  invalid_transitions_ = 0;
}

int QuadratureDecoder::update(std::uint8_t ab_state) {
  static constexpr std::int8_t kTransition[16] = {
      0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  const std::uint8_t current = ab_state & 0x3U;
  const std::uint8_t index = static_cast<std::uint8_t>((previous_state_ << 2) | current);
  const int delta = kTransition[index];
  if (current != previous_state_ && delta == 0) ++invalid_transitions_;
  count_ += delta;
  previous_state_ = current;
  return delta;
}

Encoder::Encoder(EncoderBackend& backend, bool inverted) : backend_(backend), inverted_(inverted) {}
bool Encoder::initialize() { have_sample_ = false; velocity_rad_s_ = 0.0F; return backend_.initialize(); }
std::int64_t Encoder::count() const { const auto raw = backend_.count(); return inverted_ ? -raw : raw; }
float Encoder::update_velocity(std::uint64_t now_us, std::uint32_t cpr) {
  const auto current = count();
  if (!have_sample_ || cpr == 0 || now_us <= previous_time_us_) {
    previous_count_ = current; previous_time_us_ = now_us; have_sample_ = true;
    velocity_rad_s_ = 0.0F; return velocity_rad_s_;
  }
  const auto delta = current - previous_count_;
  const double dt = static_cast<double>(now_us - previous_time_us_) / 1.0e6;
  velocity_rad_s_ = static_cast<float>((static_cast<double>(delta) * kTwoPi) /
                                      (static_cast<double>(cpr) * dt));
  previous_count_ = current; previous_time_us_ = now_us;
  return velocity_rad_s_;
}
}  // namespace ugv_mcu
