#pragma once

#include <cstdint>

namespace ugv_mcu {

class EncoderBackend {
 public:
  virtual ~EncoderBackend() = default;
  virtual bool initialize() = 0;
  virtual std::int64_t count() const = 0;
  virtual bool hardware_active() const = 0;
};

class StubEncoderBackend final : public EncoderBackend {
 public:
  bool initialize() override { return true; }
  std::int64_t count() const override { return 0; }
  bool hardware_active() const override { return false; }
};

// Hardware-independent x4 Gray-code decoder used by tests and future IRQ backends.
class QuadratureDecoder {
 public:
  void reset(std::uint8_t initial_state = 0);
  int update(std::uint8_t ab_state);
  std::int64_t count() const { return count_; }
  std::uint32_t invalid_transition_count() const { return invalid_transitions_; }

 private:
  std::uint8_t previous_state_{};
  std::int64_t count_{};
  std::uint32_t invalid_transitions_{};
};

}  // namespace ugv_mcu
