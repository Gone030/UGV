#pragma once

#include <cstdint>

namespace ugv_mcu {

class CommandWatchdog {
 public:
  explicit CommandWatchdog(std::uint32_t timeout_ms);
  void arm(std::uint64_t now_ms);
  void disarm();
  bool expired(std::uint64_t now_ms) const;
  bool configured() const { return timeout_ms_ > 0; }

 private:
  std::uint32_t timeout_ms_{};
  std::uint64_t last_valid_command_ms_{};
  bool armed_{};
};

}  // namespace ugv_mcu
