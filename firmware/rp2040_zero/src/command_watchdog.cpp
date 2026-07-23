#include "ugv_mcu/command_watchdog.hpp"

namespace ugv_mcu {
CommandWatchdog::CommandWatchdog(std::uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}
void CommandWatchdog::arm(std::uint64_t now_ms) { last_valid_command_ms_ = now_ms; armed_ = timeout_ms_ > 0; }
void CommandWatchdog::disarm() { armed_ = false; }
bool CommandWatchdog::expired(std::uint64_t now_ms) const {
  return !configured() || !armed_ || now_ms - last_valid_command_ms_ >= timeout_ms_;
}
}  // namespace ugv_mcu
