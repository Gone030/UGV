#include "ugv_mcu/system_state.hpp"

namespace ugv_mcu {
void SystemStateMachine::initialization_complete(bool valid) { state_ = valid ? SystemState::kDisabled : SystemState::kFault; }
bool SystemStateMachine::request_enable(bool enable, bool valid) {
  if (!enable) { state_ = SystemState::kDisabled; return true; }
  if (!valid || state_ == SystemState::kFault) return false;
  state_ = SystemState::kRunning; return true;
}
void SystemStateMachine::watchdog_stop() { if (state_ != SystemState::kFault) state_ = SystemState::kWatchdogStop; }
void SystemStateMachine::fault() { state_ = SystemState::kFault; }
}  // namespace ugv_mcu
