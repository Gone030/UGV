#pragma once

namespace ugv_mcu {

enum class SystemState { kBoot, kDisabled, kReady, kRunning, kWatchdogStop, kFault };

class SystemStateMachine {
 public:
  SystemState state() const { return state_; }
  void initialization_complete(bool configuration_valid);
  bool request_enable(bool enable, bool configuration_valid);
  void watchdog_stop();
  void fault();

 private:
  SystemState state_{SystemState::kBoot};
};

}  // namespace ugv_mcu
