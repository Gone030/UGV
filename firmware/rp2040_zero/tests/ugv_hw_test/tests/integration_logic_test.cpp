#include "ugv_mcu/command_watchdog.hpp"
#include "ugv_mcu/motor_driver.hpp"
#include "ugv_mcu/system_state.hpp"
#include "ugv_mcu/velocity_controller.hpp"

#include <cassert>

int main() {
  using namespace ugv_mcu;
  CommandWatchdog watchdog(1000); // TEST_ONLY timeout; not production config.
  assert(watchdog.expired(0));
  watchdog.arm(100);
  assert(!watchdog.expired(1099));
  assert(watchdog.expired(1100));

  SystemStateMachine state;
  state.initialization_complete(true);
  assert(state.state() == SystemState::kDisabled);
  assert(state.request_enable(true, true));
  assert(state.state() == SystemState::kRunning);
  state.watchdog_stop();
  assert(state.state() == SystemState::kWatchdogStop);
  assert(state.request_enable(false, true));
  assert(state.state() == SystemState::kDisabled);

  const config::PidGains test_gains{0.25F, 0.0F, 0.0F, 1.0F};
  VelocityController controller(test_gains);
  const float output_with_error = controller.update(4.0F, 1.0F, 0.01F);
  controller.reset();
  const float output_at_target = controller.update(4.0F, 4.0F, 0.01F);
  assert(output_with_error > 0.0F);
  assert(output_at_target == 0.0F);

  assert(plan_motor_command(0.5F, false, false).duty == 0.0F);
  assert(!plan_motor_command(1.5F, true, false).accepted);
}
