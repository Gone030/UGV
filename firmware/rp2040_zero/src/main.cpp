#include <cstdio>

#include "pico/stdlib.h"
#include "ugv_mcu/command_watchdog.hpp"
#include "ugv_mcu/config.hpp"
#include "ugv_mcu/encoder.hpp"
#include "ugv_mcu/encoder_pio.hpp"
#include "ugv_mcu/motor_driver.hpp"
#include "ugv_mcu/protocol.hpp"
#include "ugv_mcu/system_state.hpp"
#include "ugv_mcu/velocity_controller.hpp"

int main() {
  using namespace ugv_mcu;
  stdio_init_all(); // USB CDC enabled by CMake; UART stdio disabled.

  MotorDriver left_motor(config::kLeftMotorPins, config::kLeftMotorInverted);
  MotorDriver right_motor(config::kRightMotorPins, config::kRightMotorInverted);
  left_motor.initialize(); right_motor.initialize();
  left_motor.stop(); right_motor.stop(); // Mandatory boot-safe output.

  EncoderPioBackend left_backend(config::kLeftEncoderPins, pio0);
  EncoderPioBackend right_backend(config::kRightEncoderPins, pio1);
  Encoder left_encoder(left_backend, config::kLeftEncoderInverted);
  Encoder right_encoder(right_backend, config::kRightEncoderInverted);
  left_encoder.initialize(); right_encoder.initialize();
  VelocityController left_pid(config::kLeftPid), right_pid(config::kRightPid);
  CommandWatchdog watchdog(config::kWatchdogTimeoutMs);
  LineProtocolReceiver receiver;
  SystemStateMachine state;
  const bool drive_ready = config::drive_configuration_complete() &&
                           left_motor.hardware_active() && right_motor.hardware_active() &&
                           left_backend.hardware_active() && right_backend.hardware_active();
  state.initialization_complete(drive_ready);

  CommandPacket command{};
  StatusPacket status{};
  std::uint64_t next_control_us = time_us_64();
  std::uint64_t next_status_us = next_control_us;

  while (true) {
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
      CommandPacket candidate{};
      const auto result = receiver.push(static_cast<char>(ch), config::kMaxTargetRadS, candidate);
      if (result.accepted && state.request_enable(candidate.enable, drive_ready)) {
        command = candidate; // Invalid packets never replace the last valid command.
        watchdog.arm(to_ms_since_boot(get_absolute_time()));
      }
    }

    const std::uint64_t now_us = time_us_64();
    const bool timed_out = watchdog.expired(now_us / 1000U);
    if (timed_out) {
      command.enable = false; command.left_target_rad_s = 0.0F; command.right_target_rad_s = 0.0F;
      left_motor.set_enabled(false); right_motor.set_enabled(false);
      left_pid.reset(); right_pid.reset(); state.watchdog_stop();
    }

    if (config::kControlPeriodUs > 0 && now_us >= next_control_us) {
      const float dt = static_cast<float>(config::kControlPeriodUs) / 1.0e6F;
      const float left_speed = left_encoder.update_velocity(now_us, config::kEncoderCountsPerOutputRevolution);
      const float right_speed = right_encoder.update_velocity(now_us, config::kEncoderCountsPerOutputRevolution);
      const bool enabled = command.enable && !timed_out && drive_ready;
      left_motor.set_enabled(enabled); right_motor.set_enabled(enabled);
      left_motor.set_output(enabled ? left_pid.update(command.left_target_rad_s, left_speed, dt) : 0.0F);
      right_motor.set_output(enabled ? right_pid.update(command.right_target_rad_s, right_speed, dt) : 0.0F);
      next_control_us = now_us + config::kControlPeriodUs;
    }

    if (config::kStatusPeriodMs > 0 && now_us >= next_status_us) {
      status.sequence = command.sequence; status.enabled = command.enable && !timed_out && drive_ready;
      status.watchdog_active = timed_out; status.fault_flags = drive_ready ? 0U : 1U;
      status.left_target_rad_s = command.left_target_rad_s; status.right_target_rad_s = command.right_target_rad_s;
      status.left_measured_rad_s = left_encoder.velocity_rad_s(); status.right_measured_rad_s = right_encoder.velocity_rad_s();
      status.left_encoder_count = left_encoder.count(); status.right_encoder_count = right_encoder.count();
      status.left_pwm = left_motor.commanded_output(); status.right_pwm = right_motor.commanded_output();
      char line[256]; const auto length = serialize_status(status, line, sizeof(line), true);
      if (length) std::fwrite(line, 1, length, stdout);
      next_status_us = now_us + static_cast<std::uint64_t>(config::kStatusPeriodMs) * 1000U;
    }
    tight_loop_contents();
  }
}
