#include <cmath>
#include <cstdio>
#include <cstdint>

#include "pico/stdlib.h"
#include "ugv_hw_test/command.hpp"
#include "ugv_mcu/command_watchdog.hpp"
#include "ugv_mcu/config.hpp"
#include "ugv_mcu/encoder.hpp"
#include "ugv_mcu/encoder_pio.hpp"
#include "ugv_mcu/motor_driver.hpp"
#include "ugv_mcu/protocol.hpp"
#include "ugv_mcu/system_state.hpp"
#include "ugv_mcu/velocity_controller.hpp"

namespace {
// TEST_ONLY values exercise control flow; they are not production calibration.
constexpr std::uint32_t kTestLoopPeriodUs = 10'000;
constexpr std::uint32_t kTestStatusPeriodMs = 250;
constexpr std::uint32_t kTestWatchdogTimeoutMs = 1'000;
constexpr std::uint32_t kTestSimulationCountsPerRev = 4'096;
constexpr ugv_mcu::config::EncoderPins kTestLeftEncoderPins{2, 3};
constexpr ugv_mcu::config::EncoderPins kTestRightEncoderPins{4, 5};
constexpr ugv_mcu::config::PidGains kTestSimulationPid{0.25F, 0.05F, 0.0F, 1.0F};
constexpr double kTwoPi = 6.28318530717958647692;

class FakeEncoderBackend final : public ugv_mcu::EncoderBackend {
 public:
  bool initialize() override { count_ = 0; residual_ = 0.0; return true; }
  std::int64_t count() const override { return count_; }
  bool hardware_active() const override { return false; }
  void set_speed(float rad_s) { speed_rad_s_ = rad_s; }
  void advance(float dt_s) {
    residual_ += static_cast<double>(speed_rad_s_) * dt_s *
                 static_cast<double>(kTestSimulationCountsPerRev) / kTwoPi;
    const auto whole = static_cast<std::int64_t>(residual_);
    count_ += whole;
    residual_ -= static_cast<double>(whole);
  }
 private:
  float speed_rad_s_{};
  std::int64_t count_{};
  double residual_{};
};

const char* state_name(ugv_mcu::SystemState state) {
  using ugv_mcu::SystemState;
  switch (state) {
    case SystemState::kBoot: return "BOOT";
    case SystemState::kDisabled: return "DISABLED";
    case SystemState::kReady: return "READY";
    case SystemState::kRunning: return "RUNNING";
    case SystemState::kWatchdogStop: return "WATCHDOG_STOP";
    case SystemState::kFault: return "FAULT";
  }
  return "UNKNOWN";
}
}  // namespace

int main() {
  using namespace ugv_mcu;
  using namespace ugv_hw_test;
  stdio_init_all();

  MotorDriver left_motor(config::kLeftMotorPins, config::kLeftMotorInverted);
  MotorDriver right_motor(config::kRightMotorPins, config::kRightMotorInverted);
  left_motor.initialize(); right_motor.initialize();
  left_motor.stop(); right_motor.stop();

  EncoderPioBackend left_pio(kTestLeftEncoderPins, pio0);
  EncoderPioBackend right_pio(kTestRightEncoderPins, pio1);
  Encoder left_encoder(left_pio, config::kLeftEncoderInverted);
  Encoder right_encoder(right_pio, config::kRightEncoderInverted);
  left_encoder.initialize(); right_encoder.initialize();

  FakeEncoderBackend left_fake, right_fake;
  Encoder left_sim_encoder(left_fake, false), right_sim_encoder(right_fake, false);
  left_sim_encoder.initialize(); right_sim_encoder.initialize();
  VelocityController left_controller(kTestSimulationPid), right_controller(kTestSimulationPid);
  CommandWatchdog watchdog(kTestWatchdogTimeoutMs);
  SystemStateMachine state;
  state.initialization_complete(true); // Test harness is ready; hardware interlocks remain independent.

  CommandReceiver receiver;
  Mode mode = Mode::kEncoder;
  bool enabled = false;
  bool watchdog_active = false;
  float left_request = 0.0F, right_request = 0.0F;
  float left_feedback = 0.0F, right_feedback = 0.0F;
  float left_output = 0.0F, right_output = 0.0F;
  std::uint32_t sequence = 0;
  std::uint64_t next_loop_us = time_us_64();
  std::uint64_t next_status_us = next_loop_us;

  auto disable = [&]() {
    enabled = false; left_request = right_request = 0.0F;
    left_output = right_output = 0.0F;
    left_motor.set_enabled(false); right_motor.set_enabled(false);
    left_motor.stop(); right_motor.stop();
    left_controller.reset(); right_controller.reset(); watchdog.disarm();
    state.request_enable(false, true);
  };

  auto print_status = [&]() {
    StatusPacket packet{};
    packet.sequence = sequence;
    packet.enabled = enabled;
    packet.watchdog_active = watchdog_active;
    packet.left_target_rad_s = left_request;
    packet.right_target_rad_s = right_request;
    packet.left_measured_rad_s = left_feedback;
    packet.right_measured_rad_s = right_feedback;
    packet.left_encoder_count = mode == Mode::kSim ? left_sim_encoder.count() : left_encoder.count();
    packet.right_encoder_count = mode == Mode::kSim ? right_sim_encoder.count() : right_encoder.count();
    packet.left_pwm = left_output; packet.right_pwm = right_output;
    char line[256];
    const auto length = serialize_status(packet, line, sizeof(line));
    if (length) std::fwrite(line, 1, length, stdout);
    std::printf("TEST,%s,enabled=%u,state=%s,watchdog=%u,motor_hw=%u:%u,encoder_hw=%u:%u\n",
                mode_name(mode), enabled, state_name(state.state()), watchdog_active,
                left_motor.hardware_active(), right_motor.hardware_active(),
                left_pio.hardware_active(), right_pio.hardware_active());
  };

  std::printf("TEST,BOOT,enabled=0,hardware_output=%u\n", config::kHardwareOutputEnabled);

  while (true) {
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
      Command command{};
      const auto result = receiver.push(static_cast<char>(ch), command);
      if (!result.line_complete) continue;
      if (!result.accepted) { std::printf("ERROR,%s\n", error_name(result.error)); continue; }
      ++sequence;
      switch (command.type) {
        case CommandType::kStatus:
          print_status();
          break;
        case CommandType::kSetMode:
          disable(); watchdog_active = false; mode = command.mode;
          std::printf("TEST,%s,MODE_SET\n", mode_name(mode));
          break;
        case CommandType::kEnable:
          if (!command.enable) { disable(); watchdog_active = false; std::printf("TEST,%s,DISABLED\n", mode_name(mode)); break; }
          enabled = state.request_enable(true, true);
          watchdog_active = false;
          watchdog.arm(to_ms_since_boot(get_absolute_time()));
          left_motor.set_enabled(mode == Mode::kMotor && enabled);
          right_motor.set_enabled(mode == Mode::kMotor && enabled);
          std::printf("TEST,%s,ENABLE,%u,physical_output=%u\n", mode_name(mode), enabled,
                      left_motor.hardware_active() && right_motor.hardware_active());
          break;
        case CommandType::kMotorLeft:
        case CommandType::kMotorRight:
        case CommandType::kMotorBoth: {
          if (!enabled || mode == Mode::kEncoder) { std::printf("ERROR,NOT_ENABLED_OR_WRONG_MODE\n"); break; }
          if (command.type != CommandType::kMotorRight) left_request = command.left;
          if (command.type == CommandType::kMotorRight) right_request = command.left;
          if (command.type == CommandType::kMotorBoth) right_request = command.right;
          if (mode == Mode::kMotor) {
            const auto left_plan = plan_motor_command(left_request, true, config::kLeftMotorInverted);
            const auto right_plan = plan_motor_command(right_request, true, config::kRightMotorInverted);
            if (!left_plan.accepted || !right_plan.accepted) { disable(); std::printf("ERROR,MOTOR_RANGE\n"); break; }
            left_output = left_plan.signed_output; right_output = right_plan.signed_output;
            left_motor.try_set_output(left_request); right_motor.try_set_output(right_request);
            std::printf("TEST,MOTOR,L,dir=%u,duty=%.6g,R,dir=%u,duty=%.6g,physical=%u\n",
                        left_plan.direction_forward, left_plan.duty,
                        right_plan.direction_forward, right_plan.duty,
                        left_motor.hardware_active() && right_motor.hardware_active());
          }
          watchdog.arm(to_ms_since_boot(get_absolute_time())); watchdog_active = false;
          break;
        }
        case CommandType::kSimSpeed:
          if (mode != Mode::kSim) { std::printf("ERROR,WRONG_MODE\n"); break; }
          left_fake.set_speed(command.left); right_fake.set_speed(command.right);
          std::printf("TEST,SIM,FEEDBACK_SET,%.6g,%.6g\n", command.left, command.right);
          break;
      }
    }

    const std::uint64_t now_us = time_us_64();
    if (enabled && watchdog.expired(now_us / 1000U)) {
      disable(); watchdog_active = true; state.watchdog_stop();
      std::printf("ERROR,WATCHDOG_TIMEOUT\n");
    }

    if (now_us >= next_loop_us) {
      constexpr float dt = static_cast<float>(kTestLoopPeriodUs) / 1.0e6F;
      if (mode == Mode::kEncoder) {
        left_feedback = left_encoder.update_velocity(now_us, config::kEncoderCountsPerOutputRevolution);
        right_feedback = right_encoder.update_velocity(now_us, config::kEncoderCountsPerOutputRevolution);
        left_output = right_output = 0.0F;
        left_motor.stop(); right_motor.stop();
      } else if (mode == Mode::kSim) {
        left_fake.advance(dt); right_fake.advance(dt);
        left_feedback = left_sim_encoder.update_velocity(now_us, kTestSimulationCountsPerRev);
        right_feedback = right_sim_encoder.update_velocity(now_us, kTestSimulationCountsPerRev);
        left_output = enabled ? left_controller.update(left_request, left_feedback, dt) : 0.0F;
        right_output = enabled ? right_controller.update(right_request, right_feedback, dt) : 0.0F;
      }
      next_loop_us = now_us + kTestLoopPeriodUs;
    }

    if (now_us >= next_status_us) {
      print_status();
      next_status_us = now_us + static_cast<std::uint64_t>(kTestStatusPeriodMs) * 1000U;
    }
    tight_loop_contents();
  }
}
