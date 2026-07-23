#pragma once

#include <cstdint>

namespace ugv_mcu::config {

constexpr int kGpioTbd = -1; // 핀설정 아직 안되어있음

struct MotorPins { int pwm; int dir; };
struct EncoderPins { int a; int b; };
struct PidGains { float kp; float ki; float kd; float integral_limit; };
enum class MotorStopMode { kTbd, kCoast, kBrake };

// Hardware mapping: all values intentionally TBD.
constexpr MotorPins kLeftMotorPins{kGpioTbd, kGpioTbd};
constexpr MotorPins kRightMotorPins{kGpioTbd, kGpioTbd};
constexpr EncoderPins kLeftEncoderPins{kGpioTbd, kGpioTbd};
constexpr EncoderPins kRightEncoderPins{kGpioTbd, kGpioTbd};

// Explicit master interlock. Keep false until wiring and direction are verified.
constexpr bool kHardwareOutputEnabled = false;
constexpr bool kLeftMotorInverted = false;    // TBD
constexpr bool kRightMotorInverted = false;   // TBD
constexpr bool kLeftEncoderInverted = false;  // TBD
constexpr bool kRightEncoderInverted = false; // TBD
constexpr MotorStopMode kMotorStopMode = MotorStopMode::kTbd;

// Zero means unconfigured/TBD and prevents closed-loop drive activation.
constexpr std::uint32_t kEncoderCountsPerOutputRevolution = 0;
constexpr std::uint32_t kPwmFrequencyHz = 0;
constexpr std::uint32_t kControlPeriodUs = 0;
constexpr std::uint32_t kStatusPeriodMs = 0;
constexpr std::uint32_t kWatchdogTimeoutMs = 0;
constexpr float kMaxTargetRadS = 0.0F;
constexpr float kAccelerationLimitRadS2 = 0.0F; // TBD; not yet applied.
constexpr PidGains kLeftPid{0.0F, 0.0F, 0.0F, 0.0F};
constexpr PidGains kRightPid{0.0F, 0.0F, 0.0F, 0.0F};

constexpr bool gpio_valid(int gpio) { return gpio >= 0 && gpio <= 29; }
constexpr bool encoder_pins_valid(EncoderPins pins) {
  return gpio_valid(pins.a) && gpio_valid(pins.b) && pins.b == pins.a + 1;
}
constexpr bool drive_configuration_complete() {
  return kHardwareOutputEnabled && gpio_valid(kLeftMotorPins.pwm) &&
         gpio_valid(kLeftMotorPins.dir) && gpio_valid(kRightMotorPins.pwm) &&
         gpio_valid(kRightMotorPins.dir) && encoder_pins_valid(kLeftEncoderPins) &&
         encoder_pins_valid(kRightEncoderPins) && kPwmFrequencyHz > 0 &&
         kControlPeriodUs > 0 && kWatchdogTimeoutMs > 0 &&
         kEncoderCountsPerOutputRevolution > 0 && kMaxTargetRadS > 0.0F;
}

}  // namespace ugv_mcu::config
