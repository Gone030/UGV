#pragma once

#include <cstdint>
#include <memory>
#include <optional>

namespace ugv {

struct WheelVelocityCommand {
  std::uint32_t sequence{};
  bool enable{};
  double left_target_rad_s{};
  double right_target_rad_s{};
};

struct McuStatus {
  std::uint32_t protocol_version{};
  std::uint32_t sequence{};
  bool enabled{};
  bool watchdog_active{};
  std::uint32_t fault_flags{};
  double left_target_rad_s{};
  double right_target_rad_s{};
  double left_measured_rad_s{};
  double right_measured_rad_s{};
  std::int64_t left_encoder_count{};
  std::int64_t right_encoder_count{};
  double left_pwm{};
  double right_pwm{};
};

// Transport boundary for the future real USB CDC implementation. The concrete
// backend will own device open/close, reconnect, framing, parsing and I/O.
class SerialBridge {
 public:
  virtual ~SerialBridge() = default;
  virtual bool connected() const = 0;
  virtual bool send_command(const WheelVelocityCommand& command) = 0;
  virtual std::optional<McuStatus> poll_status() = 0;
};

// Current safe backend: performs no device I/O and never reports success.
std::unique_ptr<SerialBridge> make_null_serial_bridge();

}  // namespace ugv
