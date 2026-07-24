#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

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

struct SerialBridgeConfig {
  std::string device{"/dev/ttyACM0"};
  std::uint32_t baud_rate{115200};
  std::uint32_t reconnect_interval_ms{1000};
  bool require_crc{true};
};

class SerialBridge {
 public:
  virtual ~SerialBridge() = default;
  virtual bool connected() const = 0;
  virtual bool send_command(const WheelVelocityCommand& command) = 0;
  virtual std::optional<McuStatus> poll_status() = 0;
};

std::string serialize_command(const WheelVelocityCommand& command, bool append_crc = true);
std::optional<McuStatus> parse_status_line(const std::string& line, bool require_crc = true);
std::string serialize_status_message(const McuStatus& status);

std::unique_ptr<SerialBridge> make_usb_cdc_serial_bridge(const SerialBridgeConfig& config);
std::unique_ptr<SerialBridge> make_null_serial_bridge();

}  // namespace ugv
