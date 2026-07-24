#include "ugv/serial_bridge.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

namespace ugv {
namespace {

constexpr std::uint32_t kProtocolVersion = 1;
constexpr std::size_t kStatusFieldCount = 14;
constexpr std::size_t kMaximumLineLength = 512;

std::uint16_t crc16_ccitt(std::string_view payload) {
  std::uint16_t crc = 0xFFFF;
  for (const unsigned char byte : payload) {
    crc ^= static_cast<std::uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U)
                ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

bool split_and_check_crc(
    const std::string& input, bool require_crc, std::string& payload) {
  payload = input;
  while (!payload.empty() && (payload.back() == '\n' || payload.back() == '\r')) {
    payload.pop_back();
  }
  const auto marker = payload.rfind('*');
  if (marker == std::string::npos) {
    return !require_crc;
  }
  if (payload.size() - marker - 1 != 4) {
    return false;
  }
  const std::string checksum_text = payload.substr(marker + 1);
  char* end = nullptr;
  errno = 0;
  const auto expected = std::strtoul(checksum_text.c_str(), &end, 16);
  if (errno != 0 || end == checksum_text.c_str() || *end != '\0' ||
      expected > std::numeric_limits<std::uint16_t>::max()) {
    return false;
  }
  payload.resize(marker);
  return crc16_ccitt(payload) == static_cast<std::uint16_t>(expected);
}

std::vector<std::string> split_fields(const std::string& text) {
  std::vector<std::string> fields;
  std::size_t begin = 0;
  while (true) {
    const auto comma = text.find(',', begin);
    fields.emplace_back(text.substr(begin, comma - begin));
    if (comma == std::string::npos) {
      break;
    }
    begin = comma + 1;
  }
  return fields;
}

template <typename Integer>
bool parse_unsigned(const std::string& text, Integer& output) {
  if (text.empty() || text.front() == '-') {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  const auto parsed = std::strtoull(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0' ||
      parsed > std::numeric_limits<Integer>::max()) {
    return false;
  }
  output = static_cast<Integer>(parsed);
  return true;
}

bool parse_i64(const std::string& text, std::int64_t& output) {
  char* end = nullptr;
  errno = 0;
  const auto parsed = std::strtoll(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0') {
    return false;
  }
  output = static_cast<std::int64_t>(parsed);
  return true;
}

bool parse_finite(const std::string& text, double& output) {
  char* end = nullptr;
  errno = 0;
  output = std::strtod(text.c_str(), &end);
  return errno == 0 && end != text.c_str() && *end == '\0' && std::isfinite(output);
}

speed_t baud_constant(std::uint32_t baud_rate) {
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return 0;
  }
}

class UsbCdcSerialBridge final : public SerialBridge {
 public:
  explicit UsbCdcSerialBridge(SerialBridgeConfig config) : config_(std::move(config)) {}
  ~UsbCdcSerialBridge() override {
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked();
  }

  bool connected() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return file_descriptor_ >= 0;
  }

  bool send_command(const WheelVelocityCommand& command) override {
    const std::string line = serialize_command(command, config_.require_crc);
    if (line.empty()) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensure_connected_locked()) {
      return false;
    }
    std::size_t written = 0;
    while (written < line.size()) {
      const auto result =
          ::write(file_descriptor_, line.data() + written, line.size() - written);
      if (result > 0) {
        written += static_cast<std::size_t>(result);
        continue;
      }
      if (result < 0 && errno == EINTR) {
        continue;
      }
      close_locked();
      return false;
    }
    return true;
  }

  std::optional<McuStatus> poll_status() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensure_connected_locked()) {
      return std::nullopt;
    }
    std::array<char, 256> buffer{};
    while (true) {
      const auto count = ::read(file_descriptor_, buffer.data(), buffer.size());
      if (count > 0) {
        receive_buffer_.append(buffer.data(), static_cast<std::size_t>(count));
        if (receive_buffer_.size() > kMaximumLineLength * 2) {
          receive_buffer_.clear();
        }
        continue;
      }
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }
      if (count == 0) {
        break;
      }
      close_locked();
      return std::nullopt;
    }

    while (true) {
      const auto newline = receive_buffer_.find('\n');
      if (newline == std::string::npos) {
        return std::nullopt;
      }
      std::string line = receive_buffer_.substr(0, newline + 1);
      receive_buffer_.erase(0, newline + 1);
      if (line.size() > kMaximumLineLength) {
        continue;
      }
      if (auto status = parse_status_line(line, config_.require_crc)) {
        return status;
      }
    }
  }

 private:
  bool ensure_connected_locked() {
    if (file_descriptor_ >= 0) {
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now < next_reconnect_) {
      return false;
    }
    next_reconnect_ =
        now + std::chrono::milliseconds(config_.reconnect_interval_ms);

    const speed_t speed = baud_constant(config_.baud_rate);
    if (speed == 0 || config_.device.empty()) {
      return false;
    }
    const int descriptor =
        ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0) {
      return false;
    }
    termios settings{};
    if (tcgetattr(descriptor, &settings) != 0) {
      ::close(descriptor);
      return false;
    }
    cfmakeraw(&settings);
    cfsetispeed(&settings, speed);
    cfsetospeed(&settings, speed);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (tcsetattr(descriptor, TCSANOW, &settings) != 0) {
      ::close(descriptor);
      return false;
    }
    tcflush(descriptor, TCIOFLUSH);
    file_descriptor_ = descriptor;
    receive_buffer_.clear();
    return true;
  }

  void close_locked() {
    if (file_descriptor_ >= 0) {
      ::close(file_descriptor_);
      file_descriptor_ = -1;
    }
    receive_buffer_.clear();
  }

  SerialBridgeConfig config_;
  mutable std::mutex mutex_;
  int file_descriptor_{-1};
  std::string receive_buffer_;
  std::chrono::steady_clock::time_point next_reconnect_{};
};

class NullSerialBridge final : public SerialBridge {
 public:
  bool connected() const override { return false; }
  bool send_command(const WheelVelocityCommand&) override { return false; }
  std::optional<McuStatus> poll_status() override { return std::nullopt; }
};

}  // namespace

std::string serialize_command(const WheelVelocityCommand& command, bool append_crc) {
  if (!std::isfinite(command.left_target_rad_s) ||
      !std::isfinite(command.right_target_rad_s)) {
    return {};
  }
  std::array<char, 192> output{};
  const int length = std::snprintf(
      output.data(), output.size(), "CMD,%u,%u,%u,%.9g,%.9g",
      kProtocolVersion, command.sequence, command.enable ? 1U : 0U,
      command.left_target_rad_s, command.right_target_rad_s);
  if (length < 0 || static_cast<std::size_t>(length) >= output.size()) {
    return {};
  }
  std::string line(output.data(), static_cast<std::size_t>(length));
  if (append_crc) {
    std::array<char, 8> checksum{};
    std::snprintf(checksum.data(), checksum.size(), "*%04X", crc16_ccitt(line));
    line += checksum.data();
  }
  line.push_back('\n');
  return line;
}

std::optional<McuStatus> parse_status_line(const std::string& line, bool require_crc) {
  std::string payload;
  if (!split_and_check_crc(line, require_crc, payload)) {
    return std::nullopt;
  }
  const auto fields = split_fields(payload);
  if (fields.size() != kStatusFieldCount || fields[0] != "STATE") {
    return std::nullopt;
  }

  McuStatus candidate{};
  std::uint32_t enabled = 0;
  std::uint32_t watchdog_active = 0;
  if (!parse_unsigned(fields[1], candidate.protocol_version) ||
      candidate.protocol_version != kProtocolVersion ||
      !parse_unsigned(fields[2], candidate.sequence) ||
      !parse_unsigned(fields[3], enabled) || enabled > 1 ||
      !parse_unsigned(fields[4], watchdog_active) || watchdog_active > 1 ||
      !parse_unsigned(fields[5], candidate.fault_flags) ||
      !parse_finite(fields[6], candidate.left_target_rad_s) ||
      !parse_finite(fields[7], candidate.right_target_rad_s) ||
      !parse_finite(fields[8], candidate.left_measured_rad_s) ||
      !parse_finite(fields[9], candidate.right_measured_rad_s) ||
      !parse_i64(fields[10], candidate.left_encoder_count) ||
      !parse_i64(fields[11], candidate.right_encoder_count) ||
      !parse_finite(fields[12], candidate.left_pwm) ||
      !parse_finite(fields[13], candidate.right_pwm)) {
    return std::nullopt;
  }
  if (std::fabs(candidate.left_pwm) > 1.0 ||
      std::fabs(candidate.right_pwm) > 1.0) {
    return std::nullopt;
  }
  candidate.enabled = enabled == 1;
  candidate.watchdog_active = watchdog_active == 1;
  return candidate;
}

std::string serialize_status_message(const McuStatus& status) {
  std::ostringstream output;
  output.precision(9);
  output << "STATE," << status.protocol_version << ',' << status.sequence << ','
         << (status.enabled ? 1 : 0) << ',' << (status.watchdog_active ? 1 : 0)
         << ',' << status.fault_flags << ',' << status.left_target_rad_s << ','
         << status.right_target_rad_s << ',' << status.left_measured_rad_s << ','
         << status.right_measured_rad_s << ',' << status.left_encoder_count << ','
         << status.right_encoder_count << ',' << status.left_pwm << ','
         << status.right_pwm;
  return output.str();
}

std::unique_ptr<SerialBridge> make_usb_cdc_serial_bridge(
    const SerialBridgeConfig& config) {
  return std::make_unique<UsbCdcSerialBridge>(config);
}

std::unique_ptr<SerialBridge> make_null_serial_bridge() {
  return std::make_unique<NullSerialBridge>();
}

}  // namespace ugv
