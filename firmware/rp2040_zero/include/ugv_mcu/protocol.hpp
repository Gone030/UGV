#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace ugv_mcu {

constexpr std::uint32_t kProtocolVersion = 1;

struct CommandPacket {
  std::uint32_t protocol_version{};
  std::uint32_t sequence{};
  bool enable{};
  float left_target_rad_s{};
  float right_target_rad_s{};
};

struct StatusPacket {
  std::uint32_t protocol_version{kProtocolVersion};
  std::uint32_t sequence{};
  bool enabled{};
  bool watchdog_active{};
  std::uint32_t fault_flags{};
  float left_target_rad_s{};
  float right_target_rad_s{};
  float left_measured_rad_s{};
  float right_measured_rad_s{};
  std::int64_t left_encoder_count{};
  std::int64_t right_encoder_count{};
  float left_pwm{};
  float right_pwm{};
};

enum class ParseError { kNone, kEmpty, kOverflow, kBadType, kBadFieldCount,
                        kBadVersion, kBadNumber, kBadEnable, kOutOfRange,
                        kBadChecksum };
struct ParseResult { bool accepted{}; bool line_complete{}; ParseError error{ParseError::kNone}; };

std::uint16_t crc16_ccitt(std::string_view payload);
ParseError parse_command_line(std::string_view line, float max_abs_target_rad_s,
                              CommandPacket& output);
std::size_t serialize_status(const StatusPacket& packet, char* output,
                             std::size_t capacity, bool append_crc = false);

class LineProtocolReceiver {
 public:
  static constexpr std::size_t kBufferSize = 160;
  ParseResult push(char byte, float max_abs_target_rad_s, CommandPacket& output);
  void reset();

 private:
  std::array<char, kBufferSize> buffer_{};
  std::size_t size_{};
  bool overflow_{};
};

}  // namespace ugv_mcu
