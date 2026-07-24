#include "ugv_mcu/protocol.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace ugv_mcu {
namespace {
constexpr std::size_t kCommandFields = 6;

bool parse_u32(const char* text, std::uint32_t& value) {
  if (*text == '\0' || *text == '-') return false;
  char* end = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(text, &end, 10);
  if (errno || *end != '\0' || parsed > std::numeric_limits<std::uint32_t>::max()) return false;
  value = static_cast<std::uint32_t>(parsed);
  return true;
}

bool parse_float(const char* text, float& value) {
  char* end = nullptr;
  errno = 0;
  value = std::strtof(text, &end);
  return !errno && end != text && *end == '\0' && std::isfinite(value);
}

bool checksum_valid(char* line) {
  char* marker = std::strrchr(line, '*');
  if (!marker) return false;
  if (std::strlen(marker + 1) != 4) return false;
  char* end = nullptr;
  const unsigned long expected = std::strtoul(marker + 1, &end, 16);
  if (*end != '\0' || expected > 0xFFFFUL) return false;
  *marker = '\0';
  return crc16_ccitt(line) == static_cast<std::uint16_t>(expected);
}
}  // namespace

std::uint16_t crc16_ccitt(std::string_view payload) {
  std::uint16_t crc = 0xFFFF;
  for (unsigned char byte : payload) {
    crc ^= static_cast<std::uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000U) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021U)
                            : static_cast<std::uint16_t>(crc << 1);
  }
  return crc;
}

ParseError parse_command_line(std::string_view line, float max_abs_target_rad_s,
                              CommandPacket& output) {
  if (line.empty()) return ParseError::kEmpty;
  if (line.size() >= LineProtocolReceiver::kBufferSize) return ParseError::kOverflow;
  char storage[LineProtocolReceiver::kBufferSize]{};
  std::memcpy(storage, line.data(), line.size());
  std::size_t length = line.size();
  while (length && (storage[length - 1] == '\r' || storage[length - 1] == '\n'))
    storage[--length] = '\0';
  if (!checksum_valid(storage)) return ParseError::kBadChecksum;

  std::array<char*, kCommandFields> fields{};
  std::size_t count = 0;
  char* cursor = storage;
  while (cursor && count < fields.size()) {
    fields[count++] = cursor;
    char* comma = std::strchr(cursor, ',');
    if (!comma) { cursor = nullptr; break; }
    *comma = '\0';
    cursor = comma + 1;
  }
  if (count != kCommandFields || cursor != nullptr) return ParseError::kBadFieldCount;
  if (std::strcmp(fields[0], "CMD") != 0) return ParseError::kBadType;

  CommandPacket candidate{};
  std::uint32_t enable = 0;
  if (!parse_u32(fields[1], candidate.protocol_version) ||
      !parse_u32(fields[2], candidate.sequence) || !parse_u32(fields[3], enable) ||
      !parse_float(fields[4], candidate.left_target_rad_s) ||
      !parse_float(fields[5], candidate.right_target_rad_s)) return ParseError::kBadNumber;
  if (candidate.protocol_version != kProtocolVersion) return ParseError::kBadVersion;
  if (enable > 1) return ParseError::kBadEnable;
  candidate.enable = enable == 1;
  if (!candidate.enable) {
    if (candidate.left_target_rad_s != 0.0F ||
        candidate.right_target_rad_s != 0.0F) {
      return ParseError::kOutOfRange;
    }
  } else if (!(max_abs_target_rad_s > 0.0F) ||
             std::fabs(candidate.left_target_rad_s) > max_abs_target_rad_s ||
             std::fabs(candidate.right_target_rad_s) > max_abs_target_rad_s) {
    return ParseError::kOutOfRange;
  }
  output = candidate;  // Commit only after every field has been validated.
  return ParseError::kNone;
}

std::size_t serialize_status(const StatusPacket& p, char* out, std::size_t cap, bool crc) {
  if (!out || cap == 0) return 0;
  const int n = std::snprintf(out, cap,
      "STATE,%lu,%lu,%u,%u,%lu,%.6g,%.6g,%.6g,%.6g,%lld,%lld,%.6g,%.6g",
      static_cast<unsigned long>(p.protocol_version), static_cast<unsigned long>(p.sequence),
      static_cast<unsigned>(p.enabled), static_cast<unsigned>(p.watchdog_active),
      static_cast<unsigned long>(p.fault_flags),
      p.left_target_rad_s, p.right_target_rad_s, p.left_measured_rad_s,
      p.right_measured_rad_s, static_cast<long long>(p.left_encoder_count),
      static_cast<long long>(p.right_encoder_count), p.left_pwm, p.right_pwm);
  if (n < 0 || static_cast<std::size_t>(n) >= cap) return 0;
  std::size_t used = static_cast<std::size_t>(n);
  if (crc) {
    const int extra = std::snprintf(out + used, cap - used, "*%04X", crc16_ccitt({out, used}));
    if (extra < 0 || static_cast<std::size_t>(extra) >= cap - used) return 0;
    used += static_cast<std::size_t>(extra);
  }
  if (used + 1 >= cap) return 0;
  out[used++] = '\n'; out[used] = '\0';
  return used;
}

ParseResult LineProtocolReceiver::push(char byte, float max_target, CommandPacket& output) {
  if (byte != '\n') {
    if (byte != '\r') {
      if (size_ < buffer_.size() - 1) buffer_[size_++] = byte;
      else overflow_ = true;
    }
    return {};
  }
  if (overflow_) { reset(); return {false, true, ParseError::kOverflow}; }
  buffer_[size_] = '\0';
  const ParseError error = parse_command_line({buffer_.data(), size_}, max_target, output);
  reset();
  return {error == ParseError::kNone, true, error};
}

void LineProtocolReceiver::reset() { size_ = 0; overflow_ = false; }
}  // namespace ugv_mcu
