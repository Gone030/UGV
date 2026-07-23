#include "ugv_hw_test/command.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace ugv_hw_test {
namespace {
bool parse_float(const char* text, float& value) {
  char* end = nullptr;
  errno = 0;
  value = std::strtof(text, &end);
  return errno == 0 && end != text && *end == '\0' && std::isfinite(value);
}

std::size_t split(char* text, char** fields, std::size_t capacity) {
  std::size_t count = 0;
  char* cursor = text;
  while (cursor && count < capacity) {
    fields[count++] = cursor;
    char* comma = std::strchr(cursor, ',');
    if (!comma) break;
    *comma = '\0';
    cursor = comma + 1;
  }
  return count;
}
}  // namespace

CommandError parse_command(std::string_view line, Command& output) {
  if (line.empty()) return CommandError::kEmpty;
  if (line.size() >= CommandReceiver::kCapacity) return CommandError::kOverflow;
  char storage[CommandReceiver::kCapacity]{};
  std::memcpy(storage, line.data(), line.size());
  std::size_t length = line.size();
  while (length && (storage[length - 1] == '\r' || storage[length - 1] == '\n'))
    storage[--length] = '\0';
  char* fields[5]{}; // One extra slot ensures trailing fields are rejected.
  const std::size_t count = split(storage, fields, 5);
  Command candidate{};

  if (std::strcmp(fields[0], "STATUS") == 0) {
    if (count != 1) return CommandError::kFieldCount;
    candidate.type = CommandType::kStatus;
  } else if (std::strcmp(fields[0], "MODE") == 0) {
    if (count != 2) return CommandError::kFieldCount;
    candidate.type = CommandType::kSetMode;
    if (std::strcmp(fields[1], "ENCODER") == 0) candidate.mode = Mode::kEncoder;
    else if (std::strcmp(fields[1], "MOTOR") == 0) candidate.mode = Mode::kMotor;
    else if (std::strcmp(fields[1], "SIM") == 0) candidate.mode = Mode::kSim;
    else return CommandError::kBadMode;
  } else if (std::strcmp(fields[0], "ENABLE") == 0) {
    if (count != 2) return CommandError::kFieldCount;
    candidate.type = CommandType::kEnable;
    if (std::strcmp(fields[1], "0") == 0) candidate.enable = false;
    else if (std::strcmp(fields[1], "1") == 0) candidate.enable = true;
    else return CommandError::kBadBoolean;
  } else if (std::strcmp(fields[0], "MOTOR") == 0) {
    if (count < 3) return CommandError::kFieldCount;
    if (std::strcmp(fields[1], "L") == 0 || std::strcmp(fields[1], "R") == 0) {
      if (count != 3) return CommandError::kFieldCount;
      if (!parse_float(fields[2], candidate.left)) return CommandError::kBadNumber;
      candidate.type = std::strcmp(fields[1], "L") == 0 ? CommandType::kMotorLeft
                                                         : CommandType::kMotorRight;
    } else if (std::strcmp(fields[1], "BOTH") == 0) {
      if (count != 4) return CommandError::kFieldCount;
      if (!parse_float(fields[2], candidate.left) ||
          !parse_float(fields[3], candidate.right)) return CommandError::kBadNumber;
      candidate.type = CommandType::kMotorBoth;
    } else return CommandError::kUnknown;
  } else if (std::strcmp(fields[0], "SIM_SPEED") == 0) {
    if (count != 3) return CommandError::kFieldCount;
    if (!parse_float(fields[1], candidate.left) ||
        !parse_float(fields[2], candidate.right)) return CommandError::kBadNumber;
    candidate.type = CommandType::kSimSpeed;
  } else {
    return CommandError::kUnknown;
  }
  output = candidate;
  return CommandError::kNone;
}

const char* mode_name(Mode mode) {
  switch (mode) {
    case Mode::kEncoder: return "ENCODER";
    case Mode::kMotor: return "MOTOR";
    case Mode::kSim: return "SIM";
  }
  return "UNKNOWN";
}

const char* error_name(CommandError error) {
  switch (error) {
    case CommandError::kNone: return "NONE";
    case CommandError::kEmpty: return "EMPTY";
    case CommandError::kOverflow: return "OVERFLOW";
    case CommandError::kUnknown: return "UNKNOWN_COMMAND";
    case CommandError::kFieldCount: return "FIELD_COUNT";
    case CommandError::kBadNumber: return "BAD_NUMBER";
    case CommandError::kBadBoolean: return "BAD_BOOLEAN";
    case CommandError::kBadMode: return "BAD_MODE";
  }
  return "UNKNOWN_ERROR";
}

FeedResult CommandReceiver::push(char byte, Command& output) {
  if (byte != '\n') {
    if (byte != '\r') {
      if (size_ < buffer_.size() - 1) buffer_[size_++] = byte;
      else overflow_ = true;
    }
    return {};
  }
  if (overflow_) { reset(); return {true, false, CommandError::kOverflow}; }
  buffer_[size_] = '\0';
  const auto error = parse_command({buffer_.data(), size_}, output);
  reset();
  return {true, error == CommandError::kNone, error};
}

void CommandReceiver::reset() { size_ = 0; overflow_ = false; }
}  // namespace ugv_hw_test
