#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace ugv_hw_test {

enum class Mode { kEncoder, kMotor, kSim };
enum class CommandType { kStatus, kSetMode, kEnable, kMotorLeft, kMotorRight,
                         kMotorBoth, kSimSpeed };
enum class CommandError { kNone, kEmpty, kOverflow, kUnknown, kFieldCount,
                          kBadNumber, kBadBoolean, kBadMode };

struct Command {
  CommandType type{CommandType::kStatus};
  Mode mode{Mode::kEncoder};
  bool enable{};
  float left{};
  float right{};
};

struct FeedResult {
  bool line_complete{};
  bool accepted{};
  CommandError error{CommandError::kNone};
};

CommandError parse_command(std::string_view line, Command& output);
const char* mode_name(Mode mode);
const char* error_name(CommandError error);

class CommandReceiver {
 public:
  static constexpr std::size_t kCapacity = 128;
  FeedResult push(char byte, Command& output);
  void reset();

 private:
  std::array<char, kCapacity> buffer_{};
  std::size_t size_{};
  bool overflow_{};
};

}  // namespace ugv_hw_test
