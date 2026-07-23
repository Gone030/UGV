#include "ugv_mcu/protocol.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

int main() {
  using namespace ugv_mcu;
  CommandPacket command{};
  assert(parse_command_line("CMD,1,42,1,3.5,3.5", 10.0F, command) == ParseError::kNone);
  assert(command.sequence == 42 && command.enable && command.left_target_rad_s == 3.5F);
  const CommandPacket previous = command;
  assert(parse_command_line("CMD,1,43,1,nan,2", 10.0F, command) == ParseError::kBadNumber);
  assert(command.sequence == previous.sequence); // Rejected data is not committed.
  assert(parse_command_line("CMD,2,43,1,1,1", 10.0F, command) == ParseError::kBadVersion);
  assert(parse_command_line("CMD,1,43,1,11,1", 10.0F, command) == ParseError::kOutOfRange);

  LineProtocolReceiver receiver;
  const std::string stream = "CMD,1,7,0,0,0\nCMD,1,8,1,2,-2\n";
  int accepted = 0;
  for (char c : stream) { const auto r = receiver.push(c, 10.0F, command); if (r.accepted) ++accepted; }
  assert(accepted == 2 && command.sequence == 8);

  StatusPacket status{}; status.sequence = 42; status.enabled = true;
  status.left_target_rad_s = 3.5F; status.right_target_rad_s = 3.5F;
  status.left_measured_rad_s = 3.4F; status.right_measured_rad_s = 3.6F;
  status.left_encoder_count = 1024; status.right_encoder_count = 1031;
  status.left_pwm = 0.42F; status.right_pwm = 0.45F;
  char line[256];
  assert(serialize_status(status, line, sizeof(line)) > 0);
  assert(std::strstr(line, "STATE,1,42,1,0,0,3.5,3.5,3.4,3.6,1024,1031,0.42,0.45") == line);

  std::string checksummed = "CMD,1,99,1,1,-1";
  char suffix[8]; std::snprintf(suffix, sizeof(suffix), "*%04X", crc16_ccitt(checksummed));
  checksummed += suffix;
  assert(parse_command_line(checksummed, 10.0F, command) == ParseError::kNone);
  checksummed.back() = checksummed.back() == '0' ? '1' : '0';
  assert(parse_command_line(checksummed, 10.0F, command) == ParseError::kBadChecksum);
}
