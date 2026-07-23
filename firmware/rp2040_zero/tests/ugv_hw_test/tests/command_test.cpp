#include "ugv_hw_test/command.hpp"

#include <cassert>
#include <cmath>
#include <string>

int main() {
  using namespace ugv_hw_test;
  Command command{};
  assert(parse_command("STATUS", command) == CommandError::kNone && command.type == CommandType::kStatus);
  assert(parse_command("MODE,ENCODER", command) == CommandError::kNone && command.mode == Mode::kEncoder);
  assert(parse_command("MODE,MOTOR", command) == CommandError::kNone && command.mode == Mode::kMotor);
  assert(parse_command("MODE,SIM", command) == CommandError::kNone && command.mode == Mode::kSim);
  assert(parse_command("ENABLE,1", command) == CommandError::kNone && command.enable);
  assert(parse_command("MOTOR,L,-0.5", command) == CommandError::kNone && command.left == -0.5F);
  assert(parse_command("MOTOR,R,0.25", command) == CommandError::kNone && command.type == CommandType::kMotorRight);
  assert(parse_command("MOTOR,BOTH,0.1,-0.2", command) == CommandError::kNone && command.right == -0.2F);
  assert(parse_command("SIM_SPEED,3.5,-3.5", command) == CommandError::kNone && command.left == 3.5F);
  assert(parse_command("SIM_SPEED,nan,1", command) == CommandError::kBadNumber);
  assert(parse_command("MODE,UNKNOWN", command) == CommandError::kBadMode);
  assert(parse_command("MOTOR,BOTH,0.1,0.2,EXTRA", command) == CommandError::kFieldCount);

  CommandReceiver receiver;
  const std::string input = "STA" "TUS\nMODE,SIM\n";
  int accepted = 0;
  for (char byte : input) {
    const auto result = receiver.push(byte, command);
    if (result.accepted) ++accepted;
  }
  assert(accepted == 2 && command.mode == Mode::kSim);
}
