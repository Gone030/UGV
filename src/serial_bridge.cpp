#include "ugv/serial_bridge.hpp"

namespace ugv {
namespace {

class NullSerialBridge final : public SerialBridge {
 public:
  bool connected() const override { return false; }
  bool send_command(const WheelVelocityCommand&) override { return false; }
  std::optional<McuStatus> poll_status() override { return std::nullopt; }
};

}  // namespace

std::unique_ptr<SerialBridge> make_null_serial_bridge() {
  return std::make_unique<NullSerialBridge>();
}

}  // namespace ugv
