#include "ugv_mcu/encoder.hpp"
#include "ugv_mcu/motor_driver.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>

namespace {
class FakeEncoderBackend final : public ugv_mcu::EncoderBackend {
 public:
  bool initialize() override { return true; }
  std::int64_t count() const override { return value; }
  bool hardware_active() const override { return false; }
  std::int64_t value{};
};
}

int main() {
  using namespace ugv_mcu;
  QuadratureDecoder decoder;
  decoder.reset(0);
  for (auto state : {1U, 3U, 2U, 0U}) decoder.update(state);
  assert(decoder.count() == 4);
  decoder.reset(0);
  for (auto state : {2U, 3U, 1U, 0U}) decoder.update(state);
  assert(decoder.count() == -4);
  decoder.reset(0);
  decoder.update(3);
  assert(decoder.count() == 0 && decoder.invalid_transition_count() == 1);

  FakeEncoderBackend backend;
  Encoder normal(backend, false);
  assert(normal.initialize());
  assert(normal.update_velocity(1'000'000, 100) == 0.0F); // Test CPR only; config remains TBD.
  backend.value = 25;
  const float speed = normal.update_velocity(1'500'000, 100);
  assert(std::fabs(speed - 3.14159265F) < 1.0e-5F);
  Encoder inverted(backend, true);
  assert(inverted.initialize() && inverted.count() == -25);

  assert(clamp_motor_command(2.0F) == 1.0F);
  assert(clamp_motor_command(-2.0F) == -1.0F);
  auto plan = plan_motor_command(0.4F, true, false);
  assert(plan.accepted && plan.direction_forward && plan.duty == 0.4F);
  plan = plan_motor_command(0.4F, true, true);
  assert(plan.accepted && !plan.direction_forward && plan.duty == 0.4F);
  plan = plan_motor_command(-0.4F, true, false);
  assert(plan.accepted && !plan.direction_forward && plan.duty == 0.4F);
  plan = plan_motor_command(0.8F, false, false);
  assert(plan.accepted && plan.duty == 0.0F && plan.signed_output == 0.0F);
  assert(!plan_motor_command(1.01F, true, false).accepted);
  assert(!plan_motor_command(std::numeric_limits<float>::quiet_NaN(), true, false).accepted);
  assert(!plan_motor_command(std::numeric_limits<float>::infinity(), true, false).accepted);
  assert(!plan_motor_command(std::numeric_limits<float>::quiet_NaN(), false, false).accepted);
}
