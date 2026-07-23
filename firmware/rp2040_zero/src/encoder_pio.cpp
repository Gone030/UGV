#include "ugv_mcu/encoder_pio.hpp"

#include "encoder_quadrature.pio.h"
#include "hardware/gpio.h"

namespace ugv_mcu {

EncoderPioBackend::EncoderPioBackend(config::EncoderPins pins, PIO pio)
    : pins_(pins), pio_(pio) {}

bool EncoderPioBackend::initialize() {
  hardware_active_ = false;
  have_raw_count_ = false;
  extended_count_ = 0;

  // Unassigned or non-consecutive pins deliberately remain a zero-count stub.
  if (!config::encoder_pins_valid(pins_)) return true;
  if (!pio_can_add_program(pio_, &encoder_quadrature_program)) return false;
  state_machine_ = pio_claim_unused_sm(pio_, false);
  if (state_machine_ < 0) return false;

  program_offset_ = pio_add_program(pio_, &encoder_quadrature_program);
  encoder_quadrature_program_init(pio_, static_cast<unsigned>(state_machine_),
                                  program_offset_, static_cast<unsigned>(pins_.a));
  hardware_active_ = true;
  return true;
}

std::int64_t EncoderPioBackend::count() const {
  if (!hardware_active_) return 0;
  const unsigned sm = static_cast<unsigned>(state_machine_);
  // Freeze while copying X through ISR into the RX FIFO.
  pio_sm_set_enabled(pio_, sm, false);
  pio_sm_exec(pio_, sm, pio_encode_mov(pio_isr, pio_x));
  pio_sm_exec(pio_, sm, pio_encode_push(false, true));
  const std::uint32_t raw = pio_sm_get_blocking(pio_, sm);
  pio_sm_exec(
      pio_, sm,
      pio_encode_jmp(program_offset_ + encoder_quadrature_offset_sample));
  pio_sm_set_enabled(pio_, sm, true);
  if (!have_raw_count_) {
    previous_raw_count_ = raw;
    have_raw_count_ = true;
    return extended_count_;
  }
  const auto delta = static_cast<std::int32_t>(raw - previous_raw_count_);
  extended_count_ += static_cast<std::int64_t>(delta);
  previous_raw_count_ = raw;
  return extended_count_;
}

}  // namespace ugv_mcu
