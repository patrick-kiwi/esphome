#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "RmtPulseGenerator.h"
#include <memory>
#include <vector>

namespace esphome {
namespace rmt_simple {

class RmtSimpleComponent : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // Configuration setters (called from Python codegen)
  void set_pin(uint8_t channel, InternalGPIOPin *pin);
  void set_resolution(uint32_t resolution_hz) { resolution_hz_ = resolution_hz; }
  void set_align_pulse_lengths(bool enable) { align_pulse_lengths_ = enable; }
  void set_use_sync_manager(bool enable) { use_sync_manager_ = enable; }
  void set_pulses(uint8_t channel, const std::vector<rmt_symbol_word_t> &symbols);

  // Status
  bool is_running() const;

 protected:
  bool auto_start_();
  bool begin_(const std::vector<std::vector<rmt_symbol_word_t>> &channel_sequences);
  void stop_();

 private:
  InternalGPIOPin *pins_[4]{nullptr};
  std::vector<rmt_symbol_word_t> channel_pulses_[4];
  uint32_t resolution_hz_{1000000};
  bool align_pulse_lengths_{true};
  bool use_sync_manager_{false};
  std::unique_ptr<RmtPulseGenerator<2>> generator_2ch_;
  std::unique_ptr<RmtPulseGenerator<4>> generator_4ch_;
  bool running_{false};
  uint8_t num_channels_{0};
};

}  // namespace rmt_simple
}  // namespace esphome
