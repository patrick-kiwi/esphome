#include "rmt_simple.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rmt_simple {

static const char *const TAG = "rmt_simple";

void RmtSimpleComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RMT Simple...");

  // Count configured channels
  for (auto &pin : this->pins_) {
    if (pin != nullptr) {
      this->num_channels_++;
    }
  }

  // Validate
  if (this->num_channels_ == 0) {
    ESP_LOGE(TAG, "At least one pin must be configured");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "  Detected %d channels", this->num_channels_);

  // Initialize appropriate RMT generator based on channel count
  if (this->num_channels_ <= 2) {
    // 2-channel mode - find the first two configured pins
    gpio_num_t gpios[2];
    int gpio_index = 0;

    for (int i = 0; i < 4 && gpio_index < 2; i++) {
      if (this->pins_[i] != nullptr) {
        gpios[gpio_index++] = (gpio_num_t) this->pins_[i]->get_pin();
      }
    }

    this->generator_2ch_ = std::make_unique<RmtPulseGenerator<2>>(
        gpios[0],
        gpio_index > 1 ? gpios[1] : gpios[0],  // Use first pin twice if only one configured
        this->resolution_hz_, this->align_pulse_lengths_, this->use_sync_manager_);

    if (!this->generator_2ch_->init()) {
      ESP_LOGE(TAG, "Failed to initialize 2-channel RMT peripheral");
      this->mark_failed();
      return;
    }

    ESP_LOGCONFIG(TAG, "  Initialized 2-channel RMT generator");

  } else {
    // 4-channel mode - find the first four configured pins
    gpio_num_t gpios[4];
    int gpio_index = 0;

    for (int i = 0; i < 4 && gpio_index < 4; i++) {
      if (this->pins_[i] != nullptr) {
        gpios[gpio_index++] = (gpio_num_t) this->pins_[i]->get_pin();
      }
    }

    this->generator_4ch_ = std::make_unique<RmtPulseGenerator<4>>(gpios[0], gpios[1], gpios[2], gpios[3],
                                                                  this->resolution_hz_, this->align_pulse_lengths_);

    if (!this->generator_4ch_->init()) {
      ESP_LOGE(TAG, "Failed to initialize 4-channel RMT peripheral");
      this->mark_failed();
      return;
    }

    ESP_LOGCONFIG(TAG, "  Initialized 4-channel RMT generator");
  }

  ESP_LOGCONFIG(TAG, "RMT Simple initialized successfully");

  // Auto-start pulses
  this->auto_start_();
}

void RmtSimpleComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "RMT Simple:");
  ESP_LOGCONFIG(TAG, "  Channels: %d", this->num_channels_);
  ESP_LOGCONFIG(TAG, "  Resolution: %u Hz", this->resolution_hz_);

  for (int i = 0; i < 4; i++) {
    if (this->pins_[i] != nullptr) {
      ESP_LOGCONFIG(TAG, "  Channel %d:", i);
      LOG_PIN("    Pin: ", this->pins_[i]);
      ESP_LOGCONFIG(TAG, "    Pulse symbols: %d", this->channel_pulses_[i].size());
    }
  }

  ESP_LOGCONFIG(TAG, "  Auto-start: enabled");
}

void RmtSimpleComponent::set_pin(uint8_t channel, InternalGPIOPin *pin) {
  if (channel < 4) {
    this->pins_[channel] = pin;
    pin->setup();
  }
}

void RmtSimpleComponent::set_pulses(uint8_t channel, const std::vector<rmt_symbol_word_t> &symbols) {
  if (channel < 4) {
    this->channel_pulses_[channel] = symbols;
  }
}

void RmtSimpleComponent::auto_start_() {
  // Build vector of pulse patterns for configured channels
  std::vector<std::vector<rmt_symbol_word_t>> patterns;

  for (int i = 0; i < 4; i++) {
    if (this->pins_[i] != nullptr) {
      if (!this->channel_pulses_[i].empty()) {
        patterns.push_back(this->channel_pulses_[i]);
      } else {
        // Empty pattern for channel with no pulses configured
        patterns.emplace_back();
      }
    }
  }

  // Start generation
  if (!this->begin_(patterns)) {
    ESP_LOGE(TAG, "Failed to auto-start pulse generation");
  } else {
    ESP_LOGI(TAG, "Auto-started pulse generation on %d channel(s)", this->num_channels_);
  }
}

bool RmtSimpleComponent::begin_(const std::vector<std::vector<rmt_symbol_word_t>> &channel_sequences) {
  if (this->generator_2ch_ == nullptr && this->generator_4ch_ == nullptr) {
    ESP_LOGE(TAG, "Component not initialized");
    return false;
  }

  // Validate channel count
  if (channel_sequences.size() != this->num_channels_) {
    ESP_LOGE(TAG, "Expected %d channel sequences, got %d", this->num_channels_, channel_sequences.size());
    return false;
  }

  bool success = false;
  if (this->generator_2ch_ != nullptr) {
    success = this->generator_2ch_->begin(channel_sequences);
  } else if (this->generator_4ch_ != nullptr) {
    success = this->generator_4ch_->begin(channel_sequences);
  }

  if (success) {
    this->running_ = true;
    ESP_LOGD(TAG, "Started pulse generation");
  } else {
    this->running_ = false;
    ESP_LOGE(TAG, "Failed to start pulse generation");
  }

  return success;
}

void RmtSimpleComponent::stop_() {
  if (this->generator_2ch_ == nullptr && this->generator_4ch_ == nullptr) {
    ESP_LOGE(TAG, "Component not initialized");
    return;
  }

  if (this->generator_2ch_ != nullptr) {
    this->generator_2ch_->stop();
  } else if (this->generator_4ch_ != nullptr) {
    this->generator_4ch_->stop();
  }

  this->running_ = false;
  ESP_LOGI(TAG, "Stopped pulse generation");
}

bool RmtSimpleComponent::is_running() const {
  if (this->generator_2ch_ != nullptr) {
    return this->generator_2ch_->is_running();
  } else if (this->generator_4ch_ != nullptr) {
    return this->generator_4ch_->is_running();
  }
  return false;
}

}  // namespace rmt_simple
}  // namespace esphome
