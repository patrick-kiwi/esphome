#include "pulse_meter_sensor.h"
#include <utility>
#include "esphome/core/log.h"

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
  this->total_pulses_ = pulses;
  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_);
  }
}

void PulseMeterSensor::set_total_pulse_width(uint32_t pulse_width_seconds) {
  this->total_pulse_width_ = pulse_width_seconds;
  if (this->cumulative_sensor_ != nullptr) {
    this->cumulative_sensor_->publish_state(static_cast<float>(this->total_pulse_width_) /
                                            1e6f);  // Convert µs to seconds
  }
}

void PulseMeterSensor::setup() {
  this->pin_->setup();
  this->isr_pin_ = pin_->to_isr();
  this->last_processed_edge_us_ = micros();

  if (this->filter_mode_ == FILTER_EDGE) {
    this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
  } else if (this->filter_mode_ == FILTER_PULSE) {
    this->pulse_state_.last_pin_val_ = this->isr_pin_.digital_read();
    this->pulse_state_.latched_ = this->pulse_state_.last_pin_val_;
    this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
  }
}

void PulseMeterSensor::loop() {
  const uint32_t now = micros();
  this->get_->count_ = 0;

  auto *temp = this->set_;
  this->set_ = this->get_;
  this->get_ = temp;

  if (this->get_->count_ > 0) {
    if (this->total_sensor_ != nullptr) {
      this->total_pulses_ += this->get_->count_;
      this->total_sensor_->publish_state(this->total_pulses_);
    }

    // Calculate and accumulate pulse width
    uint32_t delta_us = this->get_->last_detected_edge_us_ - this->last_processed_edge_us_;
    float pulse_width_us = delta_us / float(this->get_->count_);
    this->total_pulse_width_ += static_cast<uint32_t>(pulse_width_us);

    if (this->cumulative_sensor_ != nullptr) {
      this->cumulative_sensor_->publish_state(static_cast<float>(this->total_pulse_width_) / 1e6f);
    }

    this->last_processed_edge_us_ = this->get_->last_detected_edge_us_;
  } else if (now - this->last_processed_edge_us_ > this->timeout_us_) {
    this->meter_state_ = MeterState::TIMED_OUT;
    ESP_LOGD(TAG, "No pulse detected for %" PRIu32 "s, assuming 0 pulses/min",
             (now - this->last_processed_edge_us_) / 1e6);
    this->publish_state(0.0f);
  }
}

float PulseMeterSensor::get_setup_priority() const { return setup_priority::DATA; }

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  if (this->filter_mode_ == FILTER_EDGE) {
    ESP_LOGCONFIG(TAG, "  Filtering rising edges less than %" PRIu32 " µs apart", this->filter_us_);
  } else {
    ESP_LOGCONFIG(TAG, "  Filtering pulses shorter than %" PRIu32 " µs", this->filter_us_);
  }
  ESP_LOGCONFIG(TAG, "  Assuming 0 pulses/min after not receiving a pulse for %" PRIu32 "s", this->timeout_us_ / 1e6);
  if (this->cumulative_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Cumulative pulse width sensor configured.");
  }
}

void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  auto &state = sensor->edge_state_;
  auto &set = *sensor->set_;

  if ((now - state.last_sent_edge_us_) >= sensor->filter_us_) {
    state.last_sent_edge_us_ = now;
    set.last_detected_edge_us_ = now;
    set.last_rising_edge_us_ = now;
    set.count_++;
  }
}

void IRAM_ATTR PulseMeterSensor::pulse_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  const bool pin_val = sensor->isr_pin_.digital_read();
  auto &state = sensor->pulse_state_;
  auto &set = *sensor->set_;

  const bool length = now - state.last_intr_ >= sensor->filter_us_;
  if (length && state.latched_ && !state.last_pin_val_) {
    state.latched_ = false;
  } else if (length && !state.latched_ && state.last_pin_val_) {
    state.latched_ = true;
    set.last_detected_edge_us_ = state.last_intr_;
    set.count_++;
  }

  set.last_rising_edge_us_ = !state.latched_ && pin_val ? now : set.last_detected_edge_us_;
  state.last_intr_ = now;
  state.last_pin_val_ = pin_val;
}

}  // namespace pulse_meter
}  // namespace esphome
