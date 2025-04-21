#include "esphome/core/log.h"
#include "pulse_width_accumulate.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace esphome {
namespace pulse_width_accumulate {
static const char *const TAG = "pulse_width";
constexpr uint32_t LOWER_PULSE_WIDTH_THRESHOLD = 17;  // pulses shorter than this will be dropped
constexpr uint32_t DISSECTION_THRESHOLD =
    4.9e5L;  // longer pulses disected during polling.  0.49 seconds works okay for polling periods >=1s
PulseWidthAccumulateSensorStore::PulseWidthAccumulateSensorStore() { mux_ = portMUX_INITIALIZER_UNLOCKED; }

void PulseWidthAccumulateSensorStore::setup(InternalGPIOPin *pin) {
  pin->setup();
  this->pin_ = pin->to_isr();
  this->last_rise_us_ = micros();
  pin->attach_interrupt(&PulseWidthAccumulateSensorStore::gpio_intr, this, gpio::INTERRUPT_ANY_EDGE);
}

float PulseWidthAccumulateSensorStore::get_pulses_this_cycle() {
  uint32_t pulse_count = 0;
  // Safely copy & reset the pulse counter
  portENTER_CRITICAL(&this->mux_);
  pulse_count = this->pulse_count_;
  this->pulse_count_ = 0;
  portEXIT_CRITICAL(&this->mux_);
  float pulses_this_cycle = static_cast<float>(pulse_count);
  return static_cast<float>(pulses_this_cycle);
}

void PulseWidthAccumulateSensor::setup(void) {
  this->store_.setup(this->pin_);
  float interval_s = static_cast<float>(this->get_update_interval()) / 1000.0f;
  float short_pulse_threshold = (5 * interval_s < 1000.0f) ? 5 * interval_s : 1000.0f;
  float long_pulses_threshold = 2 * interval_s;
  this->rejection_threshold_ =
      (short_pulse_threshold > long_pulses_threshold) ? short_pulse_threshold : long_pulses_threshold;
  ESP_LOGW(TAG, "Rejection threshold set: %.1f s", this->rejection_threshold_);
}

float PulseWidthAccumulateSensorStore::get_cumulative_pulse_width_s() {
  float cumulative_local = 0;
  uint32_t last_rising_edge_local = 0;
  uint32_t last_falling_edge_local = 0;
  uint32_t cumulative_width_us_local = 0;
  // Fast section, interacting with an ISR running up to 10 KHz
  portENTER_CRITICAL(&this->mux_);
  cumulative_width_us_local = this->cumulative_width_us_;
  this->cumulative_width_us_ = 0;
  last_rising_edge_local = this->last_rise_us_;
  portEXIT_CRITICAL(&this->mux_);
  cumulative_local = static_cast<float>(cumulative_width_us_local) / 1e6f;
  // Slow section. Interacts with ISR at(1/DISSECTION_THRESHOLD) Hz
  if (micros() - last_rising_edge_local > DISSECTION_THRESHOLD) {
    // Measure GPIO directly in case startup occured while pin was high
    bool pulse_active = false;
    portENTER_CRITICAL(&this->mux_);
    pulse_active = this->pin_.digital_read();
    portEXIT_CRITICAL(&this->mux_);
    if (pulse_active) {
      uint32_t right_shift = micros() - last_rising_edge_local;
      portENTER_CRITICAL(&this->mux_);
      this->last_rise_us_ += right_shift;  // Subtract time-chunk from next ISR cycle
      portEXIT_CRITICAL(&this->mux_);
      cumulative_local += static_cast<float>(right_shift) / 1e6f;  // bring forward time-chunk to current polling cycle
    }
    // do nothing, falling edge interrupt has taken care of the accounting
  }
  return cumulative_local;
}

// ISR. Get in and out ASAP. No floating point math
void IRAM_ATTR PulseWidthAccumulateSensorStore::gpio_intr(PulseWidthAccumulateSensorStore *arg) {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&arg->mux_);
  if (arg->pin_.digital_read()) {
    // detected rising edge
    arg->last_rise_us_ = now;
  } else {
    // detected falling edge
    arg->last_fall_us_ = now;
    uint32_t pulse_width_us = now - arg->last_rise_us_;
    if (pulse_width_us > LOWER_PULSE_WIDTH_THRESHOLD) {
      arg->cumulative_width_us_ += pulse_width_us;
      arg->pulse_count_ += 1;
    }
  }
  portEXIT_CRITICAL_ISR(&arg->mux_);
}

void PulseWidthAccumulateSensor::dump_config() {
  LOG_SENSOR("", "Pulse Width", this)
  LOG_UPDATE_INTERVAL(this)
  LOG_PIN("  Pin: ", this->pin_);
}

void PulseWidthAccumulateSensor::update() {
  // Retrieve cumulative pulse width, and zero the counter
  float cumulative_width = this->store_.get_cumulative_pulse_width_s();
  float polling_interval_s = static_cast<float>(this->get_update_interval()) / 1000.0f;
  // Warn if outside normal range (0-103% of polling_interval)
  if (cumulative_width < 0 || cumulative_width > 1.03f * polling_interval_s) {
    ESP_LOGW(TAG, "Cumulative pulse width: %.3f s exceeded polling interval: %.3f ", cumulative_width,
             polling_interval_s);
  }
  // catch and fix errors so they dont corrupt the whole dataset
  if (cumulative_width > this->rejection_threshold_) {
    ESP_LOGW(TAG, "Discarding data: %.3f s Exceeds rejection threshold: %.3f ", cumulative_width,
             this->rejection_threshold_);
    cumulative_width = 0.0f;
  }

  // Calculate relative pulse width if relative mode is enabled
  if (this->relative_mode_) {
    cumulative_width = cumulative_width / polling_interval_s;  // Normalize to 0..1
    if (cumulative_width > 1.0f) {
      ESP_LOGW(TAG, "Relative pulse width exceeds 1.0: %.3f", cumulative_width);
      cumulative_width = 1.0f;  // Clamp to 1.0 if it exceeds
    }
  }

  // get frequency if needed
  if (this->frequency_sensor_ != nullptr) {
    float pulse_count = this->store_.get_pulses_this_cycle();
    float frequency = pulse_count / polling_interval_s;
    this->frequency_sensor_->publish_state(frequency);
  }
  this->publish_state(cumulative_width);
}

}  // namespace pulse_width_accumulate
}  // namespace esphome
