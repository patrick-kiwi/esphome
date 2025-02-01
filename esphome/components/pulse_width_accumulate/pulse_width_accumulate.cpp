#include "esphome/core/log.h"
#include "pulse_width_accumulate.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace esphome {
namespace pulse_width_accumulate {
static const char *const TAG = "pulse_width";
constexpr uint32_t LOWER_PULSE_WIDTH_THRESHOLD = 17;  //pulses shorter than this will be dropped
constexpr uint32_t DISECTION_THRESHOLD = 4.5e5L;  //pulses longer than this will be disected during polling
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
  float short_pulse_threshold = (5 * interval_s < 1000.0f) ? 5 * interval_s: 1000.0f;
  float long_pulses_threshold = 2*interval_s;
  this->rejection_threshold_ = (short_pulse_threshold > long_pulses_threshold) ? short_pulse_threshold : long_pulses_threshold;
  ESP_LOGW(TAG, "Rejection threshold set: %.1f s", this->rejection_threshold_);
}

// Zero the microsecond counter every polling cycle so we never overflow at 2^32
// (ie. ~71.58 min)
float PulseWidthAccumulateSensorStore::get_cumulative_pulse_width_s() {
  float cumulative_local = 0;
  uint32_t now = micros();
  uint32_t dissection_threshold = 1e6;  // 1 second
  portENTER_CRITICAL(&this->mux_);
  if (this->pulse_in_progress_) {
    uint32_t pulse_duration = now - this->last_rise_us_;
    ESP_LOGW(TAG, "Difference: %u, Polling interval: %u", pulse_duration, dissection_threshold);
    if (pulse_duration >= dissection_threshold) {
      // GPIO is continuously HIGH, break time into chunks
      cumulative_local = static_cast<float>(dissection_threshold) / 1e6f;
      ESP_LOGW(TAG, "Dissecting out time: %.1f s", cumulative_local);
      // Move the reference point forward by 1 second
      this->last_rise_us_ += dissection_threshold;
      this->cumulative_width_us_ -= dissection_threshold;
    } else {
      // Assume a short pulse or unfinished long pulse
      cumulative_local = static_cast<float>(this->cumulative_width_us_) / 1e6f;
      this->cumulative_width_us_ = 0;
    }
  } else {
    // Input LOW case
    cumulative_local = static_cast<float>(this->cumulative_width_us_) / 1e6f;
    this->cumulative_width_us_ = 0;
  }
  portEXIT_CRITICAL(&this->mux_);
  return cumulative_local;
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
