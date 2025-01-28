#include "esphome/core/log.h"
#include "pulse_width_accumulate.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace esphome {
namespace pulse_width_accumulate {
static const char *const TAG = "pulse_width";
constexpr uint32_t MICROSECOND_PER_PULSE_LOWER_THRESHOLD = 17;
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

// Zero the microsecond counter every polling cycle so we never overflow at 2^32
// (ie. ~71.58 min)
float PulseWidthAccumulateSensorStore::get_cumulative_pulse_width_s() {
  float cumulative_local = 0;
  
  // handle long pulses that span beyond the polling window
  if (this->pin_.digital_read()) {
    portENTER_CRITICAL(&this->mux_);
    cumulative_local = static_cast<float>(now - this->last_rise_us_) / 1e6f;
    this->last_rise_us_ = now;
    portEXIT_CRITICAL(&this->mux_);
  } else {
    portENTER_CRITICAL(&this->mux_);
    cumulative_local = static_cast<float>(this->cumulative_width_us_) / 1e6f;
    this->cumulative_width_us_ = 0;
    portEXIT_CRITICAL(&this->mux_);
  }
  
  return cumulative_local;
}

// ISR. Get in and out ASAP. No floating point math
void IRAM_ATTR PulseWidthAccumulateSensorStore::gpio_intr(PulseWidthAccumulateSensorStore *arg) {
  uint32_t now = micros();
  if (arg->pin_.digital_read()) {
    portENTER_CRITICAL_ISR(&arg->mux_);
    arg->last_rise_us_ = now;
    portEXIT_CRITICAL_ISR(&arg->mux_);
  } else {
    portENTER_CRITICAL_ISR(&arg->mux_);
    uint32_t pulse_width_us = now - arg->last_rise_us_;
    if (pulse_width_us > MICROSECOND_PER_PULSE_LOWER_THRESHOLD) {
      arg->cumulative_width_us_ += pulse_width_us;
      arg->pulse_count_ += 1;
    }
    portEXIT_CRITICAL_ISR(&arg->mux_);
  }
  
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
    ESP_LOGW(TAG, "Warning, cumulative pulse width: %.3f s ouside expected range: %.3f ", cumulative_width,
             polling_interval_s);
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
