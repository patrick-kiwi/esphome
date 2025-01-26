#include "pulse_width_accumulate.h"
#include "esphome/core/log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace esphome {
namespace pulse_width_accumulate {

static const char *const TAG = "pulse_width";
constexpr uint32_t MICROSECOND_PER_PULSE_LOWER_THRESHOLD = 17;
PulseWidthAccumulateSensorStore::PulseWidthAccumulateSensorStore() {  mux_ = portMUX_INITIALIZER_UNLOCKED; }

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

// Zero the microsecond counter every polling cycle so we never overflow at 2^32 (ie. ~71.58 min)
float PulseWidthAccumulateSensorStore::get_cumulative_pulse_width_s() {
  float cumulative_local = 0;
  //handle long pulses that span beyond the polling window
portENTER_CRITICAL(&this->mux_);
  if ( this->pin_.digital_read() ) {
    uint32_t now = micros();
    cumulative_local = static_cast<float>( now - this->last_rise_us_ ) / 1e6f;
    this->last_rise_us_ = now;
  } else {
    cumulative_local = static_cast<float>(this->cumulative_width_us_) / 1e6f;
    this->cumulative_width_us_ = 0;
  }
portEXIT_CRITICAL(&this->mux_);
  return cumulative_local;
}

// ISR. Get in and out ASAP.  No floating point math!
void IRAM_ATTR PulseWidthAccumulateSensorStore::gpio_intr(PulseWidthAccumulateSensorStore *arg) {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&arg->mux_);
  bool pin_state = arg->pin_.digital_read();
  if (pin_state) { 
    // Rising edge detected
    arg->last_rise_us_ = now;
  } else { 
    // Falling edge detected
    uint32_t pulse_width_us = now - arg->last_rise_us_;
    // Filter noise and accumulate
    if (pulse_width_us > MICROSECOND_PER_PULSE_LOWER_THRESHOLD) {
      arg->cumulative_width_us_+= pulse_width_us;
      arg->pulse_count_+= 1;
    }
  }
portEXIT_CRITICAL_ISR(&arg->mux_);
}
  
// ESPHome Component Methods
void PulseWidthAccumulateSensor::dump_config() {
  LOG_SENSOR("", "Pulse Width", this)
  LOG_UPDATE_INTERVAL(this)
  LOG_PIN("  Pin: ", this->pin_);
}


void PulseWidthAccumulateSensor::update() {
  //Retrieve cumulative pulse width, and zero the counter
  float cumulative_width = this->store_.get_cumulative_pulse_width_s();
  float polling_interval_s = static_cast<float>(this->get_update_interval()) / 1000.0f;

  //Check and fix errors, and issue warnings if necessary.
  if (polling_interval_s > 4294.9f) {
    ESP_LOGW(TAG, "Error! Polling interval: %.1f s exceeds 71.58 min. Microseconds will overflow if pw > 71.58 min", polling_interval_s);
  }
  //Clamp cumulative width to valid range, 
  if (cumulative_width < 0) {
    ESP_LOGW(TAG, "Warning, cumulative pulse width %.1f s doesn't make sense! Setting to zero.", cumulative_width);
    cumulative_width = 0.0f;
  } 
  if (cumulative_width > polling_interval_s) {
    ESP_LOGW(TAG, "Warning, cumulative pulse width: %.4f s exceeds the polling window: %.4f s.", cumulative_width, polling_interval_s);
    /*Always issue a warning, but only attempt to fix if there's a large deviation
    Experimental observation: When the GPIO input is continuously on we often get a self correcting error.  
    Specifically one polling period will overshoot by 1 or 2 ms, and the next will be deficiant by the same amount
    we dont want to introduce any systemic inaccuracy by correcting when we shouldn't*/
    if ( (cumulative_width - polling_interval_s) > 3.0e-3f ) {
      ESP_LOGW(TAG, "Clamping cumulative pulse width to range: %.4f", polling_interval_s);
      cumulative_width = polling_interval_s;
    }
    
  }

  //get frequency if needed
  if (this->frequency_sensor_ != nullptr) {
    float pulse_count = this->store_.get_pulses_this_cycle();
    float frequency = pulse_count / polling_interval_s;
    this->frequency_sensor_->publish_state(frequency);
  }

  this->publish_state(cumulative_width);
}

}  // namespace pulse_width_accumulate
}  // namespace esphome
