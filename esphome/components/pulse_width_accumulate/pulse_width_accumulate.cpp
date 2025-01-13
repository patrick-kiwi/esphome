#include "pulse_width_accumulate.h"
#include "esphome/core/log.h"
#include <algorithm>

namespace esphome {
namespace pulse_width_accumulate {

static const char *const TAG = "pulse_width";

// ~Threshold for microsecond (71 minutes) overflow in milliseconds
constexpr uint32_t MICROSECOND_OVERFLOW_THRESHOLD_MS = 4260000;

// Simplified ISR: Only sets flag
void IRAM_ATTR PulseWidthAccumulateSensorStore::gpio_intr(PulseWidthAccumulateSensorStore *arg) {
  arg->new_edge_detected_ = true;
}

// FreeRTOS task: Processes pulses continuously
void PulseWidthAccumulateSensorStore::continuous_task(void *pv) {
  auto *store = static_cast<PulseWidthAccumulateSensorStore *>(pv);
  while (true) {
    if (store->new_edge_detected_) {     // Detected a rising or falling edge
      if (store->pin_.digital_read()) {  // Rising edge
        store->pulse_in_progress_ = true;
        store->last_rise_us_ = micros();  // Record timestamp (microseconds)
        store->last_rise_ms_ = millis();  // Record timestamp (milliseconds)
      } else {                            // Falling edge
        store->pulse_in_progress_ = false;
        store->last_fall_us_ = micros();  // Record timestamp (microseconds)
        store->last_fall_ms_ = millis();  // Record timestamp (milliseconds)

        // Calculate pulse width
        uint32_t pulse_width_us = store->last_fall_us_ - store->last_rise_us_;
        uint32_t pulse_width_ms = store->last_fall_ms_ - store->last_rise_ms_;

        // Add to cumulative width
        if (pulse_width_ms > MICROSECOND_OVERFLOW_THRESHOLD_MS) {
          store->cumulative_width_s_.fetch_add(static_cast<float>(pulse_width_ms) / 1e3f);
        } else {
          store->cumulative_width_s_.fetch_add(static_cast<float>(pulse_width_us) / 1e6f);
        }
      }
      store->new_edge_detected_ = false;  // Reset flag
    }

    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(1));  // 1 us delay
  }
}

// ESPHome Component Methods
void PulseWidthAccumulateSensor::dump_config() {
  LOG_SENSOR("", "Pulse Width", this)
  LOG_UPDATE_INTERVAL(this)
  LOG_PIN("  Pin: ", this->pin_);
}

void PulseWidthAccumulateSensor::update() {
  float cumulative_width = this->store_.get_cumulative_pulse_width_s();  // Retrieve cumulative pulse width

  // Handle long in-progress pulses
  if (this->store_.is_pulse_in_progress()) {
    uint32_t elapsed_ms = millis() - this->store_.get_last_rise_ms();

    // Check if the pulse has been in progress for more than 1 second
    if (elapsed_ms > 1000) {  // 1000 ms = 1 second
      cumulative_width += static_cast<float>(elapsed_ms) / 1e3f;

      // Update "pulse start time" to now, as we’ve recorded a chunk of the pulse
      this->store_.set_last_rise_ms(millis());
      this->store_.set_last_rise_us(micros());
    }
  }

  // Get the polling interval in seconds
  float polling_interval_s = 2.0f * static_cast<float>(this->get_update_interval()) / 1000.0f;

  /*
  Sanity check: Ensure cumulative width is within valid range
  A bouncing signal can generate microsecond pulses which somehow break the code and cause massive cumulative widths
  However, the last pulse may have initiated just after the previous polling interval so we have to use a 2x the polling
  window)
  */
  if (cumulative_width < 0 || cumulative_width > polling_interval_s) {
    ESP_LOGW(TAG, "Cumulative pulse width %.1f s exceeds polling interval %.1f s. Clamping to range.", cumulative_width,
             polling_interval_s);
    cumulative_width = std::clamp(cumulative_width, 0.0f, polling_interval_s);
  }

  /*Because Home Assistant will only update the sensor if there's a **NEW** number, then repeated 60s ontimes are a big
  problem because we'll miss them. We should put some 3rd decimal place random noise in ontimes greater than 57 seconds
  I couldn't figure out how to use the random library here, so I've hacked together a random number out of micros().
  */
  if (cumulative_width > (polling_interval_s - 3.0f)) {
    uint32_t random_micros = micros();
    uint32_t random_micros_inverted = 0;
    for (uint8_t i = 1; i <= 4; ++i) {
      uint32_t digit = (random_micros % 10);
      random_micros_inverted = random_micros_inverted * 10 + digit;
      random_micros /= 10;
    }
    float pseudo_rand = static_cast<float>(random_micros_inverted) / 1e6f;
    cumulative_width -= pseudo_rand;
  }

  ESP_LOGCONFIG(TAG, "'%s' - Cumulative pulse width: %.5f s", this->name_.c_str(), cumulative_width);
  this->publish_state(cumulative_width);
}

}  // namespace pulse_width_accumulate
}  // namespace esphome
