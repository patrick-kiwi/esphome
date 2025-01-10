#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>

namespace esphome {
namespace pulse_width_accumulate {

class PulseWidthAccumulateSensorStore {
 public:
  void setup(InternalGPIOPin *pin) {
    pin->setup();
    this->pin_ = pin->to_isr();
    this->last_rise_us_ = micros();
    this->last_rise_ms_ = millis();
    this->new_edge_detected_ = false;  // Initialize flag
    this->pulse_in_progress_ = false;  // Initialize flag
    pin->attach_interrupt(&PulseWidthAccumulateSensorStore::gpio_intr, this, gpio::INTERRUPT_ANY_EDGE);
  }

  static void gpio_intr(PulseWidthAccumulateSensorStore *arg);

  float get_cumulative_pulse_width_s() {
    // Atomically fetch the current value and reset it to zero
    return this->cumulative_width_s_.exchange(0.0f);
  }

  // Getter for last_rise_ms_
  uint32_t get_last_rise_ms() const { return this->last_rise_ms_; }

  // Setter for last_rise_ms_
  void set_last_rise_ms(uint32_t value) { this->last_rise_ms_ = value; }

  // Setter for last_rise_us_
  void set_last_rise_us(uint32_t value) { this->last_rise_us_ = value; }

  uint32_t get_last_rise_ms_() { return this->last_rise_ms_; }

  bool is_pulse_in_progress() const { return this->pulse_in_progress_; }

  static void continuous_task(void *pv);

 private:
  ISRInternalGPIOPin pin_;
  uint32_t last_rise_ms_{0};                // Timestamp of last rising edge
  uint32_t last_fall_ms_{0};                // Timestamp of last falling edge
  uint32_t last_rise_us_{0};                // Timestamp of last rising edge
  uint32_t last_fall_us_{0};                // Timestamp of last falling edge
  volatile bool new_edge_detected_{false};  // Flag for edge detection
  volatile bool pulse_in_progress_{false};  // Flag for pulse detection
                                            // float cumulative_width_s_{0.0f};
 protected:
  std::atomic<float> cumulative_width_s_{0.0f};  // Total cumulative pulse width
};

class PulseWidthAccumulateSensor : public sensor::Sensor, public PollingComponent {
 public:
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }

  void setup() override {
    this->store_.setup(this->pin_);
    // this->set_update_interval(1000);   //temp set polling to 1s

    // Initialize RTOS task
    xTaskCreatePinnedToCore(PulseWidthAccumulateSensorStore::continuous_task,  // Task function
                            "continuous_task",                                 // Task name
                            1024,                                              // Stack size
                            &(this->store_),                                   // Task parameter
                            1,                                                 // Priority
                            nullptr,                                           // Task handle
                            1                                                  // Core
    );
  }

  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void update();

 private:
  PulseWidthAccumulateSensorStore store_;
  InternalGPIOPin *pin_;
};

}  // namespace pulse_width_accumulate
}  // namespace esphome
