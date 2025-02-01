#pragma once
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
namespace esphome {
namespace pulse_width_accumulate {


// Store data in a class that doesn't use multiple-inheritance (vtables in flash)
class PulseWidthAccumulateSensorStore {
 public:
  PulseWidthAccumulateSensorStore(); 
  void setup(InternalGPIOPin *pin);
  static void gpio_intr(PulseWidthAccumulateSensorStore *arg);
  float get_pulses_this_cycle();
  float get_cumulative_pulse_width_s();
  
 private:
  ISRInternalGPIOPin pin_;
  uint32_t last_rise_us_{0};
  uint32_t last_fall_us_{0};
  uint32_t cumulative_width_us_{0};
  bool pulse_in_progress_{false};
  float cumulative_width_s_{0.0f};
  uint32_t pulse_count_{0};
  portMUX_TYPE mux_;
};

class PulseWidthAccumulateSensor : public sensor::Sensor, public PollingComponent {
 public:
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void setup() override; 
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void update();
  void set_frequency_sensor(sensor::Sensor *frequency_sensor) { frequency_sensor_ = frequency_sensor; }

 private:
  float rejection_threshold_{1000.0f};
  PulseWidthAccumulateSensorStore store_;
  InternalGPIOPin *pin_;
  sensor::Sensor *frequency_sensor_{nullptr};
};
}  // namespace pulse_width_accumulate
}  // namespace esphome
