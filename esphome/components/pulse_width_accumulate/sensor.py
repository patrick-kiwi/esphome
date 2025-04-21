from esphome import pins
import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_PIN,
    ICON_TIMER,
    ICON_PULSE,
    STATE_CLASS_TOTAL_INCREASING,
    CONF_FREQUENCY,
    STATE_CLASS_MEASUREMENT,
    UNIT_SECOND,
    UNIT_HERTZ,
)

CONF_RELATIVE = "relative"

pulse_width_ns = cg.esphome_ns.namespace("pulse_width_accumulate")

PulseWidthAccumulateSensor = pulse_width_ns.class_(
    "PulseWidthAccumulateSensor", sensor.Sensor, cg.PollingComponent
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        PulseWidthAccumulateSensor,
        unit_of_measurement=UNIT_SECOND,
        icon=ICON_TIMER,
        accuracy_decimals=2,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    )
    .extend(
        {
            cv.Required(CONF_PIN): cv.All(pins.internal_gpio_input_pin_schema),
            cv.Optional(CONF_FREQUENCY): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                accuracy_decimals=1,
                icon=ICON_PULSE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_RELATIVE, default=False): cv.boolean,
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    # Register the main sensor
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    # Configure the GPIO pin
    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    # Register the frequency sub-sensor if configured
    if conf_freq := config.get(CONF_FREQUENCY):
        sens = await sensor.new_sensor(conf_freq)
        cg.add(var.set_frequency_sensor(sens))
    # Pass the relative option to the C++ code
    if config[CONF_RELATIVE]:
        cg.add(var.set_relative_mode(True))
