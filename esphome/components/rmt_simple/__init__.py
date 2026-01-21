from esphome import pins
import esphome.codegen as cg
from esphome.components.esp32 import (
    VARIANT_ESP32C3,
    VARIANT_ESP32C6,
    VARIANT_ESP32S3,
    only_on_variant,
)
import esphome.config_validation as cv
from esphome.const import CONF_ID

# Dependencies
DEPENDENCIES = ["esp32"]
CODEOWNERS = ["@patrick-kiwi"]

# Constants
CONF_PIN_0 = "pin_0"
CONF_PIN_1 = "pin_1"
CONF_PIN_2 = "pin_2"
CONF_PIN_3 = "pin_3"
CONF_PULSES_0 = "pulses_0"
CONF_PULSES_1 = "pulses_1"
CONF_PULSES_2 = "pulses_2"
CONF_PULSES_3 = "pulses_3"
CONF_RESOLUTION_HZ = "resolution_hz"
CONF_DURATION0 = "duration0"
CONF_LEVEL0 = "level0"
CONF_DURATION1 = "duration1"
CONF_LEVEL1 = "level1"

# Namespace
rmt_simple_ns = cg.esphome_ns.namespace("rmt_simple")
RmtSimpleComponent = rmt_simple_ns.class_("RmtSimpleComponent", cg.Component)


# RMT symbol schema (direct tick-based format)
RMT_SYMBOL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_DURATION0): cv.int_range(min=0, max=32767),
        cv.Required(CONF_LEVEL0): cv.int_range(min=0, max=1),
        cv.Required(CONF_DURATION1): cv.int_range(min=0, max=32767),
        cv.Required(CONF_LEVEL1): cv.int_range(min=0, max=1),
    }
)


def validate_unique_pins(config):
    """Ensure all configured pins are unique."""
    pins_list = []
    for pin_key in [CONF_PIN_0, CONF_PIN_1, CONF_PIN_2, CONF_PIN_3]:
        if pin_key in config:
            pin_num = config[pin_key]
            # Extract pin number if it's a dict (GPIO object)
            if isinstance(pin_num, dict) and "number" in pin_num:
                pin_num = pin_num["number"]
            if pin_num in pins_list:
                raise cv.Invalid(
                    f"GPIO{pin_num} is assigned to multiple channels. "
                    f"Each channel requires a unique pin."
                )
            pins_list.append(pin_num)
    return config


def validate_at_least_one_pin(config):
    """Ensure at least one pin is configured."""
    has_pin = any(
        pin_key in config
        for pin_key in [CONF_PIN_0, CONF_PIN_1, CONF_PIN_2, CONF_PIN_3]
    )
    if not has_pin:
        raise cv.Invalid(
            "At least one pin must be configured (pin_0, pin_1, pin_2, or pin_3)"
        )
    return config


# Main component schema
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RmtSimpleComponent),
            cv.Optional(CONF_PIN_0): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_PIN_1): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_PIN_2): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_PIN_3): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_PULSES_0): cv.All(
                cv.ensure_list(RMT_SYMBOL_SCHEMA),
                cv.Length(max=64),
            ),
            cv.Optional(CONF_PULSES_1): cv.All(
                cv.ensure_list(RMT_SYMBOL_SCHEMA),
                cv.Length(max=64),
            ),
            cv.Optional(CONF_PULSES_2): cv.All(
                cv.ensure_list(RMT_SYMBOL_SCHEMA),
                cv.Length(max=64),
            ),
            cv.Optional(CONF_PULSES_3): cv.All(
                cv.ensure_list(RMT_SYMBOL_SCHEMA),
                cv.Length(max=64),
            ),
            cv.Optional(CONF_RESOLUTION_HZ, default=1000000): cv.int_range(
                min=1, max=80000000
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    only_on_variant(supported=[VARIANT_ESP32C3, VARIANT_ESP32C6, VARIANT_ESP32S3]),
    validate_unique_pins,
    validate_at_least_one_pin,
)


def convert_symbols(symbols):
    """Convert RMT symbol config to internal format."""
    return [
        {
            "duration0": symbol[CONF_DURATION0],
            "level0": symbol[CONF_LEVEL0],
            "duration1": symbol[CONF_DURATION1],
            "level1": symbol[CONF_LEVEL1],
        }
        for symbol in symbols
    ]


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Set resolution
    cg.add(var.set_resolution(config[CONF_RESOLUTION_HZ]))

    # Configure each channel
    for i in range(4):
        pin_key = f"pin_{i}"
        pulses_key = f"pulses_{i}"

        if pin_key in config:
            pin = await cg.gpio_pin_expression(config[pin_key])
            cg.add(var.set_pin(i, pin))

            if pulses_key in config:
                # Convert RMT symbols
                symbols = convert_symbols(config[pulses_key])

                # Generate C++ std::vector constructor
                if symbols:
                    symbols_str = "std::vector<rmt_symbol_word_t>({\n"
                    for sym in symbols:
                        symbols_str += "        rmt_symbol_word_t{{\n"
                        symbols_str += f"          .duration0 = {sym['duration0']},\n"
                        symbols_str += f"          .level0 = {sym['level0']},\n"
                        symbols_str += f"          .duration1 = {sym['duration1']},\n"
                        symbols_str += f"          .level1 = {sym['level1']},\n"
                        symbols_str += "        }},\n"
                    symbols_str += "      })"
                else:
                    # Empty vector
                    symbols_str = "std::vector<rmt_symbol_word_t>()"

                symbols_vector = cg.RawExpression(symbols_str)
                cg.add(var.set_pulses(i, symbols_vector))
