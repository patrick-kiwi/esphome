from esphome import pins
import esphome.codegen as cg
from esphome.components.esp32 import (
    VARIANT_ESP32,
    VARIANT_ESP32C3,
    VARIANT_ESP32C6,
    VARIANT_ESP32P4,
    VARIANT_ESP32S3,
    get_esp32_variant,
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
CONF_GPIO_NUMBER = "gpio_number"
CONF_PULSE_SEQUENCE = "pulse_sequence"
CONF_RESOLUTION_HZ = "resolution_hz"
CONF_ALIGN_PULSE_LENGTHS = "align_pulse_lengths"
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


def pin_channel_schema(value):
    """
    Schema for pin channel configuration. Supports two formats:
    1. Shorthand: pin_0: GPIO17
    2. Full: pin_0: {gpio_number: GPIO17, pulse_sequence: [...]}
    """
    if isinstance(value, dict) and CONF_GPIO_NUMBER in value:
        # Full format with gpio_number and optional pulse_sequence
        return cv.Schema(
            {
                cv.Required(CONF_GPIO_NUMBER): pins.internal_gpio_output_pin_schema,
                cv.Optional(CONF_PULSE_SEQUENCE): cv.All(
                    cv.ensure_list(RMT_SYMBOL_SCHEMA),
                    cv.Length(max=64),
                ),
            }
        )(value)
    # Shorthand format - just a pin
    return {CONF_GPIO_NUMBER: pins.internal_gpio_output_pin_schema(value)}


def extract_pin_number(pin_config):
    """Extract the GPIO number from a pin configuration."""
    gpio = pin_config[CONF_GPIO_NUMBER]
    # Extract pin number if it's a dict (GPIO object)
    if isinstance(gpio, dict) and "number" in gpio:
        return gpio["number"]
    return gpio


def validate_unique_pins(config):
    """Ensure all configured pins are unique."""
    pins_list = []
    for pin_key in [CONF_PIN_0, CONF_PIN_1, CONF_PIN_2, CONF_PIN_3]:
        if pin_key in config:
            pin_num = extract_pin_number(config[pin_key])
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


def validate_align_pulse_lengths(config):
    """Validate that align_pulse_lengths=false is only used on ESP32 (original)."""
    if not config.get(CONF_ALIGN_PULSE_LENGTHS, True):
        # align_pulse_lengths disabled - only ESP32 (original) supports this
        variant = get_esp32_variant()
        if variant != VARIANT_ESP32:
            raise cv.Invalid(
                f"align_pulse_lengths=false is only supported on ESP32 (original). "
                f"Current variant: {variant}. "
                f"All other variants (C3, C6, S3, P4) must align pulse lengths."
            )
    return config


# Main component schema
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RmtSimpleComponent),
            cv.Optional(CONF_PIN_0): pin_channel_schema,
            cv.Optional(CONF_PIN_1): pin_channel_schema,
            cv.Optional(CONF_PIN_2): pin_channel_schema,
            cv.Optional(CONF_PIN_3): pin_channel_schema,
            cv.Optional(CONF_RESOLUTION_HZ, default=1000000): cv.int_range(
                min=1, max=80000000
            ),
            cv.Optional(CONF_ALIGN_PULSE_LENGTHS, default=True): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_unique_pins,
    validate_at_least_one_pin,
    validate_align_pulse_lengths,
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

    # Set resolution and align_pulse_lengths
    cg.add(var.set_resolution(config[CONF_RESOLUTION_HZ]))
    cg.add(var.set_align_pulse_lengths(config[CONF_ALIGN_PULSE_LENGTHS]))

    # Determine if we should use sync manager (C3, C6, S3, P4 have sync hardware)
    variant = get_esp32_variant()
    use_sync_manager = variant in [
        VARIANT_ESP32C3,
        VARIANT_ESP32C6,
        VARIANT_ESP32P4,
        VARIANT_ESP32S3,
    ]
    cg.add(var.set_use_sync_manager(use_sync_manager))

    # Configure each channel
    for i in range(4):
        pin_key = f"pin_{i}"

        if pin_key in config:
            pin_config = config[pin_key]

            # Extract and configure the GPIO pin
            pin = await cg.gpio_pin_expression(pin_config[CONF_GPIO_NUMBER])
            cg.add(var.set_pin(i, pin))

            # Configure pulse sequence if provided
            if CONF_PULSE_SEQUENCE in pin_config:
                # Convert RMT symbols
                symbols = convert_symbols(pin_config[CONF_PULSE_SEQUENCE])

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
