# RMT Simple - Auto-Start Pulse Generator

A minimal ESP32 RMT (Remote Control) pulse generator component for ESPHome that automatically starts generating pulses on boot. This is a simplified version of `rmt_pulse_generator` with zero configuration overhead.

## Features

- **Auto-start on Boot**: Pulses begin automatically when the device powers up
- **Flexible Channel Support**: Use 1-4 channels (any combination)
- **Inline Configuration**: Define pulse patterns directly in the YAML
- **Direct RMT Symbol Format**: Define pulses using tick-based RMT symbols
- **No Manual Control**: Restart the device to change patterns
- **Minimal Codebase**: ~200 LOC total

## Supported Hardware

- ESP32-S3 (2 or 4 channels)
- ESP32-C3 (2 channels)
- ESP32-C6 (2 channels)

Requires ESP-IDF framework.

## Configuration

### Basic Example (Single Channel)

```yaml
rmt_simple:
  resolution_hz: 1000000  # 1MHz = 1 tick per microsecond

  pin_0: GPIO10
  pulses_0:
    - duration0: 50   # 50 ticks high
      level0: 1
      duration1: 50   # 50 ticks low
      level1: 0
```

This generates a 10 kHz square wave on GPIO10 (50µs high, 50µs low at 1MHz resolution).

### Two Channels

```yaml
rmt_simple:
  resolution_hz: 1000000  # 1MHz resolution

  pin_0: GPIO10
  pulses_0:
    - duration0: 50
      level0: 1
      duration1: 50
      level1: 0

  pin_1: GPIO11
  pulses_1:
    - duration0: 100
      level0: 1
      duration1: 100
      level1: 0
```

- Channel 0: 10 kHz square wave on GPIO10
- Channel 1: 5 kHz square wave on GPIO11

### Four Channels (ESP32-S3)

```yaml
rmt_simple:
  pin_0: GPIO10
  pulses_0:
    - duration0: 50
      level0: 1
      duration1: 50
      level1: 0

  pin_1: GPIO11
  pulses_1:
    - duration0: 100
      level0: 1
      duration1: 100
      level1: 0

  pin_2: GPIO12
  pulses_2:
    - duration0: 200
      level0: 1
      duration1: 200
      level1: 0

  pin_3: GPIO13
  pulses_3:
    - duration0: 400
      level0: 1
      duration1: 400
      level1: 0
```

### Multi-Symbol Patterns

Each channel supports up to 64 RMT symbols. Each symbol defines two level transitions:

```yaml
rmt_simple:
  pin_0: GPIO10
  pulses_0:
    # Symbol 1: HIGH(4 ticks) then LOW(1 tick)
    - duration0: 4
      level0: 1
      duration1: 1
      level1: 0
    # Symbol 2: LOW(5 ticks) then HIGH(2 ticks)
    - duration0: 5
      level0: 0
      duration1: 2
      level1: 1
    # Symbol 3: HIGH(3 ticks) then LOW(3 ticks)
    - duration0: 3
      level0: 1
      duration1: 3
      level1: 0
```

This creates a repeating pattern that cycles through all defined symbols.

### Creating Gaps/Dead Time

Use `duration: 0` to create gaps:

```yaml
rmt_simple:
  pin_0: GPIO10
  pulses_0:
    - duration0: 10
      level0: 1
      duration1: 0    # No transition, stays HIGH
      level1: 1
    - duration0: 5
      level0: 0
      duration1: 0    # No transition, stays LOW
      level1: 0
```

## Configuration Variables

- **pin_0** to **pin_3** (*Optional*): GPIO pins for each channel. At least one must be configured.
- **pulses_0** to **pulses_3** (*Optional*): Array of RMT symbols for each channel (max 64 symbols).
  - **duration0** (*Required*, int): Duration of first level in ticks (0-32767).
  - **level0** (*Required*, int): First level state (0 or 1).
  - **duration1** (*Required*, int): Duration of second level in ticks (0-32767).
  - **level1** (*Required*, int): Second level state (0 or 1).
- **resolution_hz** (*Optional*, int): RMT tick resolution in Hz. Range: 1-80000000. Default: 1000000 (1 MHz).

## Understanding RMT Symbols

Each RMT symbol (`rmt_symbol_word_t`) contains two level transitions. This allows compact representation of pulse patterns.

**Example Symbol:**
```yaml
- duration0: 10   # Stay at level0 for 10 ticks
  level0: 1       # Level is HIGH
  duration1: 5    # Stay at level1 for 5 ticks
  level1: 0       # Level is LOW
```

This single symbol produces: HIGH for 10 ticks → LOW for 5 ticks

## Resolution and Timing

The RMT peripheral uses tick-based timing. The resolution determines how long each tick lasts:

- **Tick Duration** = 1 / resolution_hz
- **Real Time** = duration_ticks / resolution_hz

**Constraints**:
- Duration range: 0-32767 ticks per level
- Level range: 0 (LOW) or 1 (HIGH)

**Examples at 1 MHz resolution** (1 tick = 1 µs):
- `duration: 50` = 50 ticks = 50 µs ✓
- `duration: 1000` = 1000 ticks = 1 ms ✓
- `duration: 32767` = 32.767 ms (maximum) ✓
- `duration: 40000` → ERROR (exceeds 32767)

**Examples at 80 MHz resolution** (1 tick = 12.5 ns):
- `duration: 80` = 80 ticks = 1 µs ✓
- `duration: 4000` = 4000 ticks = 50 µs ✓
- `duration: 32767` = 409.6 µs (maximum) ✓

## Auto-Start Behavior

The component automatically starts pulse generation during setup. This happens once when the device boots.

**To change pulse patterns**: Restart the device or reflash the firmware.

## Limitations

- **No Home Assistant Integration**: No actions, triggers, or automations
- **No Runtime Updates**: Cannot modify patterns without restart
- **No Named Sequences**: Only inline pulse definitions
- **Tick-Based Only**: Durations specified in ticks, not microseconds
- **No Manual Control**: Pulses always run (no start/stop)

## Migration from rmt_pulse_generator

If you're using `rmt_pulse_generator` and want to simplify:

### Before (rmt_pulse_generator)
```yaml
rmt_pulse_generator:
  pin_0: GPIO10
  pin_1: GPIO11
  resolution_hz: 1000000
  sequences:
    - id: my_pattern
      pulses:
        - high_us: 50
          low_us: 50

# In automation:
on_boot:
  then:
    - rmt_pulse_generator.begin:
        channel_0: my_pattern
        channel_1: my_pattern
```

### After (rmt_simple)
```yaml
rmt_simple:
  resolution_hz: 1000000

  pin_0: GPIO10
  pulses_0:
    - duration0: 50  # High for 50 ticks (50µs at 1MHz)
      level0: 1
      duration1: 50  # Low for 50 ticks
      level1: 0

  pin_1: GPIO11
  pulses_1:
    - duration0: 50
      level0: 1
      duration1: 50
      level1: 0
```

The pulses start automatically on boot. No automation needed.

**Note:** `rmt_pulse_generator` uses microsecond-based durations (`high_us`/`low_us`), while `rmt_simple` uses tick-based durations. Convert using: `ticks = microseconds × (resolution_hz / 1000000)`.

## Use Cases

Perfect for:
- Fixed frequency clock generation
- Simple PWM patterns
- Static test signals
- Embedded pulse generators
- Bit-banging protocols

**Not suitable for**:
- Dynamic pattern updates
- Home Assistant control
- Complex automation sequences

## License

Same as ESPHome (MIT License)

## Credits

Simplified fork of `rmt_pulse_generator` by @patrick-kiwi.
