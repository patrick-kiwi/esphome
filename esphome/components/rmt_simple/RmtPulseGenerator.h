#pragma once

#include <driver/rmt_tx.h>
#include <vector>
#include <utility>  // for std::pair

// Utility to count total ticks in a pulse pattern
inline uint32_t total_ticks(const rmt_symbol_word_t *pattern, size_t len) {
  uint32_t total = 0;
  for (size_t i = 0; i < len; ++i) {
    total += pattern[i].duration0 + pattern[i].duration1;
  }
  return total;
}

// Utility to pad a pattern with dummy ticks to reach target length
inline std::vector<rmt_symbol_word_t> pad_pattern(const rmt_symbol_word_t *pattern, size_t len, uint32_t pad_ticks) {
  std::vector<rmt_symbol_word_t> result(pattern, pattern + len);
  if (pad_ticks > 0) {
    result.push_back({.duration0 = static_cast<uint16_t>(pad_ticks), .level0 = 0, .duration1 = 0, .level1 = 0});
  }
  return result;
}

// Utility to align multiple patterns to the same length by padding shorter ones
// This does NOT provide timing synchronization - use sync manager for that
inline std::vector<std::vector<rmt_symbol_word_t>> align_pulse_lengths(
    const std::vector<std::vector<rmt_symbol_word_t>> &patterns) {
  if (patterns.empty()) {
    return {};
  }

  // Find maximum tick count across all patterns
  uint32_t max_ticks = 0;
  for (const auto &pattern : patterns) {
    uint32_t ticks = total_ticks(pattern.data(), pattern.size());
    if (ticks > max_ticks) {
      max_ticks = ticks;
    }
  }

  // Pad all patterns to match the longest
  std::vector<std::vector<rmt_symbol_word_t>> aligned;
  aligned.reserve(patterns.size());
  for (const auto &pattern : patterns) {
    uint32_t ticks = total_ticks(pattern.data(), pattern.size());
    uint32_t pad_ticks = max_ticks - ticks;
    aligned.push_back(pad_pattern(pattern.data(), pattern.size(), pad_ticks));
  }

  return aligned;
}

/**
 * Base class for RMT pulse generation
 * Supports both 2-channel and 4-channel variants via template specialization
 */
template<int NumChannels> class RmtPulseGenerator {
  static_assert(NumChannels == 2 || NumChannels == 4, "Only 2 or 4 channels supported");
};

/**
 * 2-Channel RMT Pulse Generator
 * For ESP32-C3, ESP32-C6
 */
template<> class RmtPulseGenerator<2> {
 public:
  RmtPulseGenerator(gpio_num_t gpioA, gpio_num_t gpioB, uint32_t resolution_hz = 1000000,
                    bool align_pulse_lengths = true, bool use_sync_manager = true)
      : resolution_hz_(resolution_hz), align_pulse_lengths_(align_pulse_lengths), use_sync_manager_(use_sync_manager) {
    tx_gpio_number[0] = gpioA;
    tx_gpio_number[1] = gpioB;
    tx_channels[0] = nullptr;
    tx_channels[1] = nullptr;
    copyEncoder = nullptr;
    sync_mgr = nullptr;
    running = false;
  }

  ~RmtPulseGenerator() {
    stop();
    cleanup();
  }

  bool init() {
    // ESP32 (original) needs different settings than C3/C6/P4/S3
    uint16_t mem_block_symbols = use_sync_manager_ ? 48 : 64;
    uint8_t trans_queue_depth = use_sync_manager_ ? 8 : 1;

    for (int i = 0; i < 2; ++i) {
      rmt_tx_channel_config_t config = {.gpio_num = tx_gpio_number[i],
                                        .clk_src = RMT_CLK_SRC_DEFAULT,
                                        .resolution_hz = resolution_hz_,
                                        .mem_block_symbols = mem_block_symbols,
                                        .trans_queue_depth = trans_queue_depth,
                                        .flags = {}};
      if (rmt_new_tx_channel(&config, &tx_channels[i]) != ESP_OK) {
        return false;
      }
      if (rmt_enable(tx_channels[i]) != ESP_OK) {
        return false;
      }
    }

    // Create sync manager for boards that support it (C3, C6, P4, S3)
    if (use_sync_manager_) {
      rmt_sync_manager_config_t sync_config = {.tx_channel_array = tx_channels, .array_size = 2};
      if (rmt_new_sync_manager(&sync_config, &sync_mgr) != ESP_OK) {
        return false;
      }
    }

    rmt_copy_encoder_config_t enc_config = {};
    if (rmt_new_copy_encoder(&enc_config, &copyEncoder) != ESP_OK) {
      return false;
    }

    return true;
  }

  bool begin(const std::vector<std::vector<rmt_symbol_word_t>> &patterns) {
    if (patterns.size() != 2) {
      return false;
    }

    // Conditionally align pulse lengths based on configuration
    std::vector<std::vector<rmt_symbol_word_t>> aligned_patterns;
    if (align_pulse_lengths_) {
      aligned_patterns = align_pulse_lengths(patterns);
    } else {
      // Use patterns as-is without length alignment
      aligned_patterns = patterns;
    }

    // Store aligned patterns
    current_patterns[0] = aligned_patterns[0];
    current_patterns[1] = aligned_patterns[1];

    rmt_transmit_config_t tx_config = {.loop_count = -1};

    if (rmt_transmit(tx_channels[0], copyEncoder, current_patterns[0].data(),
                     current_patterns[0].size() * sizeof(rmt_symbol_word_t), &tx_config) != ESP_OK) {
      return false;
    }
    if (rmt_transmit(tx_channels[1], copyEncoder, current_patterns[1].data(),
                     current_patterns[1].size() * sizeof(rmt_symbol_word_t), &tx_config) != ESP_OK) {
      return false;
    }

    running = true;
    return true;
  }

  bool update(const std::vector<std::vector<rmt_symbol_word_t>> &patterns) {
    stop();
    return begin(patterns);
  }

  void stop() {
    if (running) {
      for (int i = 0; i < 2; ++i) {
        if (tx_channels[i] != nullptr) {
          rmt_disable(tx_channels[i]);
        }
      }
      running = false;
    }

    // Re-enable for next use
    for (int i = 0; i < 2; ++i) {
      if (tx_channels[i] != nullptr) {
        rmt_enable(tx_channels[i]);
      }
    }
  }

  bool is_running() const { return running; }

 private:
  void cleanup() {
    if (copyEncoder != nullptr) {
      rmt_del_encoder(copyEncoder);
      copyEncoder = nullptr;
    }
    if (sync_mgr != nullptr) {
      rmt_del_sync_manager(sync_mgr);
      sync_mgr = nullptr;
    }
    for (int i = 0; i < 2; ++i) {
      if (tx_channels[i] != nullptr) {
        rmt_disable(tx_channels[i]);
        rmt_del_channel(tx_channels[i]);
        tx_channels[i] = nullptr;
      }
    }
  }

  rmt_channel_handle_t tx_channels[2];
  gpio_num_t tx_gpio_number[2];
  rmt_encoder_handle_t copyEncoder;
  rmt_sync_manager_handle_t sync_mgr;
  uint32_t resolution_hz_;
  bool align_pulse_lengths_;
  bool use_sync_manager_;
  bool running;
  std::vector<rmt_symbol_word_t> current_patterns[2];
};

/**
 * 4-Channel RMT Pulse Generator
 * For ESP32-S3, ESP32-P4
 */
template<> class RmtPulseGenerator<4> {
 public:
  RmtPulseGenerator(gpio_num_t gpioA, gpio_num_t gpioB, gpio_num_t gpioC, gpio_num_t gpioD,
                    uint32_t resolution_hz = 1000000, bool align_pulse_lengths = true)
      : resolution_hz_(resolution_hz), align_pulse_lengths_(align_pulse_lengths) {
    tx_gpio_number[0] = gpioA;
    tx_gpio_number[1] = gpioB;
    tx_gpio_number[2] = gpioC;
    tx_gpio_number[3] = gpioD;
    for (int i = 0; i < 4; ++i) {
      tx_channels[i] = nullptr;
    }
    copyEncoder = nullptr;
    sync_mgr = nullptr;
    running = false;
  }

  ~RmtPulseGenerator() {
    stop();
    cleanup();
  }

  bool init() {
    // 4-channel variants always have sync manager support
    for (int i = 0; i < 4; ++i) {
      rmt_tx_channel_config_t config = {.gpio_num = tx_gpio_number[i],
                                        .clk_src = RMT_CLK_SRC_DEFAULT,
                                        .resolution_hz = resolution_hz_,
                                        .mem_block_symbols = 48,
                                        .trans_queue_depth = 8,
                                        .flags = {}};
      if (rmt_new_tx_channel(&config, &tx_channels[i]) != ESP_OK) {
        return false;
      }
      if (rmt_enable(tx_channels[i]) != ESP_OK) {
        return false;
      }
    }

    // Create sync manager for reliable edge alignment
    rmt_sync_manager_config_t sync_config = {.tx_channel_array = tx_channels, .array_size = 4};
    if (rmt_new_sync_manager(&sync_config, &sync_mgr) != ESP_OK) {
      return false;
    }

    rmt_copy_encoder_config_t enc_config = {};
    if (rmt_new_copy_encoder(&enc_config, &copyEncoder) != ESP_OK) {
      return false;
    }

    return true;
  }

  bool begin(const std::vector<std::vector<rmt_symbol_word_t>> &patterns) {
    if (patterns.size() != 4) {
      return false;
    }

    // Align pulse lengths - 4-channel variants must always align
    // (experimentally determined requirement)
    std::vector<std::vector<rmt_symbol_word_t>> aligned_patterns;
    if (align_pulse_lengths_) {
      aligned_patterns = align_pulse_lengths(patterns);
    } else {
      aligned_patterns = patterns;
    }

    // Store aligned patterns
    for (int i = 0; i < 4; ++i) {
      current_patterns[i] = aligned_patterns[i];
    }

    rmt_transmit_config_t tx_config = {.loop_count = -1};

    for (int i = 0; i < 4; ++i) {
      if (rmt_transmit(tx_channels[i], copyEncoder, current_patterns[i].data(),
                       current_patterns[i].size() * sizeof(rmt_symbol_word_t), &tx_config) != ESP_OK) {
        return false;
      }
    }

    running = true;
    return true;
  }

  bool update(const std::vector<std::vector<rmt_symbol_word_t>> &patterns) {
    stop();
    return begin(patterns);
  }

  void stop() {
    if (running) {
      for (int i = 0; i < 4; ++i) {
        if (tx_channels[i] != nullptr) {
          rmt_disable(tx_channels[i]);
        }
      }
      running = false;
    }

    // Re-enable for next use
    for (int i = 0; i < 4; ++i) {
      if (tx_channels[i] != nullptr) {
        rmt_enable(tx_channels[i]);
      }
    }
  }

  bool is_running() const { return running; }

 private:
  void cleanup() {
    if (copyEncoder != nullptr) {
      rmt_del_encoder(copyEncoder);
      copyEncoder = nullptr;
    }
    if (sync_mgr != nullptr) {
      rmt_del_sync_manager(sync_mgr);
      sync_mgr = nullptr;
    }
    for (int i = 0; i < 4; ++i) {
      if (tx_channels[i] != nullptr) {
        rmt_disable(tx_channels[i]);
        rmt_del_channel(tx_channels[i]);
        tx_channels[i] = nullptr;
      }
    }
  }

  rmt_channel_handle_t tx_channels[4];
  gpio_num_t tx_gpio_number[4];
  rmt_encoder_handle_t copyEncoder;
  rmt_sync_manager_handle_t sync_mgr;
  uint32_t resolution_hz_;
  bool align_pulse_lengths_;
  bool running;
  std::vector<rmt_symbol_word_t> current_patterns[4];
};
