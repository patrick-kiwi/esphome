#pragma once

#include <driver/rmt_tx.h>
#include <vector>
#include <utility>  // for std::pair

namespace esphome {
namespace rmt_simple {

// Utility to count total ticks in a pulse pattern
inline uint32_t total_ticks(const rmt_symbol_word_t *pattern, size_t len) {
  uint32_t total = 0;
  for (size_t i = 0; i < len; ++i) {
    total += pattern[i].duration0 + pattern[i].duration1;
  }
  return total;
}

// Utility to pad a pattern with dummy ticks to equalize timing
inline std::vector<rmt_symbol_word_t> pad_pattern(const rmt_symbol_word_t *pattern, size_t len, uint32_t pad_ticks) {
  std::vector<rmt_symbol_word_t> result(pattern, pattern + len);
  if (pad_ticks > 0) {
    result.push_back({.duration0 = static_cast<uint16_t>(pad_ticks), .level0 = 0, .duration1 = 0, .level1 = 0});
  }
  return result;
}

// Utility to synchronize both patterns to the longer one
inline std::pair<std::vector<rmt_symbol_word_t>, std::vector<rmt_symbol_word_t>> synchronize_patterns(
    const rmt_symbol_word_t *pattern_a, size_t len_a, const rmt_symbol_word_t *pattern_b, size_t len_b) {
  uint32_t ticks_a = total_ticks(pattern_a, len_a);
  uint32_t ticks_b = total_ticks(pattern_b, len_b);

  if (ticks_a > ticks_b) {
    return {std::vector<rmt_symbol_word_t>(pattern_a, pattern_a + len_a),
            pad_pattern(pattern_b, len_b, ticks_a - ticks_b)};
  } else if (ticks_b > ticks_a) {
    return {pad_pattern(pattern_a, len_a, ticks_b - ticks_a),
            std::vector<rmt_symbol_word_t>(pattern_b, pattern_b + len_b)};
  } else {
    return {std::vector<rmt_symbol_word_t>(pattern_a, pattern_a + len_a),
            std::vector<rmt_symbol_word_t>(pattern_b, pattern_b + len_b)};
  }
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
  RmtPulseGenerator(gpio_num_t gpio_a, gpio_num_t gpio_b, uint32_t resolution_hz = 1000000)
      : resolution_hz_(resolution_hz) {
    tx_gpio_number[0] = gpio_a;
    tx_gpio_number[1] = gpio_b;
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
    for (int i = 0; i < 2; ++i) {
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

    rmt_sync_manager_config_t sync_config = {.tx_channel_array = tx_channels, .array_size = 2};
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
    if (patterns.size() != 2) {
      return false;
    }

    auto [synced_a, synced_b] =
        synchronize_patterns(patterns[0].data(), patterns[0].size(), patterns[1].data(), patterns[1].size());

    // Store synchronized patterns
    current_patterns[0] = synced_a;
    current_patterns[1] = synced_b;

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
  bool running;
  std::vector<rmt_symbol_word_t> current_patterns[2];
};

/**
 * 4-Channel RMT Pulse Generator
 * For ESP32-S3
 */
template<> class RmtPulseGenerator<4> {
 public:
  RmtPulseGenerator(gpio_num_t gpio_a, gpio_num_t gpio_b, gpio_num_t gpio_c, gpio_num_t gpio_d,
                    uint32_t resolution_hz = 1000000)
      : resolution_hz_(resolution_hz) {
    tx_gpio_number[0] = gpio_a;
    tx_gpio_number[1] = gpio_b;
    tx_gpio_number[2] = gpio_c;
    tx_gpio_number[3] = gpio_d;
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

    // Synchronize patterns (hacky way from original library)
    auto [synced_a, synced_b] =
        synchronize_patterns(patterns[0].data(), patterns[0].size(), patterns[1].data(), patterns[1].size());
    auto [synced_c, synced_d] =
        synchronize_patterns(patterns[2].data(), patterns[2].size(), patterns[3].data(), patterns[3].size());
    auto [synced_c2, synced_a2] =
        synchronize_patterns(synced_c.data(), synced_c.size(), synced_a.data(), synced_a.size());
    auto [synced_b2, synced_d2] =
        synchronize_patterns(synced_b.data(), synced_b.size(), synced_d.data(), synced_d.size());

    // Store synchronized patterns
    current_patterns[0] = synced_a2;
    current_patterns[1] = synced_b2;
    current_patterns[2] = synced_c2;
    current_patterns[3] = synced_d2;

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
  bool running;
  std::vector<rmt_symbol_word_t> current_patterns[4];
};

}  // namespace rmt_simple
}  // namespace esphome
