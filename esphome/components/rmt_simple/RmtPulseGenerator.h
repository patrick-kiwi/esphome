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
  RmtPulseGenerator(gpio_num_t gpio_a, gpio_num_t gpio_b, uint32_t resolution_hz = 1000000,
                    bool align_pulse_lengths = true, bool use_sync_manager = true, uint8_t num_active_channels = 2)
      : resolution_hz_(resolution_hz),
        align_pulse_lengths_(align_pulse_lengths),
        use_sync_manager_(use_sync_manager),
        num_active_channels_(num_active_channels) {
    tx_gpio_number_[0] = gpio_a;
    tx_gpio_number_[1] = gpio_b;
    tx_channels_[0] = nullptr;
    tx_channels_[1] = nullptr;
    copy_encoder_ = nullptr;
    sync_mgr_ = nullptr;
    running_ = false;
  }

  ~RmtPulseGenerator() {
    stop();
    cleanup_();
  }

  bool init() {
    // ESP32 (original) needs different settings than C3/C6/P4/S3
    uint16_t mem_block_symbols = use_sync_manager_ ? 48 : 64;
    uint8_t trans_queue_depth = use_sync_manager_ ? 8 : 1;

    // Only create the number of channels that are actually active
    for (int i = 0; i < num_active_channels_; ++i) {
      rmt_tx_channel_config_t config = {.gpio_num = tx_gpio_number_[i],
                                        .clk_src = RMT_CLK_SRC_DEFAULT,
                                        .resolution_hz = resolution_hz_,
                                        .mem_block_symbols = mem_block_symbols,
                                        .trans_queue_depth = trans_queue_depth,
                                        .flags = {}};
      esp_err_t err = rmt_new_tx_channel(&config, &tx_channels_[i]);
      if (err != ESP_OK) {
        return false;
      }
      err = rmt_enable(tx_channels_[i]);
      if (err != ESP_OK) {
        return false;
      }
    }

    // Create sync manager for boards that support it (C3, C6, P4, S3)
    // Only use sync manager if we have 2 active channels
    if (use_sync_manager_ && num_active_channels_ == 2) {
      rmt_sync_manager_config_t sync_config = {.tx_channel_array = tx_channels_, .array_size = 2};
      esp_err_t err = rmt_new_sync_manager(&sync_config, &sync_mgr_);
      if (err != ESP_OK) {
        return false;
      }
    }

    rmt_copy_encoder_config_t enc_config = {};
    return rmt_new_copy_encoder(&enc_config, &copy_encoder_) == ESP_OK;
  }

  bool begin(const std::vector<std::vector<rmt_symbol_word_t>> &patterns) {
    // Validate pattern count matches active channels
    if (patterns.size() != num_active_channels_) {
      return false;
    }

    // Conditionally align pulse lengths based on configuration
    std::vector<std::vector<rmt_symbol_word_t>> aligned_patterns;
    if (align_pulse_lengths_ && num_active_channels_ > 1) {
      aligned_patterns = align_pulse_lengths(patterns);
    } else {
      // Use patterns as-is without length alignment
      aligned_patterns = patterns;
    }

    // Store aligned patterns and start transmission
    rmt_transmit_config_t tx_config = {.loop_count = -1};

    for (int i = 0; i < num_active_channels_; ++i) {
      current_patterns_[i] = aligned_patterns[i];
      esp_err_t err = rmt_transmit(tx_channels_[i], copy_encoder_, current_patterns_[i].data(),
                                   current_patterns_[i].size() * sizeof(rmt_symbol_word_t), &tx_config);
      if (err != ESP_OK) {
        return false;
      }
    }

    running_ = true;
    return true;
  }

  bool update(const std::vector<std::vector<rmt_symbol_word_t>> &patterns) {
    stop();
    return begin(patterns);
  }

  void stop() {
    if (running_) {
      for (auto &tx_channel : tx_channels_) {
        if (tx_channel != nullptr) {
          rmt_disable(tx_channel);
        }
      }
      running_ = false;
    }

    // Re-enable for next use
    for (auto &tx_channel : tx_channels_) {
      if (tx_channel != nullptr) {
        rmt_enable(tx_channel);
      }
    }
  }

  bool is_running() const { return running_; }

 private:
  void cleanup_() {
    if (copy_encoder_ != nullptr) {
      rmt_del_encoder(copy_encoder_);
      copy_encoder_ = nullptr;
    }
    if (sync_mgr_ != nullptr) {
      rmt_del_sync_manager(sync_mgr_);
      sync_mgr_ = nullptr;
    }
    for (auto &tx_channel : tx_channels_) {
      if (tx_channel != nullptr) {
        rmt_disable(tx_channel);
        rmt_del_channel(tx_channel);
        tx_channel = nullptr;
      }
    }
  }

  rmt_channel_handle_t tx_channels_[2];
  gpio_num_t tx_gpio_number_[2];
  rmt_encoder_handle_t copy_encoder_;
  rmt_sync_manager_handle_t sync_mgr_;
  uint32_t resolution_hz_;
  bool align_pulse_lengths_;
  bool use_sync_manager_;
  uint8_t num_active_channels_;
  bool running_;
  std::vector<rmt_symbol_word_t> current_patterns_[2];
};

/**
 * 4-Channel RMT Pulse Generator
 * For ESP32-S3, ESP32-P4
 */
template<> class RmtPulseGenerator<4> {
 public:
  RmtPulseGenerator(gpio_num_t gpio_a, gpio_num_t gpio_b, gpio_num_t gpio_c, gpio_num_t gpio_d,
                    uint32_t resolution_hz = 1000000, bool align_pulse_lengths = true)
      : resolution_hz_(resolution_hz), align_pulse_lengths_(align_pulse_lengths) {
    tx_gpio_number_[0] = gpio_a;
    tx_gpio_number_[1] = gpio_b;
    tx_gpio_number_[2] = gpio_c;
    tx_gpio_number_[3] = gpio_d;
    for (auto &tx_channel : tx_channels_) {
      tx_channel = nullptr;
    }
    copy_encoder_ = nullptr;
    sync_mgr_ = nullptr;
    running_ = false;
  }

  ~RmtPulseGenerator() {
    stop();
    cleanup_();
  }

  bool init() {
    // 4-channel variants always have sync manager support
    for (int i = 0; i < 4; ++i) {
      rmt_tx_channel_config_t config = {.gpio_num = tx_gpio_number_[i],
                                        .clk_src = RMT_CLK_SRC_DEFAULT,
                                        .resolution_hz = resolution_hz_,
                                        .mem_block_symbols = 48,
                                        .trans_queue_depth = 8,
                                        .flags = {}};
      if (rmt_new_tx_channel(&config, &tx_channels_[i]) != ESP_OK) {
        return false;
      }
      if (rmt_enable(tx_channels_[i]) != ESP_OK) {
        return false;
      }
    }

    // Create sync manager for reliable edge alignment
    rmt_sync_manager_config_t sync_config = {.tx_channel_array = tx_channels_, .array_size = 4};
    if (rmt_new_sync_manager(&sync_config, &sync_mgr_) != ESP_OK) {
      return false;
    }

    rmt_copy_encoder_config_t enc_config = {};
    return rmt_new_copy_encoder(&enc_config, &copy_encoder_) == ESP_OK;
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
      current_patterns_[i] = aligned_patterns[i];
    }

    rmt_transmit_config_t tx_config = {.loop_count = -1};

    for (int i = 0; i < 4; ++i) {
      if (rmt_transmit(tx_channels_[i], copy_encoder_, current_patterns_[i].data(),
                       current_patterns_[i].size() * sizeof(rmt_symbol_word_t), &tx_config) != ESP_OK) {
        return false;
      }
    }

    running_ = true;
    return true;
  }

  bool update(const std::vector<std::vector<rmt_symbol_word_t>> &patterns) {
    stop();
    return begin(patterns);
  }

  void stop() {
    if (running_) {
      for (auto &tx_channel : tx_channels_) {
        if (tx_channel != nullptr) {
          rmt_disable(tx_channel);
        }
      }
      running_ = false;
    }

    // Re-enable for next use
    for (auto &tx_channel : tx_channels_) {
      if (tx_channel != nullptr) {
        rmt_enable(tx_channel);
      }
    }
  }

  bool is_running() const { return running_; }

 private:
  void cleanup_() {
    if (copy_encoder_ != nullptr) {
      rmt_del_encoder(copy_encoder_);
      copy_encoder_ = nullptr;
    }
    if (sync_mgr_ != nullptr) {
      rmt_del_sync_manager(sync_mgr_);
      sync_mgr_ = nullptr;
    }
    for (auto &tx_channel : tx_channels_) {
      if (tx_channel != nullptr) {
        rmt_disable(tx_channel);
        rmt_del_channel(tx_channel);
        tx_channel = nullptr;
      }
    }
  }

  rmt_channel_handle_t tx_channels_[4];
  gpio_num_t tx_gpio_number_[4];
  rmt_encoder_handle_t copy_encoder_;
  rmt_sync_manager_handle_t sync_mgr_;
  uint32_t resolution_hz_;
  bool align_pulse_lengths_;
  bool running_;
  std::vector<rmt_symbol_word_t> current_patterns_[4];
};

}  // namespace rmt_simple
}  // namespace esphome
