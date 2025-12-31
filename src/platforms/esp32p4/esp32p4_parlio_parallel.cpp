#include "esp32p4_parlio_parallel.hpp"

#if defined(CONFIG_IDF_TARGET_ESP32P4)

#include <esp_log.h>
#include <esp_err.h>

static const char *TAG_PARLIO = "ESP32P4_PARLIO";

const Bus_Parallel16::config_t& Bus_Parallel16::config(void) const {
    return _cfg;
}

void Bus_Parallel16::config(const config_t& config) {
    _cfg = config;
}

bool Bus_Parallel16::init(void) {
    ESP_LOGI(TAG_PARLIO, "Initializing PARLIO TX unit");
    
    parlio_tx_unit_config_t config = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = (gpio_num_t)-1,
        .output_clk_freq_hz = _cfg.bus_freq,
        .data_width = 16,
        .data_gpio_nums = {
            (gpio_num_t)_cfg.pin_d0, (gpio_num_t)_cfg.pin_d1, (gpio_num_t)_cfg.pin_d2, (gpio_num_t)_cfg.pin_d3,
            (gpio_num_t)_cfg.pin_d4, (gpio_num_t)_cfg.pin_d5, (gpio_num_t)_cfg.pin_d6, (gpio_num_t)_cfg.pin_d7,
            (gpio_num_t)_cfg.pin_d8, (gpio_num_t)_cfg.pin_d9, (gpio_num_t)_cfg.pin_d10, (gpio_num_t)_cfg.pin_d11,
            (gpio_num_t)_cfg.pin_d12, (gpio_num_t)_cfg.pin_d13, (gpio_num_t)_cfg.pin_d14, (gpio_num_t)_cfg.pin_d15
        },
        .clk_out_gpio_num = (gpio_num_t)_cfg.pin_wr,
        .valid_gpio_num = (gpio_num_t)-1,
        .trans_queue_depth = 4, 
        .max_transfer_size = 256 * 1024, // Set a large enough buffer size (256KB)
        .dma_burst_size = DMA_MAX,
        .sample_edge = PARLIO_SAMPLE_EDGE_POS, 
    };
    
    if (_cfg.invert_pclk) {
            config.sample_edge = PARLIO_SAMPLE_EDGE_NEG;
    }

    esp_err_t ret = parlio_new_tx_unit(&config, &_tx_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PARLIO, "Failed to create PARLIO TX unit: %s", esp_err_to_name(ret));
        return false;
    }

    ret = parlio_tx_unit_enable(_tx_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PARLIO, "Failed to enable PARLIO TX unit: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

void Bus_Parallel16::release(void) {
    if (_tx_unit) {
        parlio_tx_unit_disable(_tx_unit);
        parlio_del_tx_unit(_tx_unit);
        _tx_unit = NULL;
    }
}

void Bus_Parallel16::enable_double_dma_desc() {
    _double_dma_buffer = true;
}

bool Bus_Parallel16::allocate_dma_desc_memory(size_t len) {
    // No manual descriptor allocation needed for parlio driver
    return true;
}

void Bus_Parallel16::create_dma_desc_link(void *memory, size_t size, bool dmadesc_b) {
    if (dmadesc_b) {
        _buffer_b = memory;
    } else {
        _buffer_a = memory;
    }
    _buffer_size = size;
}

void Bus_Parallel16::dma_transfer_start() {
    if (!_tx_unit || !_buffer_a) return;

    parlio_transmit_config_t transmit_config = {
        .idle_value = 0x00,
        .flags = {
            .queue_nonblocking = false,
            .loop_transmission = true 
        }
    };

    // Start with buffer A
    esp_err_t ret = parlio_tx_unit_transmit(_tx_unit, _buffer_a, _buffer_size * 8, &transmit_config);
    if (ret != ESP_OK) {
            ESP_LOGE(TAG_PARLIO, "Failed to start transmission: %s", esp_err_to_name(ret));
    }
}

void Bus_Parallel16::dma_transfer_stop() {
    if (_tx_unit) {
        parlio_tx_unit_disable(_tx_unit);
        // Re-enable to reset state?
        parlio_tx_unit_enable(_tx_unit);
    }
}

void Bus_Parallel16::flip_dma_output_buffer(int back_buffer_id) {
    if (!_tx_unit) return;
    
    // back_buffer_id is the one we just wrote to. So we want to display it.
    void* buffer = (back_buffer_id == 1) ? _buffer_b : _buffer_a;
    if (!buffer) return;

    parlio_transmit_config_t transmit_config = {
        .idle_value = 0x00,
        .flags = {
            .queue_nonblocking = false,
            .loop_transmission = true 
        }
    };

    esp_err_t ret = parlio_tx_unit_transmit(_tx_unit, buffer, _buffer_size * 8, &transmit_config);
    if (ret != ESP_OK) {
            ESP_LOGE(TAG_PARLIO, "Failed to flip buffer: %s", esp_err_to_name(ret));
    }
}

#endif
