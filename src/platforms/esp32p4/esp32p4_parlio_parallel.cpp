#include "esp32p4_parlio_parallel.hpp"

#if defined(CONFIG_IDF_TARGET_ESP32P4)

#include <esp_log.h>
#include <esp_err.h>
#include <esp_rom_gpio.h>
#include <esp_heap_caps.h>
#include <soc/parlio_periph.h>

static const char *TAG_PARLIO = "ESP32P4_PARLIO";

const Bus_Parallel16::config_t& Bus_Parallel16::config(void) const {
    return _cfg;
}

void Bus_Parallel16::config(const config_t& config) {
    _cfg = config;
}

bool Bus_Parallel16::init(void) {
    ESP_LOGI(TAG_PARLIO, "Initializing PARLIO TX unit (prefer driver, fallback to LL)");

    if (init_with_driver()) {
        _using_driver = true;
        return true;
    }

    ESP_LOGW(TAG_PARLIO, "Driver init failed, falling back to low-level PARLIO setup");
    _using_driver = false;
    return init_with_ll();
}

void Bus_Parallel16::release(void) {
    if (_using_driver && _tx_unit) {
        parlio_tx_unit_disable(_tx_unit);
        parlio_del_tx_unit(_tx_unit);
        _tx_unit = NULL;
    }

    // Clean up low level resources if they were used
    stop_ll();
}

void Bus_Parallel16::enable_double_dma_desc() {
    _double_dma_buffer = true;
}

bool Bus_Parallel16::allocate_dma_desc_memory(size_t len) {
    if (_using_driver) {
        // Driver path manages descriptors internally
        return true;
    }

    _dmadesc_count = dma_desc_get_required_num(len, DMA_DESCRIPTOR_BUFFER_MAX_SIZE);
    _dmadesc_a = static_cast<HUB75_DMA_DESCRIPTOR_T *>(heap_caps_calloc(_dmadesc_count, sizeof(HUB75_DMA_DESCRIPTOR_T), MALLOC_CAP_DMA));
    if (!_dmadesc_a) {
        ESP_LOGE(TAG_PARLIO, "Failed to allocate primary DMA descriptors");
        return false;
    }

    if (_double_dma_buffer) {
        _dmadesc_b = static_cast<HUB75_DMA_DESCRIPTOR_T *>(heap_caps_calloc(_dmadesc_count, sizeof(HUB75_DMA_DESCRIPTOR_T), MALLOC_CAP_DMA));
        if (!_dmadesc_b) {
            ESP_LOGE(TAG_PARLIO, "Failed to allocate secondary DMA descriptors");
            return false;
        }
    }

    return true;
}

void Bus_Parallel16::create_dma_desc_link(void *memory, size_t size, bool dmadesc_b) {
    if (dmadesc_b) {
        _buffer_b = memory;
    } else {
        _buffer_a = memory;
    }

    _buffer_bits = size * 8; // record bit length for the active buffer

    if (_using_driver) {
        return; // driver will manage descriptors internally
    }

    HUB75_DMA_DESCRIPTOR_T *desc = dmadesc_b ? _dmadesc_b : _dmadesc_a;
    if (!desc) {
        ESP_LOGE(TAG_PARLIO, "DMA descriptor storage is not allocated");
        return;
    }

    size_t remaining = size;
    uint8_t *buf = static_cast<uint8_t *>(memory);
    for (size_t i = 0; i < _dmadesc_count; ++i) {
        size_t chunk = remaining > DMA_DESCRIPTOR_BUFFER_MAX_SIZE ? DMA_DESCRIPTOR_BUFFER_MAX_SIZE : remaining;
        desc[i].dw0.size = chunk;
        desc[i].dw0.length = chunk;
        desc[i].dw0.suc_eof = (i == (_dmadesc_count - 1));
        desc[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        desc[i].buffer = buf;
        desc[i].next = (i == (_dmadesc_count - 1)) ? nullptr : &desc[i + 1];
        buf += chunk;
        remaining -= chunk;
    }
}

void Bus_Parallel16::dma_transfer_start() {
    void *buffer = _buffer_a;
    if (!buffer || _buffer_bits == 0) {
        ESP_LOGE(TAG_PARLIO, "DMA start requested without a valid buffer");
        return;
    }

    if (_using_driver && _tx_unit) {
        if (start_with_driver(buffer)) {
            return;
        }
        ESP_LOGW(TAG_PARLIO, "Driver start failed, retrying with low-level path");
    }

    start_with_ll(buffer);
}

void Bus_Parallel16::dma_transfer_stop() {
    if (_using_driver && _tx_unit) {
        parlio_tx_unit_disable(_tx_unit);
        // Re-enable to reset state?
        parlio_tx_unit_enable(_tx_unit);
        return;
    }

    stop_ll();
}

void Bus_Parallel16::flip_dma_output_buffer(int back_buffer_id) {
    // back_buffer_id is the one we just wrote to. So we want to display it.
    void *buffer = (back_buffer_id == 1) ? _buffer_b : _buffer_a;
    if (!buffer || _buffer_bits == 0) {
        return;
    }

    if (_using_driver && _tx_unit) {
        if (start_with_driver(buffer)) {
            return;
        }
        ESP_LOGW(TAG_PARLIO, "Driver flip failed, retrying with low-level path");
    }

    start_with_ll(buffer);
}

bool Bus_Parallel16::init_with_driver() {
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
        .trans_queue_depth = 8,
        .max_transfer_size = 256 * 1024,
        .dma_burst_size = 32,
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
    };

    if (_cfg.invert_pclk) {
        config.sample_edge = PARLIO_SAMPLE_EDGE_NEG;
    }

    esp_err_t ret = parlio_new_tx_unit(&config, &_tx_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PARLIO, "Failed to create PARLIO TX unit: %s", esp_err_to_name(ret));
        _tx_unit = nullptr;
        return false;
    }

    ret = parlio_tx_unit_enable(_tx_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PARLIO, "Failed to enable PARLIO TX unit: %s", esp_err_to_name(ret));
        parlio_del_tx_unit(_tx_unit);
        _tx_unit = nullptr;
        return false;
    }

    return true;
}

bool Bus_Parallel16::init_with_ll() {
    parl_io_dev_t *dev = &PARL_IO;

    // Enable and reset the peripheral clocks
    _parlio_ll_enable_bus_clock(0, true);
    _parlio_ll_reset_register(0);
    _parlio_ll_tx_enable_clock(dev, true);
    _parlio_ll_tx_reset_clock(dev);

    // Configure clock source and divider (best effort approximation to requested bus frequency)
    uint32_t src_clk_hz = 160000000; // PLL_160M is available on ESP32-P4
    uint32_t div = src_clk_hz / (_cfg.bus_freq ? _cfg.bus_freq : 1);
    if (div == 0) {
        div = 1;
    }
    if (div > PARLIO_LL_TX_MAX_CLK_INT_DIV) {
        div = PARLIO_LL_TX_MAX_CLK_INT_DIV;
    }

    hal_utils_clk_div_t clk_div = {
        .integer = static_cast<uint32_t>(div),
        .denominator = 0,
        .numerator = 0,
    };

    _parlio_ll_tx_set_clock_source(dev, PARLIO_CLK_SRC_PLL_F160M);
    _parlio_ll_tx_set_clock_div(dev, &clk_div);

    // Configure data path
    _parlio_ll_tx_set_bus_width(dev, 16);
    _parlio_ll_tx_set_sample_clock_edge(dev, _cfg.invert_pclk ? PARLIO_SAMPLE_EDGE_NEG : PARLIO_SAMPLE_EDGE_POS);
    _parlio_ll_tx_enable_clock_gating(dev, false);
    parlio_ll_tx_set_eof_condition(dev, PARLIO_LL_TX_EOF_COND_DMA_EOF);
    parlio_ll_tx_set_idle_data_value(dev, 0);

    // Route GPIO signals manually via matrix
#if defined(SOC_PARLIO_SUPPORT_DMA) && defined(SOC_PARLIO_TX_UNITS_PER_GROUP)
    for (size_t i = 0; i < 16; ++i) {
        int8_t pin = _cfg.pin_data[i];
        if (pin >= 0) {
            gpio_reset_pin((gpio_num_t)pin);
            gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
            esp_rom_gpio_connect_out_signal(pin, soc_parlio_signals[0].tx_units[0].data_sigs[i], false, false);
        }
    }
    if (_cfg.pin_wr >= 0) {
        gpio_reset_pin((gpio_num_t)_cfg.pin_wr);
        gpio_set_direction((gpio_num_t)_cfg.pin_wr, GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(_cfg.pin_wr, soc_parlio_signals[0].tx_units[0].clk_out_sig, false, false);
    }
#else
    ESP_LOGW(TAG_PARLIO, "Low-level GPIO matrix setup unavailable; low-level init will fail");
    return false;
#endif

    // Basic GDMA setup
    gdma_channel_alloc_config_t dma_chan_config = {
        .direction = GDMA_CHANNEL_DIRECTION_TX,
    };

    esp_err_t ret = gdma_new_channel(&dma_chan_config, &dma_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PARLIO, "Failed to allocate GDMA channel: %s", esp_err_to_name(ret));
        dma_chan = nullptr;
        return false;
    }

    ret = gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_PARLIO, 0));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PARLIO, "Failed to connect GDMA to PARLIO: %s", esp_err_to_name(ret));
        return false;
    }

    gdma_strategy_config_t strategy = {
        .owner_check = true,
        .auto_update_desc = false,
        .eof_till_data_popped = true,
    };
    gdma_apply_strategy(dma_chan, &strategy);

    gdma_transfer_config_t trans_cfg = {
        .max_data_burst_size = 32,
        .access_ext_mem = true,
    };
    gdma_config_transfer(dma_chan, &trans_cfg);

    return true;
}

bool Bus_Parallel16::start_with_driver(void *buffer) {
    if (!_tx_unit) {
        return false;
    }

    parlio_transmit_config_t transmit_config = {
        .idle_value = 0x00,
        .flags = {
            .queue_nonblocking = false,
            .loop_transmission = true,
        }
    };

    esp_err_t ret = parlio_tx_unit_transmit(_tx_unit, buffer, _buffer_bits, &transmit_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PARLIO, "PARLIO driver transmit failed: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool Bus_Parallel16::start_with_ll(void *buffer) {
    if (!dma_chan) {
        ESP_LOGE(TAG_PARLIO, "GDMA channel is not ready for low-level start");
        return false;
    }

    HUB75_DMA_DESCRIPTOR_T *desc = (buffer == _buffer_b && _dmadesc_b) ? _dmadesc_b : _dmadesc_a;
    if (!desc) {
        ESP_LOGE(TAG_PARLIO, "No DMA descriptor available for low-level start");
        return false;
    }

    // Reset DMA and peripheral state before restart
    gdma_stop(dma_chan);
    gdma_reset(dma_chan);

    esp_err_t ret = gdma_start(dma_chan, (intptr_t)desc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PARLIO, "GDMA start failed: %s", esp_err_to_name(ret));
        return false;
    }

    parl_io_dev_t *dev = &PARL_IO;
    parlio_ll_tx_reset_fifo(dev);
    parlio_ll_tx_set_trans_bit_len(dev, (uint32_t)_buffer_bits);
    parlio_ll_tx_start(dev, true);

    return true;
}

void Bus_Parallel16::stop_ll() {
    if (dma_chan) {
        gdma_stop(dma_chan);
        gdma_reset(dma_chan);
        gdma_disconnect(dma_chan);
        gdma_del_channel(dma_chan);
        dma_chan = nullptr;
    }

    parl_io_dev_t *dev = &PARL_IO;
    parlio_ll_tx_start(dev, false);
    _parlio_ll_tx_enable_clock(dev, false);

    if (_dmadesc_a) {
        heap_caps_free(_dmadesc_a);
        _dmadesc_a = nullptr;
    }
    if (_dmadesc_b) {
        heap_caps_free(_dmadesc_b);
        _dmadesc_b = nullptr;
    }

    _dmadesc_count = 0;
    _buffer_bits = 0;
}

#endif
