#include "esp32p4_parlio_parallel.hpp"

#if defined(CONFIG_IDF_TARGET_ESP32P4)

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_clk_tree.h>
#include <esp_rom_gpio.h>
#include <hal/hal_utils.h>
#include <hal/parlio_ll.h>
#include <hal/parlio_types.h>
#include <soc/gdma_periph.h>
#include <soc/parl_io_struct.h>
#include <soc/parlio_periph.h>

static const char *TAG_PARLIO = "ESP32P4_PARLIO";

const Bus_Parallel16::config_t& Bus_Parallel16::config(void) const {
    return _cfg;
}

void Bus_Parallel16::config(const config_t& config) {
    _cfg = config;
}

bool Bus_Parallel16::init(void) {
    ESP_LOGI(TAG_PARLIO, "Initializing PARLIO TX (LL)");

    // Enable and reset PARLIO clocks/registers
    _parlio_ll_enable_bus_clock(0, true);
    _parlio_ll_reset_register(0);

    parlio_ll_tx_reset_fifo(&PARL_IO);
    _parlio_ll_tx_reset_clock(&PARL_IO);
    _parlio_ll_tx_enable_clock(&PARL_IO, false);

    // Clock source and divider
    const parlio_clock_source_t clk_src = PARLIO_CLK_SRC_DEFAULT;
    _parlio_ll_tx_set_clock_source(&PARL_IO, clk_src);

    uint32_t src_hz = 0;
    esp_clk_tree_src_get_freq_hz((soc_module_clk_t)clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src_hz);
    uint32_t div = (_cfg.bus_freq == 0) ? 1 : (src_hz + _cfg.bus_freq - 1) / _cfg.bus_freq;
    if (div < 1) div = 1;
    if (div > PARLIO_LL_TX_MAX_CLK_INT_DIV) div = PARLIO_LL_TX_MAX_CLK_INT_DIV;

    hal_utils_clk_div_t clk_div = {
        .integer = div,
        .denominator = 1,
        .numerator = 0,
    };
    _parlio_ll_tx_set_clock_div(&PARL_IO, &clk_div);
    _cfg.bus_freq = src_hz / clk_div.integer;

    // Bus configuration
    parlio_ll_tx_set_bus_width(&PARL_IO, 16);
    auto sample_edge = _cfg.invert_pclk ? PARLIO_SAMPLE_EDGE_NEG : PARLIO_SAMPLE_EDGE_POS;
    parlio_ll_tx_set_sample_clock_edge(&PARL_IO, sample_edge);
    parlio_ll_tx_set_idle_data_value(&PARL_IO, 0);
    parlio_ll_tx_set_trans_bit_len(&PARL_IO, 0); // use DMA EOF
    parlio_ll_tx_set_eof_condition(&PARL_IO, PARLIO_LL_TX_EOF_COND_DMA_EOF);
    parlio_ll_tx_enable_clock_gating(&PARL_IO, false);

#if HAL_CONFIG(CHIP_SUPPORT_MIN_REV) >= 300
    parlio_ll_tx_clock_gating_from_valid(&PARL_IO, false);
#else
    parlio_ll_tx_treat_msb_as_valid(&PARL_IO, false);
#endif

    parlio_ll_clear_interrupt_status(&PARL_IO, PARLIO_LL_EVENT_TX_MASK);
    parlio_ll_enable_interrupt(&PARL_IO, PARLIO_LL_EVENT_TX_MASK, false);

    // GPIO matrix routing for data lines
    int8_t *pins = _cfg.pin_data;
    gpio_config_t gpio_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    for (int i = 0; i < 16; i++) {
        if (pins[i] < 0) continue;
        gpio_conf.pin_bit_mask = BIT64(pins[i]);
        gpio_config(&gpio_conf);
        esp_rom_gpio_connect_out_signal(pins[i], parlio_periph_signals.groups[0].tx_units[0].data_sigs[i], false, false);
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pins[i]], PIN_FUNC_GPIO);
        gpio_set_drive_capability((gpio_num_t)pins[i], (gpio_drive_cap_t)3);
    }

    // Clock pin
    if (_cfg.pin_wr >= 0) {
        gpio_conf.pin_bit_mask = BIT64(_cfg.pin_wr);
        gpio_config(&gpio_conf);
        esp_rom_gpio_connect_out_signal(_cfg.pin_wr, parlio_periph_signals.groups[0].tx_units[0].clk_out_sig, _cfg.invert_pclk, false);
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[_cfg.pin_wr], PIN_FUNC_GPIO);
        gpio_set_drive_capability((gpio_num_t)_cfg.pin_wr, (gpio_drive_cap_t)3);
    }

    // GDMA setup
    static gdma_channel_alloc_config_t dma_chan_config = {
        .sibling_chan = NULL,
        .direction = GDMA_CHANNEL_DIRECTION_TX,
        .flags = { .reserve_sibling = 0 }
    };
    if (gdma_new_channel(&dma_chan_config, &dma_chan) != ESP_OK) {
        ESP_LOGE(TAG_PARLIO, "Failed to allocate GDMA channel");
        return false;
    }

    gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_PARLIO, 0));

    static gdma_strategy_config_t strategy_config = {
        .owner_check = false,
        .auto_update_desc = false
    };
    gdma_apply_strategy(dma_chan, &strategy_config);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    gdma_transfer_config_t transfer_config = {
#ifdef SPIRAM_DMA_BUFFER
        .max_data_burst_size = 64,
        .access_ext_mem = true
#else
        .max_data_burst_size = 32,
        .access_ext_mem = false
#endif
    };
    gdma_config_transfer(dma_chan, &transfer_config);
#else
    gdma_transfer_ability_t ability = {
        .sram_trans_align = 32,
        .psram_trans_align = 64,
    };
    gdma_set_transfer_ability(dma_chan, &ability);
#endif

    return true;
}

void Bus_Parallel16::release(void) {
    if (dma_chan) {
        gdma_stop(dma_chan);
        gdma_del_channel(dma_chan);
        dma_chan = nullptr;
    }

    if (_dmadesc_a) {
        heap_caps_free(_dmadesc_a);
        _dmadesc_a = nullptr;
    }
    if (_dmadesc_b) {
        heap_caps_free(_dmadesc_b);
        _dmadesc_b = nullptr;
    }
    _dmadesc_count = 0;
    _dmadesc_a_idx = 0;
    _dmadesc_b_idx = 0;
}

void Bus_Parallel16::enable_double_dma_desc() {
    _double_dma_buffer = true;
}

bool Bus_Parallel16::allocate_dma_desc_memory(size_t len) {
    if (_dmadesc_a) {
        heap_caps_free(_dmadesc_a);
        _dmadesc_a = nullptr;
    }
    if (_dmadesc_b) {
        heap_caps_free(_dmadesc_b);
        _dmadesc_b = nullptr;
    }

    _dmadesc_count = len;
    _dmadesc_a_idx = 0;
    _dmadesc_b_idx = 0;

    _dmadesc_a = (HUB75_DMA_DESCRIPTOR_T *)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);
    if (!_dmadesc_a) {
        ESP_LOGE(TAG_PARLIO, "Failed to malloc _dmadesc_a (%u)", (unsigned)len);
        return false;
    }

    if (_double_dma_buffer) {
        _dmadesc_b = (HUB75_DMA_DESCRIPTOR_T *)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);
        if (!_dmadesc_b) {
            ESP_LOGE(TAG_PARLIO, "Failed to malloc _dmadesc_b (%u)", (unsigned)len);
            _double_dma_buffer = false;
        }
    }

    return true;
}

void Bus_Parallel16::create_dma_desc_link(void *data, size_t size, bool dmadesc_b) {
    static constexpr size_t MAX_DMA_LEN = (4096 - 4);

    if (size > MAX_DMA_LEN) {
        ESP_LOGW(TAG_PARLIO, "Payload size %u exceeds MAX_DMA_LEN, truncating", (unsigned)size);
        size = MAX_DMA_LEN;
    }

    if (dmadesc_b && !_double_dma_buffer) {
        ESP_LOGW(TAG_PARLIO, "Double buffer not enabled, ignoring B descriptor");
        dmadesc_b = false;
    }

    HUB75_DMA_DESCRIPTOR_T *desc_array = dmadesc_b ? _dmadesc_b : _dmadesc_a;
    uint32_t *idx = dmadesc_b ? &_dmadesc_b_idx : &_dmadesc_a_idx;

    if (!desc_array || *idx >= _dmadesc_count) {
        ESP_LOGE(TAG_PARLIO, "Descriptor array missing or overflow");
        return;
    }

    auto &desc = desc_array[*idx];
    desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    desc.dw0.suc_eof = (*idx == (_dmadesc_count - 1));
    desc.dw0.size = desc.dw0.length = size;
    desc.buffer = data;

    if (*idx == _dmadesc_count - 1) {
        desc.next = (dma_descriptor_t *)&desc_array[0];
    } else {
        desc.next = (dma_descriptor_t *)&desc_array[*idx + 1];
    }

    (*idx)++;
}

void Bus_Parallel16::dma_transfer_start() {
    if (!dma_chan || !_dmadesc_a) return;

    parlio_ll_tx_reset_fifo(&PARL_IO);
    parlio_ll_tx_reset_clock(&PARL_IO);
    _parlio_ll_tx_enable_clock(&PARL_IO, true);

    gdma_start(dma_chan, (intptr_t)&_dmadesc_a[0]);

    while (!parlio_ll_tx_is_ready(&PARL_IO)) {
        ;
    }

    parlio_ll_tx_start(&PARL_IO, true);
}

void Bus_Parallel16::dma_transfer_stop() {
    if (dma_chan) {
        gdma_stop(dma_chan);
    }
    parlio_ll_tx_start(&PARL_IO, false);
    _parlio_ll_tx_enable_clock(&PARL_IO, false);
}

void Bus_Parallel16::flip_dma_output_buffer(int back_buffer_id) {
    if (!_double_dma_buffer || !_dmadesc_a || !_dmadesc_b || _dmadesc_count == 0) return;

    if (back_buffer_id == 1) {
        _dmadesc_a[_dmadesc_count - 1].next = (dma_descriptor_t *)&_dmadesc_b[0];
        _dmadesc_b[_dmadesc_count - 1].next = (dma_descriptor_t *)&_dmadesc_b[0];
    } else {
        _dmadesc_b[_dmadesc_count - 1].next = (dma_descriptor_t *)&_dmadesc_a[0];
        _dmadesc_a[_dmadesc_count - 1].next = (dma_descriptor_t *)&_dmadesc_a[0];
    }
}

#endif
