/*
 * ESP32-P4 PARLIO Parallel Output for HUB75 LED Matrix
 *
 * This implementation uses ONLY the official ESP-IDF PARLIO TX driver API.
 * No low-level register hacking or private GDMA calls — the driver manages
 * all DMA descriptors and clock configuration internally.
 *
 * Design rationale
 * ────────────────
 * The core HUB75 library was originally written for the ESP32 I2S peripheral
 * and the ESP32-S3 LCD peripheral, both of which require the user to manually
 * allocate DMA descriptors and build a circular linked list.  The cross-
 * platform Bus_Parallel16 interface therefore exposes:
 *
 *   allocate_dma_desc_memory(count)   – reserve `count` descriptor slots
 *   create_dma_desc_link(ptr, size)   – called once per DMA chunk (≤4092 B)
 *   dma_transfer_start()              – kick off continuous output
 *   flip_dma_output_buffer(id)        – swap to the other ping-pong buffer
 *
 * On ESP32-P4, the PARLIO driver manages descriptors internally.  We therefore
 * treat the descriptor-management calls as lightweight bookkeeping:
 *
 *   allocate_dma_desc_memory() → no-op (driver allocates internally)
 *   create_dma_desc_link()     → record the start pointer of the contiguous
 *                                framebuffer and accumulate the total size
 *   dma_transfer_start()       → call parlio_tx_unit_transmit() with loop mode
 *   flip_dma_output_buffer()   → call parlio_tx_unit_transmit() with new buffer
 *
 * The PARLIO driver's loop-transmission mode with on_buffer_switched callback
 * is the ESP-IDF-sanctioned way to do double-buffered HUB75 output on P4.
 * See the advanced_rgb_led_matrix example in ESP-IDF v5.5.
 */
#include "esp32p4_parlio_parallel.hpp"

#if defined(CONFIG_IDF_TARGET_ESP32P4)

#include <esp_log.h>
#include <esp_err.h>
#include <esp_heap_caps.h>

static const char *TAG = "ESP32P4-PARLIO";

/* ───────────────────────── config accessors ───────────────────────── */

const Bus_Parallel16::config_t& Bus_Parallel16::config(void) const {
    return _cfg;
}

void Bus_Parallel16::config(const config_t& cfg) {
    _cfg = cfg;
}

/* ──────────────────────── init / release ──────────────────────── */

bool Bus_Parallel16::init(void) {
    ESP_LOGI(TAG, "Initialising PARLIO TX unit for HUB75 output");
    ESP_LOGI(TAG, "  Requested pixel clock : %lu Hz", (unsigned long)_cfg.bus_freq);
    ESP_LOGI(TAG, "  Clock GPIO            : %d", _cfg.pin_wr);
    ESP_LOGI(TAG, "  Invert PCLK           : %s", _cfg.invert_pclk ? "yes" : "no");

    /*
     * Build the PARLIO TX unit configuration.
     *
     * data_width = 16 because the HUB75 library packs every clock cycle as
     * a uint16_t containing R1/G1/B1/R2/G2/B2/LAT/OE plus address lines.
     *
     * We set max_transfer_size generously so the driver can handle large
     * frame buffers (panels chained horizontally can be several KB per frame).
     *
     * trans_queue_depth = 4 matches the ESP-IDF advanced_rgb_led_matrix example
     * and is sufficient for double-buffered operation: one slot for the active
     * loop transmission and a couple of spares for the flip.
     */
    parlio_tx_unit_config_t tx_cfg = {};
    tx_cfg.clk_src            = PARLIO_CLK_SRC_DEFAULT;
    tx_cfg.clk_in_gpio_num    = (gpio_num_t)-1;            // use internal clock
    tx_cfg.output_clk_freq_hz = _cfg.bus_freq;
    tx_cfg.data_width         = 16;
    tx_cfg.clk_out_gpio_num   = (gpio_num_t)_cfg.pin_wr;
    tx_cfg.valid_gpio_num     = (gpio_num_t)-1;             // no valid signal
    tx_cfg.trans_queue_depth  = 4;
    tx_cfg.max_transfer_size  = 512 * 1024;                 // 512 KB headroom
    tx_cfg.dma_burst_size     = 64;
    tx_cfg.sample_edge        = _cfg.invert_pclk
                                    ? PARLIO_SAMPLE_EDGE_NEG
                                    : PARLIO_SAMPLE_EDGE_POS;

    /* Map data GPIOs — order matches the bit layout defined in the core lib:
     *   d0..d5  = R1 G1 B1 R2 G2 B2
     *   d6..d7  = LAT OE
     *   d8..d12 = A B C D E
     *   d13..15 = unused (-1)
     */
    for (int i = 0; i < 16; i++) {
        tx_cfg.data_gpio_nums[i] = (gpio_num_t)_cfg.pin_data[i];
    }

    esp_err_t ret = parlio_new_tx_unit(&tx_cfg, &_tx_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "parlio_new_tx_unit failed: %s", esp_err_to_name(ret));
        _tx_unit = nullptr;
        return false;
    }

    ret = parlio_tx_unit_enable(_tx_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "parlio_tx_unit_enable failed: %s", esp_err_to_name(ret));
        parlio_del_tx_unit(_tx_unit);
        _tx_unit = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "PARLIO TX unit ready");
    return true;
}

void Bus_Parallel16::release(void) {
    if (_tx_unit) {
        /*
         * The PARLIO TX unit must be in the *disabled* state before deletion.
         * If loop transmission was active we need to disable it.
         * If it was never started (still in enabled-but-idle state from
         * init()), a single disable is sufficient.
         *
         * parlio_tx_unit_disable() is safe to call even if no transmission
         * is in progress — it simply transitions the state machine from
         * "enabled" to "init".
         */
        parlio_tx_unit_disable(_tx_unit);
        parlio_del_tx_unit(_tx_unit);
        _tx_unit = nullptr;
    }

    if (_shadow_a) { heap_caps_free(_shadow_a); _shadow_a = nullptr; }
    if (_shadow_b) { heap_caps_free(_shadow_b); _shadow_b = nullptr; }
    _shadow_a_size = _shadow_a_offset = 0;
    _shadow_b_size = _shadow_b_offset = 0;
    _started = false;
}

/* ──────────────────── DMA descriptor shims ──────────────────── */

void Bus_Parallel16::enable_double_dma_desc() {
    ESP_LOGI(TAG, "Double-buffer (ping-pong) DMA mode enabled");
    _double_dma_buffer = true;
}

bool Bus_Parallel16::allocate_dma_desc_memory(size_t len) {
    /*
     * The core library passes the total number of DMA descriptors required.
     * Each descriptor covers up to DMA_MAX (4092) bytes.  We use this count
     * to pre-allocate a contiguous shadow buffer large enough to hold the
     * entire frame, since parlio_tx_unit_transmit() requires contiguous
     * memory but the core library allocates each row independently.
     */
    static constexpr size_t DMA_MAX = 4096 - 4;   // same as ESP32-S3
    size_t total_bytes = len * DMA_MAX;            // upper-bound estimate

    /* Free any previous allocation */
    if (_shadow_a) { heap_caps_free(_shadow_a); _shadow_a = nullptr; }
    if (_shadow_b) { heap_caps_free(_shadow_b); _shadow_b = nullptr; }

    _shadow_a = (uint8_t *)heap_caps_malloc(total_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!_shadow_a) {
        ESP_LOGE(TAG, "Failed to allocate shadow buffer A (%zu bytes)", total_bytes);
        return false;
    }
    _shadow_a_size   = total_bytes;
    _shadow_a_offset = 0;

    ESP_LOGI(TAG, "Allocated shadow buffer A: %zu bytes (%lu descriptors)",
             total_bytes, (unsigned long)len);

    if (_double_dma_buffer) {
        _shadow_b = (uint8_t *)heap_caps_malloc(total_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!_shadow_b) {
            ESP_LOGE(TAG, "Failed to allocate shadow buffer B (%zu bytes)", total_bytes);
            return false;
        }
        _shadow_b_size   = total_bytes;
        _shadow_b_offset = 0;

        ESP_LOGI(TAG, "Allocated shadow buffer B: %zu bytes", total_bytes);
    }

    return true;
}

void Bus_Parallel16::create_dma_desc_link(void *data, size_t size, bool dmadesc_b) {
    /*
     * The core library calls this once per DMA chunk (≤ DMA_MAX bytes).
     * Each call may point to a different, non-contiguous heap allocation
     * (rows are malloc'd separately).  We memcpy each chunk into our
     * contiguous shadow buffer so that parlio_tx_unit_transmit() gets a
     * single flat buffer it can pass to GDMA.
     */
    if (dmadesc_b) {
        if (_shadow_b && (_shadow_b_offset + size <= _shadow_b_size)) {
            memcpy(_shadow_b + _shadow_b_offset, data, size);
            _shadow_b_offset += size;
        } else {
            ESP_LOGE(TAG, "Shadow buffer B overflow (%zu + %zu > %zu)",
                     _shadow_b_offset, size, _shadow_b_size);
        }
    } else {
        if (_shadow_a && (_shadow_a_offset + size <= _shadow_a_size)) {
            memcpy(_shadow_a + _shadow_a_offset, data, size);
            _shadow_a_offset += size;
        } else {
            ESP_LOGE(TAG, "Shadow buffer A overflow (%zu + %zu > %zu)",
                     _shadow_a_offset, size, _shadow_a_size);
        }
    }
}

/* ──────────────────── DMA transfer control ──────────────────── */

bool Bus_Parallel16::_transmit_loop(void *buffer, size_t size_bytes) {
    if (!_tx_unit) {
        ESP_LOGE(TAG, "TX unit not initialised");
        return false;
    }
    if (!buffer || size_bytes == 0) {
        ESP_LOGE(TAG, "Invalid buffer (%p) or size (%zu)", buffer, size_bytes);
        return false;
    }

    parlio_transmit_config_t tx_cfg = {};
    tx_cfg.idle_value = 0x0000;             // all outputs low when idle
    tx_cfg.flags.queue_nonblocking = false;  // block if queue is full
    tx_cfg.flags.loop_transmission = true;   // continuous DMA loop

    /*
     * The PARLIO transmit API takes the payload length in *bits*.
     * size_bytes comes from the accumulated create_dma_desc_link() calls
     * which already counted the raw byte size of the framebuffer.
     */
    size_t payload_bits = size_bytes * 8;

    esp_err_t ret = parlio_tx_unit_transmit(_tx_unit, buffer, payload_bits, &tx_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "parlio_tx_unit_transmit failed: %s", esp_err_to_name(ret));
        return false;
    }

    _started = true;
    return true;
}

void Bus_Parallel16::dma_transfer_start() {
    ESP_LOGI(TAG, "Starting loop transmission (buffer A, %zu bytes)",
             _shadow_a_offset);
    _transmit_loop(_shadow_a, _shadow_a_offset);
}

void Bus_Parallel16::dma_transfer_stop() {
    if (!_tx_unit) return;

    if (_started) {
        /*
         * Disable terminates any ongoing (including loop) transmission
         * immediately.  Re-enable so the unit is ready for the next start.
         * These must always be called in matched pairs per the ESP-IDF docs.
         */
        parlio_tx_unit_disable(_tx_unit);
        parlio_tx_unit_enable(_tx_unit);
        _started = false;
    }
}

void Bus_Parallel16::flip_dma_output_buffer(int back_buffer_id) {
    /*
     * back_buffer_id is the buffer the core library *just finished writing*.
     * We want to display it, so we submit it for loop transmission.
     *
     * When in loop mode, calling parlio_tx_unit_transmit() with a new buffer
     * makes the driver switch to it seamlessly after the current DMA
     * descriptor chain completes — no tearing.
     */
    if (back_buffer_id == 1) {
        _transmit_loop(_shadow_b, _shadow_b_offset);
    } else {
        _transmit_loop(_shadow_a, _shadow_a_offset);
    }
}

#endif // CONFIG_IDF_TARGET_ESP32P4

