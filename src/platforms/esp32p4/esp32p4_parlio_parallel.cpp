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
 * On ESP32-P4, the PARLIO driver manages descriptors internally.  However,
 * there is a fundamental mismatch: the core library allocates each row
 * independently (separate heap_caps_malloc per rowBitStruct), producing
 * scattered memory.  The PARLIO driver's parlio_tx_unit_transmit() requires
 * a single contiguous buffer.
 *
 * SOLUTION — Contiguous shadow buffers with scatter-gather tracking:
 *
 *   allocate_dma_desc_memory() → allocate contiguous DMA shadow buffer(s)
 *                                plus a chunk-mapping array
 *   create_dma_desc_link()     → record {src, offset, size} mapping and
 *                                do initial memcpy into the shadow buffer
 *   dma_transfer_start()       → re-sync shadow from live row buffers,
 *                                call parlio_tx_unit_transmit() with loop mode
 *   flip_dma_output_buffer()   → re-sync the written-to shadow buffer,
 *                                call parlio_tx_unit_transmit() with new buffer
 *
 * The re-sync step is necessary because the core library writes pixel data
 * directly into the scattered rowBitStruct::data arrays (via drawPixel etc.),
 * not into the shadow buffer.  Before each transmit we memcpy all tracked
 * chunks from their live source pointers into the contiguous shadow.
 *
 * The PARLIO driver's loop-transmission mode with seamless buffer switching
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
        parlio_tx_unit_disable(_tx_unit);
        parlio_del_tx_unit(_tx_unit);
        _tx_unit = nullptr;
    }

    for (int i = 0; i < 2; i++) {
        if (_fb[i].buf) { heap_caps_free(_fb[i].buf); _fb[i].buf = nullptr; }
        if (_fb[i].map) { free(_fb[i].map);           _fb[i].map = nullptr; }
        _fb[i].capacity  = 0;
        _fb[i].used      = 0;
        _fb[i].map_cap   = 0;
        _fb[i].map_count = 0;
    }
    _started = false;
}

/* ──────────────────── DMA descriptor shims ──────────────────── */

void Bus_Parallel16::enable_double_dma_desc() {
    ESP_LOGI(TAG, "Double-buffer (ping-pong) DMA mode enabled");
    _double_dma_buffer = true;
}

bool Bus_Parallel16::allocate_dma_desc_memory(size_t len) {
    /*
     * `len` = total number of DMA descriptors the core library needs.
     * Each descriptor covers up to DMA_MAX (4092) bytes.  We allocate a
     * contiguous shadow buffer sized to hold all the data, plus a mapping
     * array with `len` entries (one per create_dma_desc_link() call).
     *
     * The shadow buffer doubles RAM usage for the framebuffer, but it's the
     * only way to bridge the gap between the core library's per-row heap
     * allocations and PARLIO's contiguous-buffer transmit API.
     */
    static constexpr size_t DMA_MAX = 4096 - 4;  // same as ESP32-S3

    int num_fbs = _double_dma_buffer ? 2 : 1;

    for (int i = 0; i < num_fbs; i++) {
        /* Free any previous allocation */
        if (_fb[i].buf) { heap_caps_free(_fb[i].buf); _fb[i].buf = nullptr; }
        if (_fb[i].map) { free(_fb[i].map);           _fb[i].map = nullptr; }

        size_t total_bytes = len * DMA_MAX;   // upper-bound; actual may be smaller

        _fb[i].buf = (uint8_t *)heap_caps_malloc(total_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!_fb[i].buf) {
            ESP_LOGE(TAG, "Failed to allocate shadow buffer %c (%zu bytes)",
                     'A' + i, total_bytes);
            return false;
        }
        _fb[i].capacity = total_bytes;
        _fb[i].used     = 0;

        _fb[i].map = (DmaChunkMapping *)calloc(len, sizeof(DmaChunkMapping));
        if (!_fb[i].map) {
            ESP_LOGE(TAG, "Failed to allocate chunk map %c (%zu entries)",
                     'A' + i, len);
            return false;
        }
        _fb[i].map_cap   = len;
        _fb[i].map_count = 0;

        ESP_LOGI(TAG, "Shadow buffer %c: %zu bytes (%lu desc slots)",
                 'A' + i, total_bytes, (unsigned long)len);
    }

    return true;
}

void Bus_Parallel16::create_dma_desc_link(void *data, size_t size, bool dmadesc_b) {
    /*
     * Called once per DMA chunk (≤ DMA_MAX bytes) during setupDMA().
     * `data` points into a rowBitStruct::data array — these are scattered
     * across the heap.  We:
     *   1. Record the mapping {src, offset, size} for later re-sync.
     *   2. memcpy the initial data into the contiguous shadow buffer.
     */
    int idx = dmadesc_b ? 1 : 0;
    ShadowBuffer &sb = _fb[idx];

    if (sb.map_count >= sb.map_cap) {
        ESP_LOGE(TAG, "Chunk map %c overflow (%zu >= %zu)",
                 'A' + idx, sb.map_count, sb.map_cap);
        return;
    }
    if (sb.used + size > sb.capacity) {
        ESP_LOGE(TAG, "Shadow buffer %c overflow (%zu + %zu > %zu)",
                 'A' + idx, sb.used, size, sb.capacity);
        return;
    }

    /* Record the mapping */
    DmaChunkMapping &m = sb.map[sb.map_count++];
    m.src    = data;
    m.offset = sb.used;
    m.size   = size;

    /* Initial copy into shadow */
    memcpy(sb.buf + sb.used, data, size);
    sb.used += size;
}

/* ──────────────────── DMA transfer control ──────────────────── */

void Bus_Parallel16::_sync_shadow(ShadowBuffer &sb) {
    /*
     * Re-copy every chunk from the live row buffers into the shadow.
     * This must be called before every transmit because the core library
     * writes pixels directly into the scattered rowBitStruct::data arrays,
     * not into our shadow buffer.
     */
    for (size_t i = 0; i < sb.map_count; i++) {
        const DmaChunkMapping &m = sb.map[i];
        memcpy(sb.buf + m.offset, m.src, m.size);
    }
}

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
    _sync_shadow(_fb[0]);
    ESP_LOGI(TAG, "Starting loop transmission (buffer A, %zu bytes)", _fb[0].used);
    _transmit_loop(_fb[0].buf, _fb[0].used);
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
     * We want to display it, so:
     *   1. Re-sync the shadow from the live scattered row buffers.
     *   2. Submit the shadow for loop transmission.
     *
     * When in loop mode, calling parlio_tx_unit_transmit() with a new buffer
     * makes the driver switch to it seamlessly after the current DMA
     * descriptor chain completes — no tearing.
     */
    ShadowBuffer &sb = _fb[back_buffer_id];
    _sync_shadow(sb);
    _transmit_loop(sb.buf, sb.used);
}

#endif // CONFIG_IDF_TARGET_ESP32P4

