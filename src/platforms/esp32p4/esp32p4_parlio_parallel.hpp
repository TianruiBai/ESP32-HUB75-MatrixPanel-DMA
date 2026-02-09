/*
 * ESP32-P4 PARLIO Parallel Output for HUB75 LED Matrix
 *
 * Uses the ESP-IDF PARLIO TX driver with loop transmission to continuously
 * output the DMA framebuffer to a HUB75 panel. The PARLIO driver manages all
 * DMA descriptor chains internally, so this implementation is significantly
 * simpler than the ESP32/ESP32-S3 I2S/LCD-peripheral paths.
 *
 * IMPORTANT: Double-buffering (HUB75_I2S_CFG::double_buff = true) is strongly
 * recommended on ESP32-P4.  In single-buffer mode, pixel updates will not be
 * visible until the next dma_transfer_start() because the DMA reads from a
 * contiguous shadow buffer, not the scattered per-row allocations that drawPixel
 * writes to.  Double-buffered mode triggers a shadow re-sync on every flip.
 *
 * Reference:
 *   - ESP-IDF PARLIO TX driver docs:
 *     https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/parlio/parlio_tx.html
 *   - ESP-IDF advanced_rgb_led_matrix example (loop transmission + HUB75):
 *     https://github.com/espressif/esp-idf/tree/v5.5.2/examples/peripherals/parlio/parlio_tx/advanced_rgb_led_matrix
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <driver/gpio.h>
#include <driver/parlio_tx.h>
#include <esp_err.h>
#include <esp_log.h>
#include <hal/dma_types.h>

/*
 * The PARLIO driver manages DMA descriptors internally.
 * We still define the type alias so the core library's frame-buffer
 * allocation code compiles, but we never manually chain descriptors.
 */
#define HUB75_DMA_DESCRIPTOR_T dma_descriptor_t

class Bus_Parallel16 {
public:
    struct config_t {
        uint32_t bus_freq = 10000000;   // PARLIO output clock frequency (Hz)
        int8_t pin_wr  = -1;            // Pixel-clock output GPIO
        int8_t pin_rd  = -1;            // Unused – kept for cross-platform API compat
        int8_t pin_rs  = -1;            // Unused – kept for cross-platform API compat
        bool   invert_pclk = false;     // Invert pixel-clock polarity

        union {
            int8_t pin_data[16];
            struct {
                int8_t pin_d0;   // R1
                int8_t pin_d1;   // G1
                int8_t pin_d2;   // B1
                int8_t pin_d3;   // R2
                int8_t pin_d4;   // G2
                int8_t pin_d5;   // B2
                int8_t pin_d6;   // LAT
                int8_t pin_d7;   // OE
                int8_t pin_d8;   // A
                int8_t pin_d9;   // B
                int8_t pin_d10;  // C
                int8_t pin_d11;  // D
                int8_t pin_d12;  // E
                int8_t pin_d13;  // (unused, set -1)
                int8_t pin_d14;  // (unused, set -1)
                int8_t pin_d15;  // (unused, set -1)
            };
        };
    };

    const config_t& config(void) const;
    void  config(const config_t& cfg);

    bool init(void);
    void release(void);

    /*
     * DMA descriptor management shims.
     *
     * The PARLIO driver manages DMA descriptors internally.  These methods
     * exist to satisfy the cross-platform interface used by the core library.
     *
     * - allocate_dma_desc_memory(): allocates contiguous shadow buffer(s) and
     *                               chunk-mapping arrays.
     * - create_dma_desc_link():     records the source→shadow mapping for one
     *                               DMA chunk and does the initial memcpy.
     */
    void enable_double_dma_desc();
    bool allocate_dma_desc_memory(size_t len);
    void create_dma_desc_link(void *data, size_t size, bool dmadesc_b = false);

    void dma_transfer_start();
    void dma_transfer_stop();
    void flip_dma_output_buffer(int back_buffer_id);

private:
    config_t _cfg{};

    parlio_tx_unit_handle_t _tx_unit = nullptr;

    /*
     * Scatter-gather → contiguous shadow buffer.
     *
     * PROBLEM: The core library allocates each row independently via
     * heap_caps_malloc (inside rowBitStruct). The pixel-drawing code
     * (drawPixel, etc.) writes directly into these scattered row buffers.
     * But parlio_tx_unit_transmit() requires a single contiguous buffer.
     *
     * SOLUTION: We maintain a contiguous "shadow" buffer for each
     * framebuffer (A and optionally B). During create_dma_desc_link()
     * we record the mapping from each scattered source chunk to its
     * position within the shadow, and do the initial memcpy. Before
     * each transmit (start or flip), we re-sync the shadow from the
     * live row buffers via _sync_shadow().
     */

    /* One entry per create_dma_desc_link() call */
    struct DmaChunkMapping {
        void  *src;       // pointer into the live rowBitStruct::data
        size_t offset;    // byte offset within the shadow buffer
        size_t size;      // chunk size in bytes
    };

    /* Per-framebuffer state */
    struct ShadowBuffer {
        uint8_t               *buf       = nullptr;   // contiguous DMA-capable memory
        size_t                 capacity  = 0;         // allocated size (bytes)
        size_t                 used      = 0;         // bytes written so far
        DmaChunkMapping       *map       = nullptr;   // array of chunk mappings
        size_t                 map_cap   = 0;         // allocated entries in map[]
        size_t                 map_count = 0;         // entries used in map[]
    };

    ShadowBuffer _fb[2];                // index 0 = A, index 1 = B

    bool _double_dma_buffer = false;
    bool _started = false;              // loop transmission is active

    /* Internal helpers */
    void _sync_shadow(ShadowBuffer &sb);
    bool _transmit_loop(void *buffer, size_t size_bytes);
};
