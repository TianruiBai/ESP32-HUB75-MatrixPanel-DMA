/*
 * ESP32-P4 PARLIO Parallel Output for HUB75 LED Matrix
 *
 * Uses the ESP-IDF PARLIO TX driver with loop transmission to continuously
 * output the DMA framebuffer to a HUB75 panel. The PARLIO driver manages all
 * DMA descriptor chains internally, so this implementation is significantly
 * simpler than the ESP32/ESP32-S3 I2S/LCD-peripheral paths.
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
     * - allocate_dma_desc_memory(): no-op (driver allocates internally).
     * - create_dma_desc_link():     records buffer start pointer and
     *                               accumulates total frame size so we can
     *                               call parlio_tx_unit_transmit() with the
     *                               correct bit-length later.
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
     * Shadow buffers.
     *
     * The core library allocates each row independently, so the row data
     * is scattered across the heap.  PARLIO's transmit API needs a single
     * contiguous buffer.  We therefore allocate contiguous "shadow"
     * buffers and memcpy each DMA chunk into them as create_dma_desc_link()
     * is called.
     *
     * _shadow_X        – contiguous DMA-capable allocation
     * _shadow_X_size   – total allocated size in bytes
     * _shadow_X_offset – write cursor (bytes written so far)
     */
    uint8_t *_shadow_a       = nullptr;
    size_t   _shadow_a_size  = 0;
    size_t   _shadow_a_offset = 0;

    uint8_t *_shadow_b       = nullptr;
    size_t   _shadow_b_size  = 0;
    size_t   _shadow_b_offset = 0;

    bool _double_dma_buffer = false;
    bool _started = false;              // loop transmission is active

    /* Internal helpers */
    bool _transmit_loop(void *buffer, size_t size_bytes);
};
