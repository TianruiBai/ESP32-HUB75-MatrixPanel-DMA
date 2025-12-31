#pragma once

#include <sdkconfig.h>

#if defined(CONFIG_IDF_TARGET_ESP32P4)

#include <driver/gpio.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <hal/gpio_hal.h>

// DMA descriptor type expected by the library
#define HUB75_DMA_DESCRIPTOR_T dma_descriptor_t
#define DMA_MAX (4096-4)

class Bus_Parallel16 
{
public:
    struct config_t
    {
        uint32_t bus_freq = 10000000;
        int8_t pin_wr = -1; 
        int8_t pin_rd = -1; 
        int8_t pin_rs = -1; 
        bool   invert_pclk = false;
        union
        {
            int8_t pin_data[16];
            struct
            {
                int8_t pin_d0;
                int8_t pin_d1;
                int8_t pin_d2;
                int8_t pin_d3;
                int8_t pin_d4;
                int8_t pin_d5;
                int8_t pin_d6;
                int8_t pin_d7;
                int8_t pin_d8;
                int8_t pin_d9;
                int8_t pin_d10;
                int8_t pin_d11;
                int8_t pin_d12;
                int8_t pin_d13;
                int8_t pin_d14;
                int8_t pin_d15;
            };
        };
    };

    const config_t& config(void) const;
    void  config(const config_t& config);
   
    bool init(void);
    void release(void);

    void enable_double_dma_desc();
    bool allocate_dma_desc_memory(size_t len);

    void create_dma_desc_link(void *memory, size_t size, bool dmadesc_b = false);

    void dma_transfer_start();
    void dma_transfer_stop();

    void flip_dma_output_buffer(int back_buffer_id);

private:
    config_t _cfg;
    gdma_channel_handle_t dma_chan = nullptr;

    uint32_t _dmadesc_count  = 0;   // number of DMA descriptors
    uint32_t _dmadesc_a_idx  = 0;
    uint32_t _dmadesc_b_idx  = 0;

    HUB75_DMA_DESCRIPTOR_T* _dmadesc_a = nullptr;
    HUB75_DMA_DESCRIPTOR_T* _dmadesc_b = nullptr;    

    bool    _double_dma_buffer = false;
};

#endif
