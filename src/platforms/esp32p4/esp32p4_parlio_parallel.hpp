#pragma once

#include <sdkconfig.h>

#if defined(CONFIG_IDF_TARGET_ESP32P4)

#include <driver/gpio.h>
#include <driver/parlio_tx.h>
#include <vector>
#include <string.h>

// Define descriptor type as void* since we don't use it directly but the library expects a type
#define HUB75_DMA_DESCRIPTOR_T void 
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
    parlio_tx_unit_handle_t _tx_unit = NULL;
    
    struct Chunk {
        void* ptr;
        size_t size;
    };
    
    std::vector<Chunk> _scatter_a;
    std::vector<Chunk> _scatter_b;
    
    uint8_t* _shadow_buffer_a = nullptr;
    uint8_t* _shadow_buffer_b = nullptr;
    size_t _shadow_size = 0;
    
    bool _double_dma_buffer = false;
    
    void consolidate_buffer(const std::vector<Chunk>& scatter, uint8_t*& shadow, size_t& total_size);
};

#endif
