#ifndef ADC_H
#define ADC_H

#include "hal/spi_types.h"
#include "driver/spi_master.h"

#define ADC_TAG "ADC"

// define command for MCP3204
#define START_SINGLE 0x06
#define SEL_CH0 0x00

#define START (1 << 2)
#define SINGLE (1 << 1)
#define SEL_CH(x) (x << 6)

typedef enum {
    CH0,
    CH1,
    CH2,
    CH3,
    CH_NUM,
} mcp320x_ch_t;

typedef struct {
    int cs_pin;
    int expected_sampl_freq;
    int sampl_count;
    float vref;
    bool single;
    spi_device_handle_t handle;
} mcp320x_t;

esp_err_t mcp320x_add(spi_host_device_t, mcp320x_t *);
esp_err_t mcp320x_sample(mcp320x_t *, uint16_t *, mcp320x_ch_t);
esp_err_t mcp320x_remove(mcp320x_t *);

#endif