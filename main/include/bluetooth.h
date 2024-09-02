#ifndef BT_H
#define BT_H

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/semphr.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "adc.h"

#define ESP_SPP_APP_ID 0x79

#define GATTS_TABLE_TAG "GATTS_SPP"

typedef enum {
    HS,
    BP,
    CHAR_NUM,
} CHAR_IDX;

typedef enum {
    SPP_IDX_SVC,
    SPP_IDX_HEARTSOUND_CHAR,
    SPP_IDX_HEARTSOUND_VAL,
    SPP_IDX_HEARTSOUND_CCC,
    SPP_IDX_BLOODPRESSURE_CHAR,
    SPP_IDX_BLOODPRESSURE_VAL,
    SPP_IDX_BLOODPRESSURE_CCC,
    SPP_IDX_NUM,
} SPP_IDX;

typedef struct {
    esp_err_t (*fn)(mcp320x_t *, uint16_t *, mcp320x_ch_t);
    mcp320x_t mcp320x;
    SemaphoreHandle_t sem;
} sample_fn_t;

extern uint16_t spp_handle_tab[SPP_IDX_NUM];
extern sample_fn_t sample_fn;

void gap_event_handler(esp_gap_ble_cb_event_t, esp_ble_gap_cb_param_t *);
void gatts_event_handler(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t *);

#endif