#include <string.h>

#include "FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_intr_types.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"

#include "driver/spi_common.h"

#include "bluetooth.h"
#include "adc.h"

// define pins for VSPI
#define SCLK 6
#define MISO 2
#define MOSI 7

// define CS pin for MCP3204
#define CS0 10

static spi_host_device_t host = SPI2_HOST;

static spi_bus_config_t bus_config = {
    .mosi_io_num = MOSI,
    .miso_io_num = MISO,
    .sclk_io_num = SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 3,
    .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK |
    SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MISO |
    SPICOMMON_BUSFLAG_IOMUX_PINS,
    .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
    .intr_flags = ESP_INTR_FLAG_LEVEL3,
};

static mcp320x_t mcp3204 = {
    .cs_pin = CS0,
    .expected_sampl_freq = 30000,
    .sampl_count = 1,
    .vref = 3.3,
};

static esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

sample_fn_t sample_fn;

// static spi_transaction_t test_trans_desc = {
//     .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
//     .length = 3 * 8,
//     .rxlength = 3 * 8,  // defaults the value to length
//     .tx_data = {START_SIGNLE, SEL_CH0},
// };

void app_main(void) {
    esp_err_t res;
    // setup ADC
    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(mcp320x_add(host, &mcp3204));

    // setup sampling function
    sample_fn.fn = mcp320x_sample;
    sample_fn.mcp320x = mcp3204;
    (void)sample_fn;

    // ESP_LOGI(ADC_TAG, "tx_data len: %zu", test_trans_desc.length);
    // esp_log_buffer_hex(ADC_TAG, test_trans_desc.tx_data, 3);
    // for (int i = 0; i < 100; ++i) {
    //     ESP_ERROR_CHECK(spi_device_transmit(mcp3204.handle, &test_trans_desc));
    //     esp_log_buffer_hex(ADC_TAG, test_trans_desc.rx_data, 3);
    //     uint16_t value = ((test_trans_desc.rx_data[1] & 0x0F) << 8) | test_trans_desc.rx_data[2];
    //     ESP_LOGI(ADC_TAG, "%u %.02f", value, (value * mcp3204.vref / 4096.0));
    // }

    // initialize NVS
    res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES ||
        res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        res = nvs_flash_init();
    }
    ESP_ERROR_CHECK(res);

    // setup bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    res = esp_bt_controller_init(&bt_cfg);
    if (res) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__,
                 esp_err_to_name(res));
        return;
    }

    res = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (res) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__,
                 esp_err_to_name(res));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);

    res = esp_bluedroid_init();
    if (res) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__,
                 esp_err_to_name(res));
        return;
    }
    res = esp_bluedroid_enable();
    if (res) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__,
                 esp_err_to_name(res));
        return;
    }

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(ESP_SPP_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));
}