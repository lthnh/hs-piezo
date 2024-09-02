#include "include/adc.h"

#include "driver/spi_master.h"
#include "esp_check.h"
#include "hal/spi_types.h"

static spi_device_interface_config_t mcp320x_spi_config = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 0,
    .duty_cycle_pos = 128,
    // .cs_ena_pretrans = 0,  // only works on half-duplex transactions
    .cs_ena_posttrans = 0,
    .clock_speed_hz = -1, // configure later
    .input_delay_ns = 200,
    .flags = SPI_DEVICE_NO_DUMMY,
    .spics_io_num = -1, // configure later
    .queue_size = 1,
    .pre_cb = NULL,
    .post_cb = NULL,
};

static spi_transaction_t trans_desc = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .length = 3 * 8,
    .rxlength = 3 * 8,  // defaults the value to length
    .tx_data = {START_SINGLE, SEL_CH0},
};

esp_err_t mcp320x_add(spi_host_device_t host, mcp320x_t *mcp320x) {
    spi_device_interface_config_t spi_config = mcp320x_spi_config;
    spi_config.clock_speed_hz = mcp320x->expected_sampl_freq;
    spi_config.spics_io_num = mcp320x->cs_pin;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(host, &spi_config, &(mcp320x->handle)), ADC_TAG, "%s", "add adc fails");

    int actual_freq = 0;
    spi_device_get_actual_freq(mcp320x->handle, &actual_freq);
    ESP_LOGI(ADC_TAG, "actual frequency: %d (kHz)", actual_freq);
    return ESP_OK;
}

esp_err_t mcp320x_sample(mcp320x_t *mcp320x, uint16_t *result, mcp320x_ch_t ch) {
    *result = 0;
    if (mcp320x->single == false && trans_desc.tx_data[0] != START) {
        trans_desc.tx_data[0] = START;
    }
    if (trans_desc.tx_data[1] != SEL_CH(ch)) {
        trans_desc.tx_data[1] = SEL_CH(ch);
    }
    for (int i = 0; i < mcp320x->sampl_count; ++i) {
        ESP_RETURN_ON_ERROR(
            spi_device_transmit(mcp320x->handle, &trans_desc), "adc",
            "host fails to start a transaction");
        uint16_t value =
            (trans_desc.rx_data[1] & 0x0F) << 8 | trans_desc.rx_data[2];
        *result += value;
        // esp_log_buffer_hex(ADC_TAG, trans_desc.rx_data, 3);
        // float voltage = (float)value / 4096.0 * mcp320x->vref;
        // printf("%u %.02f\n", value, voltage);
    }
    *result /= mcp320x->sampl_count;
    return ESP_OK;
}

esp_err_t mcp320x_remove(mcp320x_t *mcp320x) {
    return spi_bus_remove_device(mcp320x->handle);
}
