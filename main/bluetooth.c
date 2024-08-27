#include "bluetooth.h"

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"

#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "esp_log.h"

#define DEVICE_NAME "ESP_SPP_SERVER"
#define UPDATE_VALUE_TAG "UPDATE_VALUE"

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
// #define ESP_SPP_APP_ID 0x79
#define SPP_SVC_INST_ID 0

#define CHAR_DECLARATION_SIZE sizeof(uint8_t)
#define CHAR_VALUE_SIZE sizeof(uint16_t)

// SPP service UUID
static uint8_t spp_serv_uuid[] = {0x01, 0xF0, 0xD2, 0x50, 0x37, 0xF9,
                                  0x4E, 0x32, 0x9B, 0x59, 0x3C, 0x7A,
                                  0xC4, 0xC6, 0xD4, 0xCD};
// Characteristic UUID
static uint8_t spp_hs_char_uuid[] = {0x02, 0xF0, 0xD2, 0x50, 0x37, 0xF9,
                                  0x4E, 0x32, 0x9B, 0x59, 0x3C, 0x7A,
                                  0xC4, 0xC6, 0xD4, 0xCD};

static uint8_t spp_bp_char_uuid[] = {0x03, 0xF0, 0xD2, 0x50, 0x37, 0xF9,
                                  0x4E, 0x32, 0x9B, 0x59, 0x3C, 0x7A,
                                  0xC4, 0xC6, 0xD4, 0xCD};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] =
        {
            .gatts_cb = gatts_profile_event_handler,
            .gatts_if = ESP_GATT_IF_NONE,
        },
};

uint16_t spp_handle_tab[SPP_IDX_NUM];

static const uint16_t pri_serv_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t char_declar_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t char_ccc_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t char_prop_read_notify =
    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static uint16_t hs_value = 0;
static uint16_t bp_value = 0;
static uint16_t spp_hs_char_ccc_val = 0;
static uint16_t spp_bp_char_ccc_val = 0;
static bool spp_hs_enable_notif = false;
static bool spp_bp_enable_notif = false;

static const esp_gatts_attr_db_t gatt_db[] = {
    [SPP_IDX_SVC] =
        {
            .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
            .att_desc =
                {
                    .uuid_length = ESP_UUID_LEN_16,
                    .uuid_p = (uint8_t *)&pri_serv_uuid,
                    .perm = ESP_GATT_PERM_READ,
                    .max_length = ESP_UUID_LEN_128,
                    .length = sizeof(spp_serv_uuid),
                    .value = spp_serv_uuid,
                },
        },
    [SPP_IDX_HEARTSOUND_CHAR] =
        {
            .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
            .att_desc =
                {
                    .uuid_length = ESP_UUID_LEN_16,
                    .uuid_p = (uint8_t *)&char_declar_uuid,
                    .perm = ESP_GATT_PERM_READ,
                    .max_length = CHAR_DECLARATION_SIZE,
                    .length = CHAR_DECLARATION_SIZE,
                    .value = (uint8_t *)&char_prop_read_notify,
                },
        },
    [SPP_IDX_HEARTSOUND_VAL] =
        {
            .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
            .att_desc =
                {
                    .uuid_length = ESP_UUID_LEN_128,
                    .uuid_p = spp_hs_char_uuid,
                    .perm = ESP_GATT_PERM_READ,
                    .max_length = CHAR_VALUE_SIZE,
                    .length = CHAR_VALUE_SIZE,
                    .value = (uint8_t *)&hs_value,
                },
        },
    [SPP_IDX_HEARTSOUND_CCC] =
        {
            .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
            .att_desc =
                {
                    .uuid_length = ESP_UUID_LEN_16,
                    .uuid_p = (uint8_t *)&char_ccc_uuid,
                    .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                    .max_length = 2,
                    .length = 2,
                    .value = (uint8_t *)&spp_hs_char_ccc_val,
                },
        },
    [SPP_IDX_BLOODPRESSURE_CHAR] =
        {
            .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
            .att_desc =
                {
                    .uuid_length = ESP_UUID_LEN_16,
                    .uuid_p = (uint8_t *)&char_declar_uuid,
                    .perm = ESP_GATT_PERM_READ,
                    .max_length = CHAR_DECLARATION_SIZE,
                    .length = CHAR_DECLARATION_SIZE,
                    .value = (uint8_t *)&char_prop_read_notify,
                },
        },
    [SPP_IDX_BLOODPRESSURE_VAL] =
        {
            .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
            .att_desc =
                {
                    .uuid_length = ESP_UUID_LEN_128,
                    .uuid_p = spp_bp_char_uuid,
                    .perm = ESP_GATT_PERM_READ,
                    .max_length = CHAR_VALUE_SIZE,
                    .length = CHAR_VALUE_SIZE,
                    .value = (uint8_t *)&bp_value,
                },
        },
    [SPP_IDX_BLOODPRESSURE_CCC] =
        {
            .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
            .att_desc =
                {
                    .uuid_length = ESP_UUID_LEN_16,
                    .uuid_p = (uint8_t *)&char_ccc_uuid,
                    .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                    .max_length = 2,
                    .length = 2,
                    .value = (uint8_t *)&spp_bp_char_ccc_val,
                },
        },
};

static esp_ble_adv_data_t spp_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0020,
    .max_interval = 0x0040,
    .appearance = 0x008D,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(spp_serv_uuid),
    .p_service_uuid = spp_serv_uuid,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static esp_ble_adv_data_t spp_scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0020,
    .max_interval = 0x0040,
    .appearance = 0x008D,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(spp_serv_uuid),
    .p_service_uuid = spp_serv_uuid,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x0020,
    .adv_int_max = 0x0020,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
    esp_bt_status_t res;

    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            res = param->adv_data_cmpl.status;
            if (res == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(GATTS_TABLE_TAG,
                         "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT status 0");
                esp_ble_gap_start_advertising(&spp_adv_params);
            } else {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT status %d",
                         res);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            res = param->scan_rsp_data_cmpl.status;
            if (res == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(GATTS_TABLE_TAG,
                         "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT status 0");
                esp_ble_gap_start_advertising(&spp_adv_params);
            } else {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT status %d",
                         res);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            res = param->adv_start_cmpl.status;
            if (res == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(GATTS_TABLE_TAG,
                         "ESP_GAP_BLE_ADV_START_COMPLETE_EVT status 0");
            } else {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "ESP_GAP_BLE_ADV_START_COMPLETE_EVT status %d", res);
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            res = param->adv_stop_cmpl.status;
            if (res == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(GATTS_TABLE_TAG,
                         "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT succeed with a "
                         "connection being created");
            } else {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT status %d", res);
            }
            break;
        default:
            break;
    }
}

static TaskHandle_t handle_update_hs_val = NULL;
static TaskHandle_t handle_update_bp_val = NULL;
static TaskHandle_t handle_update_val = NULL;
static TickType_t hs_last_wake_time;
static TickType_t bp_last_wake_time;
static TickType_t last_wake_time;
static const TickType_t freq = 1;

static void update_hs_value(void *pvParameters) {
    ESP_LOGI(UPDATE_VALUE_TAG, "update hs value task started");
    sample_fn_t sample_fn = *((sample_fn_t *)pvParameters);
    while (true) {
        ESP_ERROR_CHECK(sample_fn.fn(&(sample_fn.mcp320x), &hs_value, CH0));
        ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(spp_handle_tab[SPP_IDX_HEARTSOUND_VAL], sizeof(hs_value), (uint8_t *)&hs_value));
        xTaskDelayUntil(&hs_last_wake_time, freq);
    }
}

static void update_bp_value(void *pvParameters) {
    ESP_LOGI(UPDATE_VALUE_TAG, "update bp value task started");
    sample_fn_t sample_fn = *((sample_fn_t *)pvParameters);
    while (true) {
        ESP_ERROR_CHECK(sample_fn.fn(&(sample_fn.mcp320x), &hs_value, CH2));
        ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(spp_handle_tab[SPP_IDX_BLOODPRESSURE_VAL], sizeof(bp_value), (uint8_t *)&bp_value));
        xTaskDelayUntil(&bp_last_wake_time, freq);
    }
}

static void update_both_value(void *pvParameters) {
    ESP_LOGI(UPDATE_VALUE_TAG, "update hs and bp value task started");
    sample_fn_t sample_fn = *((sample_fn_t *)pvParameters);
    while (true) {
        ESP_ERROR_CHECK(sample_fn.fn(&(sample_fn.mcp320x), &hs_value, CH0));
        ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(spp_handle_tab[SPP_IDX_HEARTSOUND_VAL], sizeof(hs_value), (uint8_t *)&hs_value));
        ESP_ERROR_CHECK(sample_fn.fn(&(sample_fn.mcp320x), &hs_value, CH2));
        ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(spp_handle_tab[SPP_IDX_BLOODPRESSURE_VAL], sizeof(bp_value), (uint8_t *)&bp_value)); xTaskDelayUntil(&last_wake_time, freq);
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
    esp_err_t res;

    switch (event) {
        case ESP_GATTS_REG_EVT:
            res = esp_ble_gap_set_device_name(DEVICE_NAME);
            if (res) {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "set device name failed, error code = %x", res);
            }
            res = esp_ble_gap_config_adv_data(&spp_adv_data);
            if (res) {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "config adv data failed, error code = %x ", res);
            }
            res = esp_ble_gap_config_adv_data(&spp_scan_rsp_data);
            if (res) {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "config scan response data failed, error code = %x ",
                         res);
            }
            res = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, SPP_IDX_NUM,
                                                SPP_SVC_INST_ID);
            if (res) {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "config scan response data failed, error code = %x ",
                         res);
            }
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVENT");
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVENT");
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
            if (param->write.handle == spp_handle_tab[SPP_IDX_HEARTSOUND_CCC]) {
                switch ((param->write.value)[0]) {
                case 0x00:
                    spp_hs_enable_notif = false;
                    break;
                case 0x01:
                    spp_hs_enable_notif = true;
                    hs_last_wake_time = xTaskGetTickCount();
                    if (handle_update_bp_val != NULL) {
                        xTaskDelete(handle_update_bp_val);
                        handle_update_bp_val = NULL;
                        xTaskCreate(update_both_value, "update hs and bp value", 8192, (void *) &sample_fn, 1, &handle_update_val);
                    } else {
                        xTaskCreate(update_hs_value, "update hs_value", 8192, (void *) &sample_fn, 1, &handle_update_hs_val);
                    }
                    break;
                default:
                    break;
                }
            }
            if (param->write.handle == spp_handle_tab[SPP_IDX_BLOODPRESSURE_CCC]) {
                switch ((param->write.value)[0]) {
                case 0x00:
                    spp_bp_enable_notif = false;
                    break;
                case 0x01:
                    spp_bp_enable_notif = true;
                    bp_last_wake_time = xTaskGetTickCount();
                    if (handle_update_hs_val != NULL) {
                        xTaskDelete(handle_update_hs_val);
                        handle_update_hs_val = NULL;
                        xTaskCreate(update_both_value, "update hs and bp value", 8192, (void *) &sample_fn, 1, &handle_update_val);
                    } else {
                        xTaskCreate(update_bp_value, "update bp_value", 8192, (void *) &sample_fn, 1, &handle_update_bp_val);
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        case ESP_GATTS_SET_ATTR_VAL_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_SET_ATTR_VAL_EVT");
            if (param->set_attr_val.attr_handle == spp_handle_tab[SPP_IDX_HEARTSOUND_VAL] && spp_hs_enable_notif) {
                res = esp_ble_gatts_send_indicate(
                    spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if,
                    spp_profile_tab[SPP_PROFILE_APP_IDX].conn_id,
                    spp_handle_tab[SPP_IDX_HEARTSOUND_VAL], sizeof(hs_value),
                    (uint8_t *)&hs_value, false);
                if (res) {
                    ESP_LOGE(GATTS_TABLE_TAG, "send notification failed %d", res);
                } else {
                    ESP_LOGI(GATTS_TABLE_TAG, "send notification successfully");
                }
            }
            if (param->set_attr_val.attr_handle == spp_handle_tab[SPP_IDX_BLOODPRESSURE_VAL] && spp_bp_enable_notif) {
                res = esp_ble_gatts_send_indicate(
                    spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if,
                    spp_profile_tab[SPP_PROFILE_APP_IDX].conn_id,
                    spp_handle_tab[SPP_IDX_BLOODPRESSURE_VAL], sizeof(bp_value),
                    (uint8_t *)&bp_value, false);
                if (res) {
                    ESP_LOGE(GATTS_TABLE_TAG, "send notification failed %d", res);
                } else {
                    ESP_LOGI(GATTS_TABLE_TAG, "send notification successfully");
                }
            }
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG,
                     "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d",
                     param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG,
                     "SERVICE_START_EVT, status %d, service_handle %d",
                     param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d",
                     param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda,
                   sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;  // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;  // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;   // timeout = 400*10ms = 4000ms
            // start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            // add conn_id to app profile table
            spp_profile_tab[SPP_PROFILE_APP_IDX].conn_id =
                param->connect.conn_id;
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x",
                     param->disconnect.reason);
            if (handle_update_hs_val) {
                vTaskDelete(handle_update_hs_val);
                handle_update_hs_val = NULL;
            }
            if (handle_update_bp_val) {
                vTaskDelete(handle_update_bp_val);
                handle_update_bp_val = NULL;
            }
            if (handle_update_val) {
                vTaskDelete(handle_update_val);
                handle_update_val = NULL;
            }
            // reset client configuration descriptor
            uint8_t default_val[2] = {0x00, 0x00};
            esp_ble_gatts_set_attr_value(spp_handle_tab[SPP_IDX_HEARTSOUND_CCC], sizeof(default_val), default_val);
            spp_hs_enable_notif = false;
            esp_ble_gatts_set_attr_value(spp_handle_tab[SPP_IDX_BLOODPRESSURE_CCC], sizeof(default_val), default_val);
            spp_bp_enable_notif = false;
            // start advertising again
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "create attribute table failed, error code=0x%x",
                         param->add_attr_tab.status);
            } else if (param->add_attr_tab.num_handle != SPP_IDX_NUM) {
                ESP_LOGE(GATTS_TABLE_TAG,
                         "create attribute table abnormally, num_handle (%d) \
                    doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, SPP_IDX_NUM);
            } else {
                ESP_LOGI(GATTS_TABLE_TAG,
                         "create attribute table successfully, the number "
                         "handle = %d",
                         param->add_attr_tab.num_handle);
                memcpy(spp_handle_tab, param->add_attr_tab.handles,
                       sizeof(spp_handle_tab));
                esp_ble_gatts_start_service(spp_handle_tab[SPP_IDX_SVC]);
            }
            break;
        default:
            break;
    };
}

void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            ESP_LOGI(GATTS_TABLE_TAG, "reg app successfully, app_id %04x",
                     param->reg.app_id);
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id, param->reg.status);
            return;
        }
    }
    do {
        for (size_t idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call
             * every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE ||
                gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}