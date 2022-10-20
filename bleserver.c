// Copyright 2021-2021 The jdh99 Authors. All rights reserved.
// ble服务端
// Authors: jdh99 <jdh821@163.com>

#include "bleserver.h"
#include "tztype.h"
#include "lagan.h"
#include "tzlist.h"
#include "tzmalloc.h"
#include "async.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "bleserver.h"
#include "esp_gatt_common_api.h"

#define TAG "bleserver"

// tzmalloc字节数
#define MALLOC_TOTAL 4096

#define MTU_LEN_DEFAULT 23

// 连接间隔.单位:1.25ms
// 最小间隔7.5ms.即为6
// ios是30ms.即24
#define INTERVAL_MIN 6
#define INTERVAL_MAX 6

#pragma pack(1)

// 观察者回调函数
typedef struct {
    TZDataFunc callback;
} tItem;

#pragma pack()

static int mid = -1;

// 存储观察者列表
static intptr_t list = 0;

static bool isConnect = false;
static bool isNotifyEnable = false;

// 接收缓存
static TZBufferDynamic* rxBuffer;

// ble连接参数
static uint16_t connID = 0xffff;
static esp_gatt_if_t gattsIF = 0xff;
static int mtuLen = MTU_LEN_DEFAULT;

// ble MAC地址
static uint8_t bleMac[6] = {0};

// 可以存储ble mac或者是sn号的后6位
static TZBufferTiny gBuffer = {0};

static char gDeviceName[32] = {0};

static bool initRawAdvData(char* deviceName, uint8_t* payload, int payloadLen);
static int task(void);
static void notifyObserver(void);
static bool isObserverExist(TZDataFunc callback);
static TZListNode* createNode(void);

// 驱动模块参数
#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0;

uint16_t handleTable[HRS_IDX_NB];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

// 广播数据
static TZBufferTiny rawAdvData = {0};
static TZBufferTiny rawScanRspData = {0};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

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
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

// 服务UUID
static const uint16_t GATTS_SERVICE_UUID = BLE_SERVER_SERVICE_UUID;
// 发送
static const uint16_t GATTS_CHAR_TX_UUID = BLE_SERVER_TX_UUID;
// 接收
static const uint16_t GATTS_CHAR_RX_UUID = BLE_SERVER_RX_UUID;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID), (uint8_t *)&GATTS_SERVICE_UUID}},

    /* Characteristic Declaration */
    [IDX_CHAR_TX]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_TX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_TX_UUID, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_TX]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

    /* Characteristic Declaration */
    [IDX_CHAR_RX]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_RX]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_RX_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);

        // 保存数据
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT rx data\n");

        do {
            if (prepare_write_env->prepare_len <= 0) {
                LE(TAG, "rx buffer len is wrong:%d", prepare_write_env->prepare_len);
                break;
            }
            if (rxBuffer->len > 0) {
                LW(TAG, "deal data is too slow.throw frame!");
                break;
            }
            memcpy(rxBuffer->buf, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
            rxBuffer->len = prepare_write_env->prepare_len;
        } while (0);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name((char*)rawAdvData.buf);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }

            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(rawAdvData.buf, rawAdvData.len);
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(rawScanRspData.buf, rawScanRspData.len);
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;

            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                // esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);

                if (handleTable[IDX_CHAR_CFG_TX] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        isNotifyEnable = true;
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, handleTable[IDX_CHAR_VAL_TX],
                                            sizeof(indicate_data), indicate_data, true);
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                        isNotifyEnable = false;
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                    }

                }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }

                // 保存数据
                if (handleTable[IDX_CHAR_VAL_RX] == param->write.handle){
                    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT rx data\n");

                    if (param->write.len <= 0) {
                        LE(TAG, "rx buffer len is wrong:%d", param->write.len);
                        break;
                    }
                    if (rxBuffer->len > 0) {
                        LW(TAG, "deal data is too slow.throw frame!");
                        break;
                    }
                    memcpy(rxBuffer->buf, param->write.value, param->write.len);
                    rxBuffer->len = param->write.len;
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            mtuLen = param->mtu.mtu;
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = INTERVAL_MAX;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = INTERVAL_MIN;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);

            // 保存参数
            connID = param->connect.conn_id;
            gattsIF = gatts_if;
            isConnect = true;
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            isConnect = false;
            isNotifyEnable = false;
            mtuLen = MTU_LEN_DEFAULT;
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(handleTable, param->add_attr_tab.handles, sizeof(handleTable));
                esp_ble_gatts_start_service(handleTable[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

// BleServerLoad 模块载入.deviceName是蓝牙设备名称
// 载入之前需初始化nvs_flash_init
bool BleServerLoad(char* deviceName) {
    mid = TZMallocRegister(0, TAG, MALLOC_TOTAL);
    if (mid == -1) {
        LE(TAG, "load failed!malloc failed");
        return false;
    }
    list = TZListCreateList(mid);
    if (list == 0) {
        LE(TAG, "load failed!create list failed");
        return false;
    }
    rxBuffer = TZMalloc(mid, sizeof(TZBufferDynamic) + BLE_SERVER_RX_LEN_MAX);
    if (rxBuffer == NULL) {
        LE(TAG, "load failed!malloc rx buffer failed");
        return false;
    }

    esp_read_mac(bleMac, ESP_MAC_BT);
    if (gBuffer.len == 0) {
        gBuffer.len = 6;
        memcpy(gBuffer.buf, bleMac, gBuffer.len);
    }

    if (strlen(gDeviceName) == 0) {
        strcpy(gDeviceName, deviceName);
    }

    if (initRawAdvData(deviceName, gBuffer.buf, gBuffer.len) == false) {
        LE(TAG, "load failed!init raw adv data failed");
        return false;
    }

    if (esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
        LE(TAG, "load failed!controller mem release failed");
        return false;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        LE(TAG, "load failed!%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return false;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return false;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return false;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        return false;
    }

    if (AsyncStart(task, ASYNC_NO_WAIT) == false) {
        LE(TAG, "load failed!async start task failed");
    }

    LI(TAG, "load success.device name:%s", deviceName);
    return true;
}

static bool initRawAdvData(char* deviceName, uint8_t* payload, int payloadLen) {
    rawAdvData.len = 0;
    memset(rawAdvData.buf, 0, TZ_BUFFER_TINY_LEN);

    // flags
    rawAdvData.buf[rawAdvData.len++] = 0x02;
    rawAdvData.buf[rawAdvData.len++] = 0x01;
    rawAdvData.buf[rawAdvData.len++] = 0x06;
    // service uuid
    rawAdvData.buf[rawAdvData.len++] = 0x03;
    rawAdvData.buf[rawAdvData.len++] = 0x03;
    rawAdvData.buf[rawAdvData.len++] = (uint8_t)BLE_SERVER_SERVICE_UUID;
    rawAdvData.buf[rawAdvData.len++] = (uint8_t)(BLE_SERVER_SERVICE_UUID >> 8);
    // 厂家数据,ble MAC地址
    rawAdvData.buf[rawAdvData.len++] = 3 + payloadLen;
    rawAdvData.buf[rawAdvData.len++] = 0xFF;
    rawAdvData.buf[rawAdvData.len++] = 0xFF;
    rawAdvData.buf[rawAdvData.len++] = 0xFF;
    memcpy(rawAdvData.buf + rawAdvData.len, payload, payloadLen);
    rawAdvData.len += payloadLen;
    // device name
    int len = strlen(deviceName);
    if (len + rawAdvData.len + 2 > TZ_BUFFER_TINY_LEN) {
        LE(TAG, "init raw adv data failed!device name is too long:%d", len);
        return false;
    }
    rawAdvData.buf[rawAdvData.len++] = len + 1;
    rawAdvData.buf[rawAdvData.len++] = 0x09;
    memcpy(rawAdvData.buf + rawAdvData.len, deviceName, len);
    rawAdvData.len += len;

    // 应答数据
    memcpy(rawScanRspData.buf, rawAdvData.buf, rawAdvData.len);
    rawScanRspData.len = rawAdvData.len;
    return true;
}

// BleServerLoadBySN 模块载入.deviceName是蓝牙设备名称.sn最大10个字节
// 载入之前需初始化nvs_flash_init
bool BleServerLoadBySN(char* deviceName, char* sn) {
    int snLen = strlen(sn);
    if (snLen < 4 || snLen > 10) {
        LE(TAG, "sn len is too long:%d", snLen);
        return false;
    }
    strcpy(gDeviceName, deviceName);
    strcat(gDeviceName, sn + (snLen - 4));
    gBuffer.len = snLen - 4;
    memcpy(gBuffer.buf, (uint8_t *)sn, gBuffer.len);
    if (BleServerLoad(gDeviceName) == false) {
        return false;
    }

    return true;
}

static int task(void) {
    static struct pt pt = {0};

    PT_BEGIN(&pt);

    PT_WAIT_UNTIL(&pt, rxBuffer->len > 0);

    notifyObserver();

    PT_END(&pt);
}

static void notifyObserver(void) {
    TZListNode* node = TZListGetHeader(list);
    tItem* item = NULL;
    for (;;) {
        if (node == NULL) {
            break;
        }

        item = (tItem*)node->Data;
        if (item->callback) {
            item->callback(rxBuffer->buf, rxBuffer->len);
        }

        node = node->Next;
    }
    rxBuffer->len = 0;
}

// BleServerIsConnect 是否已连接
bool BleServerIsConnect(void) {
    return isConnect;
}

// BleServerRegisterObserver 注册接收观察者
// callback是回调函数,接收到数据会回调此函数
bool BleServerRegisterObserver(TZDataFunc callback) {
    if (mid < 0 || callback == NULL) {
        LE(TAG, "register observer failed!mid is wrong or callback is null");
        return false;
    }

    if (isObserverExist(callback)) {
        return true;
    }

    TZListNode* node = createNode();
    if (node == NULL) {
        LE(TAG, "register observer failed!create node is failed");
        return false;
    }
    tItem* item = (tItem*)node->Data;
    item->callback = callback;
    TZListAppend(list, node);
    return true;
}

static bool isObserverExist(TZDataFunc callback) {
    TZListNode* node = TZListGetHeader(list);
    tItem* item = NULL;
    for (;;) {
        if (node == NULL) {
            break;
        }
        item = (tItem*)node->Data;
        if (item->callback == callback) {
            return true;
        }
        node = node->Next;
    }
    return false;
}

static TZListNode* createNode(void) {
    TZListNode* node = TZListCreateNode(list);
    if (node == NULL) {
        return NULL;
    }
    node->Data = TZMalloc(mid, sizeof(tItem));
    if (node->Data == NULL) {
        TZFree(node);
        return NULL;
    }
    return node;
}

// BleServerIsAllowTx 是否允许发送
bool BleServerIsAllowTx(void) {
    return isNotifyEnable;
}

// BleServerTx 发送数据
bool BleServerTx(uint8_t* bytes, int size) {
    if (isNotifyEnable == false) {
        LW(TAG, "ble tx failed!ble is not connect");
        return false;
    }

    // 最大帧长为mtu - 3
    int maxLen = mtuLen - 3;
    LD(TAG, "tx frame.len:%d mtu-3:%d", size, maxLen);
    LaganPrintHex(TAG, LAGAN_LEVEL_DEBUG, bytes, size);

    int offset = 0;
    int txLen = 0;
    for (;;) {
        if (size <= maxLen) {
            txLen = size;
            size = 0;
        } else {
            txLen = maxLen;
            size -= maxLen;
        }

        esp_ble_gatts_send_indicate(gattsIF, connID, handleTable[IDX_CHAR_VAL_TX],
                txLen, bytes + offset, false);
        offset += txLen;
        if (size == 0) {
            break;
        }
    }
    return true;
}

// BleServerDisconnect 断开连接
void BleServerDisconnect(void) {
    if (isConnect == false) {
        return;
    }
    esp_ble_gatts_close(gattsIF, connID);
    isConnect = false;
    isNotifyEnable = false;
}

// BleServerGetMac 读取MAC地址
void BleServerGetMac(uint8_t* mac) {
    memcpy(mac, bleMac, 6);
}

// BleServerGetBleName 读取BLE名称
char *BleServerGetBleName(void) {
    return gDeviceName;
}
