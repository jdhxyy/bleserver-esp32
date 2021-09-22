// Copyright 2021-2021 The jdh99 Authors. All rights reserved.
// ble服务端
// Authors: jdh99 <jdh821@163.com>

#ifndef BLESERVER_H
#define BLESERVER_H

#include "tztype.h"

// 服务UUID
#define BLE_SERVER_SERVICE_UUID 0xABF0
// 发送UUID
#define BLE_SERVER_TX_UUID 0xABF1
// 接收UUID
#define BLE_SERVER_RX_UUID 0xABF2

// 接收的最大字节数.不能大于500
#define BLE_SERVER_RX_LEN_MAX 500

// 属性状态机
enum {
    IDX_SVC,

    IDX_CHAR_TX,
    IDX_CHAR_VAL_TX,
    IDX_CHAR_CFG_TX,

    IDX_CHAR_RX,
    IDX_CHAR_VAL_RX,

    HRS_IDX_NB,
};

// BleServerLoad 模块载入.deviceName是蓝牙设备名称
// 载入之前需初始化nvs_flash_init
bool BleServerLoad(char* deviceName);

// BleServerIsConnect 是否已连接
bool BleServerIsConnect(void);

// BleServerRegisterObserver 注册接收观察者
// callback是回调函数,接收到数据会回调此函数
bool BleServerRegisterObserver(TZDataFunc callback);

// BleTx 发送数据
bool BleTx(uint8_t* bytes, int size);

#endif
