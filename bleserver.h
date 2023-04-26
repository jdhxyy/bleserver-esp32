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

// BleServerLoadBySN 模块载入.deviceName是蓝牙设备名称.sn最大12个字节
// 载入之前需初始化nvs_flash_init
bool BleServerLoadBySN(char *deviceName, char *sn);

// BleServerLoadByMac 模块载入.deviceName是蓝牙设备名称
// 载入之前需初始化nvs_flash_init
bool BleServerLoadByMac(char *deviceName);

// BleServerIsConnect 是否已连接
bool BleServerIsConnect(void);

// BleServerRegisterObserver 注册接收观察者
// callback是回调函数,接收到数据会回调此函数
bool BleServerRegisterObserver(TZDataFunc callback);

// BleServerIsAllowTx 是否允许发送
bool BleServerIsAllowTx(void);

// BleServerTx 发送数据
bool BleServerTx(uint8_t* bytes, int size);

// BleServerDisconnect 断开连接
void BleServerDisconnect(void);

// BleServerGetMac 读取MAC地址
void BleServerGetMac(uint8_t mac[6]);

// BleServerGetBleName 读取BLE名称
char *BleServerGetBleName(void);

// BleServerGetRssi 获取蓝牙Rssi值
int8_t BleServerGetRssi(void);

#endif
