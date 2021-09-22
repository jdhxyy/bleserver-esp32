# bleserver

## 1. 介绍
esp32下的ble服务器驱动。

## 2. 功能
- 接收数据，推送给应用模块
- 发送数据
- 最大接收帧可设置为500，发送端可以通过设置MTU来发送长帧

## 3. UUID
UUID|功能
-|-
0xABF0|服务
0xABF1|ble发送
0xABF2|ble接收

## 4. 初始化API
```c
// BleServerLoad 模块载入.deviceName是蓝牙设备名称
// 载入之前需初始化nvs_flash_init
bool BleServerLoad(char* deviceName);
```

注意：BleServerLoad之前需初始：
```c
ESP_ERROR_CHECK(nvs_flash_init());
```

## 5. 注册接收回调
```c
// BleServerRegisterObserver 注册接收观察者
// callback是回调函数,接收到数据会回调此函数
bool BleServerRegisterObserver(TZDataFunc callback);
```

## 6. 发送数据
```c
// BleTx 发送数据
bool BleTx(uint8_t* bytes, int size);
```
