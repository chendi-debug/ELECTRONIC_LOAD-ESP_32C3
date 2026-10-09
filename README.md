# ESP32-C3 与 CH32V307 项目

该仓库按 MCU 平台分为两个独立的顶层目录，避免原有 ESP32 代码和新增 CH32V307 工程混在一起。

```text
├─ ESP32/       # 原仓库中的 ESP32/ESP32-C3 项目
└─ CH32V307/    # 基于 RT-Thread 的 CH32V307 智能电子负载
```

## 项目导航

- [ESP32-C3 WiFi 数据中转模块](ESP32/README.md)：保留原仓库代码，负责 UART、WiFi AP、HTTP/WebSocket 和网页控制。
- [CH32V307 智能电子负载](CH32V307/README.md)：包含功能架构、RT-Thread 线程、引脚分配、通信协议和构建说明。

两个目录是独立工程，使用不同的芯片、SDK/RTOS 和构建工具。修改或构建时请进入对应目录操作。
