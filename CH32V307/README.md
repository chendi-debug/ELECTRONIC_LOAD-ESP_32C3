# CH32V307 智能电子负载

该目录保存基于 **CH32V307VCT6（RISC-V，144 MHz）** 和 **RT-Thread 4.0.4** 的智能电子负载固件。工程支持恒流（CC）、恒压（CV）、恒阻（CR）、恒功率（CW）、IV 曲线扫描、电池内阻/健康度检测、Nextion 串口屏、按键、蓝牙/ESP32 通信和按功率调速的散热风扇。

## 目录结构

```text
CH32V307/
├─ ELECTRONIC_LOAD/
│  ├─ applications/       # 电子负载业务逻辑、线程、串口、按键、DAC、PWM
│  ├─ board/              # 时钟、启动和板级初始化
│  ├─ libcpu/             # CH32V307 RISC-V CPU 移植层
│  ├─ libraries/          # WCH 外设库和 RT-Thread BSP 驱动
│  ├─ rt-thread/          # RT-Thread 4.0.4 内核及启用组件
│  ├─ figures/board.png   # 板卡参考图
│  ├─ .config             # RT-Thread Kconfig 配置
│  ├─ rtconfig.h          # 由配置生成的功能宏
│  ├─ SConstruct          # SCons 构建入口
│  └─ .project/.cproject  # RT-Thread Studio 工程配置
└─ .spec-workflow/        # 原工程中的需求/设计工作流资料
```

`packages/` 是 RT-Thread 软件包下载缓存，体积超过 400 MB，未提交。项目配置已记录依赖，可通过 `pkgs --update` 恢复 U8g2 和 qpid。

## 软件架构

- RTOS：RT-Thread 4.0.4，1 kHz 系统节拍，32 级优先级（数值越小，优先级越高）。
- 主线程：栈 4096 字节、优先级 5，完成硬件与业务线程初始化，并周期检查 10 A / 200 W 保护条件。
- 命令行：FinSH/MSH `tshell`，栈 2048 字节、优先级 11，控制台为 UART1。
- 控制算法：qpid 位置式 PID。CC 由 ADC 线程闭环调节；CV 目前按设定值直接驱动 DAC1，由外部模拟反馈环闭环；CR/CW 由后台线程根据实时电压换算目标电流。
- 采样：ADC1 四通道扫描，DMA1 Channel 1 循环搬运；应用线程进行均值、低通、比例和零点补偿。
- 输出：DAC1/DAC2 与 TIM4_CH4 共同产生电压/双路电流参考；TIM8_CH1 输出低极性四线风扇 PWM。
- 通信：UART3 连接 Nextion HMI；UART5 同时承载蓝牙/ESP32 控制帧和 JSON 状态推送。

## 实际启用的应用线程

以下数据直接来自 `rt_thread_create()`。除 RT-Thread 自带的 `main`、`tshell`、空闲线程等系统线程外，应用共创建 **12 个工作线程**。

| 线程名 | 入口/来源 | 栈 | 优先级 | 时间片 | 主要职责 |
| --- | --- | ---: | ---: | ---: | --- |
| `ONOFF` | `thread5_ONOFF_entry` | 1024 | 15 | 25 | 最高优先级业务线程；处理实体键/HMI/远程开关请求，切换 CC/CV/CR/CW 输出，控制 DAC/PWM 和负载状态灯 |
| `ADC` | `thread3_ADC_entry` | 1536 | 18 | 30 | 初始化 ADC1+DMA，约每 5 ms 处理电压、电流和电池电压采样；执行 CC PID/CV 输出更新 |
| `CWCR` | `thread9_CWCR_entry` | 2048 | 19 | 15 | 每 50 ms 更新 CW/CR 目标电流，并串行执行 IV 扫描或电池检测任务 |
| `KEY` | `thread6_KEY_entry` | 512 | 20 | 20 | 每 20 ms 扫描 PD8/PD9/PD10，运行按键消抖状态机 |
| `UART3` | `UART3_data_parsing` | 1024 | 23 | 30 | 等待 UART3 接收信号量，解析 `0x40 ... 0xFF 0xFF 0xFF` HMI 帧 |
| `HMI_GetDate` | `thread4_HMI_GetDate_entry` | 2048 | 23 | 30 | 消费 UART3 数据，处理页面切换、参数设置、启停、IV 扫描和电池测试请求；同时处理实体按键事件 |
| `BlueTooth` | `thread10_BlueTooth_entry` | 2048 | 23 | 30 | 消费 UART5 控制帧，支持蓝牙/ESP32 网页侧的模式、参数、启停和电池测试命令 |
| `UART5` | `UART5_data_parsing` | 1024 | 25 | 30 | 等待 UART5 接收信号量并解析远程控制帧 |
| `HMI_Display` | `thread7_HMI_Display_entry` | 2048 | 25 | 30 | 向串口屏刷新电压、电流、功率、等效电阻和电池电压；IV 扫描时暂停常规刷新 |
| `FAN` | `thread8_FAN_entry` | 512 | 26 | 15 | 每 200 ms 按实时功率分档控制风扇 PWM；低功率维持最低转速，60 W 以上满转 |
| `ESP32Push` | `thread11_ESP32_Push_entry` | 1024 | 27 | 20 | 每 200 ms 通过 UART5 推送电压、电流、功率、模式和输出状态 JSON |
| `SYS_LED` | `thread1_sysLED_entry` | 512 | 30 | 5 | 每 500 ms 翻转系统运行指示灯 |

源码中还保留了 `OLED_Display` 线程实现和创建代码，但当前已整体注释，因此不属于运行时线程。

## 引脚与外设用途

| MCU 引脚 | 外设/信号 | 方向 | 源码中的用途 |
| --- | --- | --- | --- |
| PA0 | ADC1_IN0 / `YVF` | 模拟输入 | 负载端电压采样，经约 0.03219 分压比还原实际电压 |
| PA1 | ADC1_IN1 / `YIF1` | 模拟输入 | 第一路 MOSFET 电流采样，10 mΩ 采样电阻、约 50 倍运放增益 |
| PA2 | ADC1_IN2 / `YIF2` | 模拟输入 | 第二路 MOSFET 电流采样；与 `YIF1` 相加得到总电流 |
| PA3 | ADC1_IN3 / `VBAT` | 模拟输入 | 电池检测电压采样 |
| PA4 | DAC1 / `VREF` | 模拟输出 | 恒压参考；代码中 4095 表示关闭恒压支路，负载工作时按目标电压输出 |
| PA5 | DAC2 / `IREF1` | 模拟输出 | 第一电流参考，主要覆盖较低电流区间 |
| PA9 | USART1_TX | 复用输出 | RT-Thread FinSH/MSH 控制台输出 |
| PA10 | USART1_RX | 输入 | RT-Thread FinSH/MSH 控制台输入 |
| PB6 | 软件 I²C1 SCL | 开漏双向 | SSD1306 OLED 时钟；配置仍启用，但 OLED 显示线程当前停用 |
| PB7 | 软件 I²C1 SDA | 开漏双向 | SSD1306 OLED 数据；配置仍启用，但 OLED 显示线程当前停用 |
| PB9 | TIM4_CH4 / `IREF2` | PWM 输出 | 第二电流参考；与 DAC2 配合扩展较高电流范围，PWM 高极性 |
| PB10 | USART3_TX | 复用输出 | 向 Nextion HMI 发送页面、文本、测量值和 IV 曲线指令，921600 baud |
| PB11 | USART3_RX | 输入 | 接收 Nextion HMI 控制帧，921600 baud |
| PC6 | TIM8_CH1 / FAN PWM | PWM 输出 | 四线散热风扇调速，低极性；CCR 越小转速越高 |
| PC10 | UART4_TX | 复用输出 | BSP 中已启用的备用/调试串口；当前业务代码没有绑定协议 |
| PC11 | UART4_RX | 输入 | BSP 中已启用的备用/调试串口；当前业务代码没有绑定协议 |
| PC12 | UART5_TX | 复用输出 | 向 ESP32/蓝牙模块推送 JSON、IV 曲线和电池检测数据，921600 baud |
| PD2 | UART5_RX | 输入 | 接收蓝牙/ESP32 网页侧控制帧，921600 baud |
| PD8 | KEY3 | 上拉输入 | 菜单/返回键；低电平按下 |
| PD9 | KEY2 | 上拉输入 | 工作模式切换键；低电平按下 |
| PD10 | KEY1 | 上拉输入 | 电子负载输出启停键；低电平按下 |
| PD11 | LED2 | 数字输出 | 负载输出状态灯：输出关闭时高电平，开启时低电平 |
| PD12 | LED1 | 数字输出 | 系统心跳灯，每 500 ms 翻转 |
| PD14 | `MCU_G0` | 数字输出 | 电压采样量程控制位 0；当前因三档切换电路问题固定输出低电平 |
| PD15 | `MCU_G1` | 数字输出 | 电压采样量程控制位 1；当前固定输出低电平 |

内部外设还使用 DMA1 Channel 1 搬运 ADC1 数据，以及独立看门狗设备 `wdt`。看门狗初始化函数已经实现，但 `SYS_Init()` 中的调用当前被注释。

## 控制与通信流程

1. `main()` 初始化 UART、DAC、PWM，再创建业务线程。
2. ADC 线程连续获得四路采样值，换算 `YVF`、`YIF1`、`YIF2`、`YIF` 和 `VBAT`。
3. HMI、实体按键或 UART5 远程命令更新模式及设定值，并把启停请求交给 `ONOFF` 线程。
4. CC 使用 PID 调整 DAC2/PWM；CV 直接计算 DAC1；CR/CW 每 50 ms 根据实时电压换算所需电流。
5. HMI 显示线程刷新本地串口屏，ESP32 推送线程以 JSON 形式发送状态。
6. 主线程每 2 秒检查总电流是否超过 10 A 或功率是否超过 200 W，命中后触发关闭请求。

UART3/UART5 的控制帧均以 `0x40` 开始，以连续三个 `0xFF` 结束。UART5 状态推送示例：

```json
{"v":1200,"i":150,"p":1800,"m":0,"o":1}
```

数值采用整数缩放：电压、电流和功率通常按 ×100 发送；`m` 表示模式（0=CC、1=CV、2=CR、3=CW、4=菜单），`o` 表示负载输出状态。

## 开发与构建

推荐环境：RT-Thread Studio 2.2.9、RISC-V GCC（`riscv-none-embed`）和 WCH-Link。

1. 在 RT-Thread Env/Studio 中进入 `ELECTRONIC_LOAD`。
2. 执行 `pkgs --update` 下载 `.config` 中声明的 U8g2 与 qpid 软件包。
3. 使用 RT-Thread Studio 导入 `.project`，或运行 `scons` 构建。
4. 使用 WCH-Link 将生成固件烧录到 CH32V307VCT6。

构建目录、二进制、对象文件、软件包缓存和嵌套 Git 元数据不纳入仓库。
