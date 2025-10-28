# STM32 SmartCharge HAL 项目

一个基于 STM32F103 的环境监测与控制项目，集成 LVGL 图形界面、FreeRTOS 任务调度、HLW8032 电能计量、DHT11 温湿度、HC‑SR501 人体红外、继电器与风扇控制等功能。工程使用 HAL 库与 Keil MDK‑ARM 构建，支持在 CubeMX 继续改动硬件配置。

## 功能模块
- LVGL 显示界面（ST7735S SPI 屏幕）
- FreeRTOS 多任务与定时器
- HLW8032 电能计量（电压/电流/功率/功率因数/电能）
- DHT11 温湿度采集
- HC‑SR501 人体红外检测
- 光敏传感器（数字/模拟）
- 继电器与风扇 PWM 控制
- ESP8266 串口通信（预留）

## 硬件连接速览
详细请见 `引脚连接表.md`，以下为关键引脚：
- DHT11 DATA：`PB4`（需禁用 JTAG，调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`）
- HC‑SR501 OUT：`PC12`
- HLW8032 UART：当前固件使用 `USART2 RX` 4800bps 解析数据（默认引脚 `PA3`）。若你的硬件仍接在 `USART3 RX (PC11)`，请相应调整 `usart.c` 与 `stm32f1xx_it.c` 的串口与中断来源。

## 开发环境
- Keil MDK‑ARM（uVision5）
- STM32CubeMX（可打开 `STM32F103.ioc` 调整外设）
- STM32 HAL 驱动、FreeRTOS、LVGL 已集成于仓库

## 构建与烧录
1. 使用 Keil 打开 `MDK-ARM/STM32F103.uvprojx`
2. 选择正确的目标与优化等级，编译生成固件
3. 通过 ST‑Link 或其他工具烧录到目标板

## 运行与调试
- HLW8032：固件通过 `USART2` RXNE 中断接收帧，在任务中调用 `hlw8032_get_data()` 更新与显示电参数据。
- DHT11：在 BSP 中实现单总线读写，初始化时需确保 `PB4` 可用为 GPIO（禁用 JTAG）。
- SR501：读取 `PC12` 电平，高=有人体，低=无人。
- 继电器与风扇：见 `Bsp/RELAY` 与 `Bsp/FAN`。

## 目录结构
- `Core/`：启动文件、外设初始化、FreeRTOS 入口、系统中断
- `Bsp/`：各模块驱动（HLW8032、DHT11、SR501、LCD、UART、W25Q128 等）
- `Drivers/`：HAL/CMSIS 驱动库
- `LVGL/`：LVGL 框架与示例
- `MDK-ARM/`：Keil 工程文件
- `System/`：系统延时与基础工具

## 注意事项
- Windows 上 Git 使用系统证书（schannel）以便 HTTPS 访问
- 已提供 `.gitignore` 忽略编译产物与临时文件
- 更新硬件连线请同步修改 `STM32F103.ioc` 并在 `Core/Src` 与 `Bsp/` 中检查对应外设初始化与中断来源

## 许可证
- 本仓库包含 MIT 许可证（见 `LICENSE`）。

## 致谢
- STMicroelectronics HAL 库、FreeRTOS、LVGL 等开源组件