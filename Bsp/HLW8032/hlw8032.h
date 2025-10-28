#ifndef HLW8032_H
#define HLW8032_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// HLW8032 一帧解析后的数据（单位：V、A、W、百分比、能量为计算值）
typedef struct {
    // 原始寄存器值
    uint32_t v_num;      // RX[2..4]
    uint32_t v_den;      // RX[5..7]
    uint32_t i_num;      // RX[8..10]
    uint32_t i_den;      // RX[11..13]
    uint32_t p_num;      // RX[14..16]
    uint32_t p_den;      // RX[17..19]
    uint8_t  status;     // RX[20]
    uint16_t energy_low; // RX[21..22]
    uint8_t  header;     // RX[0], 0x55 或 0xF2

    // 计算量
    float    voltage;       // 电压(V)
    float    current;       // 电流(A)
    float    active_power;  // 有功功率(W)
    float    power_factor;  // 功率因数(0..1)
    double   energy;        // 能量（按寄存器与脉冲组合计算的值）
} HLW8032_Data;

// 初始化（可选）：设置默认校准系数
void hlw8032_init(void);

// 设置校准系数（电压/功率计算中用到的比例系数）
void hlw8032_set_calibration(float kv_ratio, float kp_ratio);

// 清零能量累计（脉冲计数清零）
void hlw8032_reset_energy(void);

// 向解析器喂入单字节（通常放在串口接收中断或轮询中调用）
void hlw8032_feed_byte(uint8_t byte);

// 是否有新帧可读（返回非0表示有新数据）
int hlw8032_frame_ready(void);

// 读取最新一帧数据（读取后保持 ready 状态，供上层反复读取）
void hlw8032_get_data(HLW8032_Data* out);

#ifdef __cplusplus
}
#endif

#endif // HLW8032_H
