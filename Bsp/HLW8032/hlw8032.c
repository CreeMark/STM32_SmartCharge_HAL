#include "hlw8032.h"
#include <string.h>

// 校准系数（与项目经验值一致，可按需调整）
static float s_kv = 3.0006f; // 电压系数
static float s_kp = 3.0006f; // 功率系数

// 解析状态
static uint8_t  s_rx[24];
static uint8_t  s_idx = 0;
static uint8_t  s_checksum = 0;
static uint8_t  s_prev_bit7 = 0; // 用于脉冲翻转计数
static uint32_t s_energy_pulses = 0;
static int      s_ready = 0;
static HLW8032_Data s_last;

void hlw8032_init(void)
{
    memset(&s_last, 0, sizeof(s_last));
    s_idx = 0;
    s_checksum = 0;
    s_prev_bit7 = 0;
    s_energy_pulses = 0;
    s_ready = 0;
}


void hlw8032_set_calibration(float kv_ratio, float kp_ratio)
{
    s_kv = kv_ratio;
    s_kp = kp_ratio;
}

void hlw8032_reset_energy(void)
{
    s_energy_pulses = 0;
    s_last.energy = 0.0;
}

static void hlw8032_parse_frame(void)
{
    // 提取原始寄存器
    uint32_t v_num = (uint32_t)s_rx[2]  * 65536u + (uint32_t)s_rx[3]  * 256u + (uint32_t)s_rx[4];
    uint32_t v_den = (uint32_t)s_rx[5]  * 65536u + (uint32_t)s_rx[6]  * 256u + (uint32_t)s_rx[7];
    uint32_t i_num = (uint32_t)s_rx[8]  * 65536u + (uint32_t)s_rx[9]  * 256u + (uint32_t)s_rx[10];
    uint32_t i_den = (uint32_t)s_rx[11] * 65536u + (uint32_t)s_rx[12] * 256u + (uint32_t)s_rx[13];
    uint32_t p_num = (uint32_t)s_rx[14] * 65536u + (uint32_t)s_rx[15] * 256u + (uint32_t)s_rx[16];
    uint32_t p_den = (uint32_t)s_rx[17] * 65536u + (uint32_t)s_rx[18] * 256u + (uint32_t)s_rx[19];
    uint8_t  status = s_rx[20];
    uint16_t energy_low = (uint16_t)s_rx[21] * 256u + (uint16_t)s_rx[22];

    s_last.v_num = v_num;
    s_last.v_den = v_den;
    s_last.i_num = i_num;
    s_last.i_den = i_den;
    s_last.p_num = p_num;
    s_last.p_den = p_den;
    s_last.status = status;
    s_last.energy_low = energy_low;
    s_last.header = s_rx[0];

    // 计算物理量（避免除零）
    float voltage = 0.0f;
    float current = 0.0f;
    float active_power = 0.0f;
    float power_factor = 0.0f;

    if (v_den) {
        voltage = (float)v_num * s_kv / (float)v_den;
    }
    if (i_den) {
        current = (float)i_num / (float)i_den;
    }
    if (p_den) {
        active_power = (float)p_num * s_kp / (float)p_den;
    }

    // 特殊状态：当报头为 0xF2 且状态位4为0时，判定功率相关量为 0
    if (s_rx[0] == 0xF2 && (((status >> 4) & 0x01) == 0)) {
        current = 0.0f;
        active_power = 0.0f;
        power_factor = 0.0f;
    } else {
        if (voltage > 0.0f && current > 0.0f) {
            power_factor = active_power / (voltage * current);
            if (power_factor < 0.0f) power_factor = 0.0f;
            if (power_factor > 1.0f) power_factor = 1.0f;
        } else {
            power_factor = 0.0f;
        }
    }

    // 能量计算：结合脉冲翻转次数与寄存器低位
    uint8_t bit7 = (status >> 7) & 0x01;
    if (bit7 != s_prev_bit7) {
        s_energy_pulses++;   // 翻转加一（65536 计数周期）
        s_prev_bit7 = bit7;
    }
    uint32_t energy_cnt = s_energy_pulses * 65536u + energy_low;

    // 与原工程一致的换算（经验系数 3.006），单位可由上层自行定义
    double energy = (double)energy_cnt / 1000000000.0 * (double)p_num * 3.006 / 3600.0;

    s_last.voltage = voltage;
    s_last.current = current;
    s_last.active_power = active_power;
    s_last.power_factor = power_factor;
    s_last.energy = energy;

    s_ready = 1;
}

void hlw8032_feed_byte(uint8_t b)
{
    switch (s_idx) {
    case 0:
        s_rx[0] = b;
        if (b == 0x55 || b == 0xF2) {
            s_idx = 1;
        } else {
            s_idx = 0;
        }
        break;
    case 1:
        s_rx[1] = b;
        if (b == 0x5A) {
            s_idx = 2;
        } else {
            s_idx = 0;
        }
        break;
    case 2:
        s_rx[2] = b;
        s_checksum = b; // 校验和从 RX[2] 开始累计至 RX[22]
        s_idx = 3;
        break;
    default:
        s_rx[s_idx] = b;
        if (s_idx <= 22) {
            s_checksum = (uint8_t)(s_checksum + b);
        }
        if (s_idx == 23) {
            // 帧结束：校验
            if (b == s_checksum) {
                hlw8032_parse_frame();
            }
            s_idx = 0; // 无论成功与否均重置
        } else {
            s_idx++;
        }
        break;
    }
}

int hlw8032_frame_ready(void)
{
    return s_ready;
}

void hlw8032_get_data(HLW8032_Data* out)
{
    if (!out) return;
    *out = s_last;
}