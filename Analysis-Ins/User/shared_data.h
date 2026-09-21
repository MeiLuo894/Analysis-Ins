#ifndef __SHARED_DATA_H
#define __SHARED_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "modbus_slave.h"

// 模式定义
#define MODE_SINGLE     1
#define MODE_ONLINE     2

// s -> ms 进制
#define SCALE_s_ms  1000

#define ON  1
#define OFF 0

#define MAX_MODBUS_PORTS 2

// 寄存器数量
#define INPUT_REG_COUNT     64   // 最大输入寄存器数量
#define HOLDING_REG_COUNT   100  // 最大保持寄存器数量
// #define HOLDING_REG_COUNT   24  // 实际保持寄存器数量

// ==================== 五参数仪质控仪 QC 控制寄存器 ====================
// 输入寄存器地址映射
#define REG_INPUT_POWER_STATUS          0x00    // 探头电源状态
#define REG_INPUT_TEMP_STATUS           0x01    // 探头温度值状态
#define REG_INPUT_NOx_STATUS            0x02    // 探头NOx状态
#define REG_INPUT_O2_STATUS             0x03    // 探头O2状态
#define REG_INPUT_INIT_STATUS           0x04    // 上电初始化状态

#define REG_INPUT_L1_MODE               0x05    // L1_MODE 0：待机；1：运行；2：报警；3：完成
#define REG_INPUT_QC_STEP               0x06    // QC流程当前步骤(1~10)
#define REG_INPUT_STEP_REMAIN_TIME      0x07    // 当前步骤剩余时间
#define REG_INPUT_STEP_INTERVAL_TIME    0x08   // 当前步骤切换剩余时间
#define REG_INPUT_QC_ALARM              0x09    // QC报警: bit0=进水超时, bit1=原水杯无液, bit2=母液不足, bit3=纯水不足

#define REG_INPUT_YW1                   0x0A    // 流通池液位
#define REG_INPUT_YW2                   0x0B    // 五参池液位
#define REG_INPUT_YW3                   0x0C    // 低标液位
#define REG_INPUT_YW4                   0x0D    // 中标液位
#define REG_INPUT_YW5                   0x0E    // 高标液位
#define REG_INPUT_YW6                   0x0F    // 纯水桶液位
#define REG_INPUT_YW7                   0x10    // 清洗液液位
#define REG_INPUT_YW8                   0x11    // 废液桶液位

#define REG_INPUT_O2_REAL_H             0x0B    // 探头O2实时值高位
#define REG_INPUT_O2_REAL_L             0x0C    // 探头O2实时值低位
#define REG_INPUT_TEMP_REAL_H           0x0D    // 测量池温度实时值高位
#define REG_INPUT_TEMP_REAL_L           0x0E    // 测量池温度实时值低位

#define REG_INPUT_NOx_HOLD_H            0x0F    // 探头NOx保持值高位
#define REG_INPUT_NOx_HOLD_L            0x10    // 探头NOx保持值低位
#define REG_INPUT_O2_HOLD_H             0x11    // 探头O2保持值高位
#define REG_INPUT_O2_HOLD_L             0x12    // 探头O2保持值低位
#define REG_INPUT_TEMP_HOLD_H           0x13    // 测量池温度保持值高位
#define REG_INPUT_TEMP_HOLD_L           0x14    // 测量池温度保持值低位
#define REG_INPUT_NOx_CONV_H            0x15    // NOx折算浓度高位
#define REG_INPUT_NOx_CONV_L            0x16    // NOx折算浓度低位

// 保持寄存器地址映射
#define REG_HOLDING_SLAVE_ID            0x00    // 输出设备地址 1~247
#define REG_HOLDING_BAUDRATE            0x01    // 输出设备波特率
#define REG_HOLDING_DATABITS            0x02    // 输出设备数据位
#define REG_HOLDING_STOPBITS            0x03    // 输出设备停止位
#define REG_HOLDING_PARITY              0x04    // 输出设备校验位

#define REG_HOLDING_MODE                0x05    // 工作模式 1：单点；2：在线
#define REG_HOLDING_QC_TASK             0x06    // QC启动命令: bit0=测量, bit1=清洗, bit2=标定, bit3=低标核查, bit4=中标核查, bit5=高标核查, bit6=DO空气标定 (写1触发,自动清零)
#define REG_HOLDING_RESET_ALARM         0x07    // 报警复位
#define REG_HOLDING_JUMP_STEP           0x08    // 跳过当前步骤
// 单点(手动)模式输出控制寄存器
#define REG_HOLDING_P1                  0x0B    // P1 标液泵 柱塞泵
#define REG_HOLDING_P2                  0x0C    // P2 进样泵
#define REG_HOLDING_P3                  0x0D    // P3 混合泵
#define REG_HOLDING_P4                  0x0E    // P4 备用泵
#define REG_HOLDING_V1                  0x0F    // V1 进标/出标切换阀
#define REG_HOLDING_V2                  0x10    // V2 原水/纯水切换阀
#define REG_HOLDING_V3                  0x11    // V3 原水/加标切换阀
#define REG_HOLDING_V4                  0x12    // V4 排水阀
#define REG_HOLDING_V5                  0x13    // V5 备用阀

// 时长配置保持寄存器
#define REG_HOLDING_INIT_TIME           0x1E    // 上电初始化时长
#define REG_HOLDING_INTERVAL_TIME       0x1F    // 相邻step间隔时间
#define REG_HOLDING_STD_PREP_STEP1      0x20    // 标液配置step1  测量杯进一段纯水时长
#define REG_HOLDING_STD_PREP_STEP2      0x21    // 标液配置step2  柱塞泵抽母液时长
#define REG_HOLDING_STD_PREP_STEP3      0x22    // 标液配置step3  柱塞泵推母液时长（示配标浓度确定抽推次数）
#define REG_HOLDING_STD_PREP_STEP4      0x23    // 标液配置step4  测量杯进二段纯水时长
#define REG_HOLDING_STD_PREP_STEP5      0x24    // 标液配置step5  纯水母液搅拌混合时长
#define REG_HOLDING_STD_PREP_STEP6      0x25    // 标液配置step6  等待仪表测试时长
#define REG_HOLDING_STD_PREP_STEP7      0x26    // 标液配置step7  排水时长
#define REG_HOLDING_STD_PREP_STEP8      0x27    // 标液配置step8  测量杯进纯水时长
#define REG_HOLDING_STD_PREP_STEP9      0x28    // 标液配置step9  测量杯混合搅拌时长
#define REG_HOLDING_STD_PREP_STEP10     0x29    // 标液配置step10 排水（等待下一次测试）时长
#define REG_HOLDING_SPIKE_REC_STEP1     0x2A    // 加标回收step1  测量杯进一段原水时长
#define REG_HOLDING_SPIKE_REC_STEP2     0x2B    // 加标回收step2  柱塞泵抽母液
#define REG_HOLDING_SPIKE_REC_STEP3     0x2C    // 加标回收step3  柱塞泵推母液时长（示配标浓度确定抽推次数）
#define REG_HOLDING_SPIKE_REC_STEP4     0x2D    // 加标回收step4  测量杯进二段原水时长
#define REG_HOLDING_SPIKE_REC_STEP5     0x2E    // 加标回收step5  原水母液搅拌混合时长
#define REG_HOLDING_SPIKE_REC_STEP6     0x2F    // 加标回收step6  等待仪表测试时长
#define REG_HOLDING_SPIKE_REC_STEP7     0x30    // 加标回收step7  排水时长
#define REG_HOLDING_SPIKE_REC_STEP8     0x31    // 加标回收step8  测量杯进纯水时长
#define REG_HOLDING_SPIKE_REC_STEP9     0x32    // 加标回收step9  测量杯混合搅拌时长
#define REG_HOLDING_SPIKE_REC_STEP10    0x33    // 加标回收step10 排水时长（等待下一次测试）

// #define REG_HOLDING_MEASURE_VALVE   0x08    // 测量阀门状态 0：关闭；1：打开
// #define REG_HOLDING_INNER_VALVE     0x09    // 内吹阀门状态 0：关闭；1：打开
// #define REG_HOLDING_OUTER_VALVE     0x0A    // 外吹阀门状态 0：关闭；1：打开

// #define REG_HOLDING_INIT_TIME       0x0B    // 上电初始化等待时长
// #define REG_HOLDING_PURGE_CYCLE     0x0C    // 吹扫周期设置 单位：s
// #define REG_HOLDING_MEASURE_TIME    0x0D    // 测量时长 单位：s
// #define REG_HOLDING_MEASURE_INNER   0x0E    // 测量-内吹阀门切换延时 单位：s
// #define REG_HOLDING_INNER_TIME      0x0F    // 探头内吹吹扫时长设置 单位：s
// #define REG_HOLDING_INNER_OUTER     0x10    // 内吹-外吹阀门切换延时 单位：s
// #define REG_HOLDING_OUTER_TIME      0x11    // 探杆外吹吹扫时长设置 单位：s
// #define REG_HOLDING_OUTER_MEASURE   0x12    // 外吹-测量阀门切换延时 单位：s
// #define REG_HOLDING_READY_TIME      0x13    // 测量稳定延时 单位：s
// #define REG_HOLDING_O2_STD_H        0x14    // 基准氧含量高位
// #define REG_HOLDING_O2_STD_L        0x15    // 基准氧含量低位
// #define REG_HOLDING_ATM_H           0x16    // 压力高位
// #define REG_HOLDING_ATM_L           0x17    // 压力低位

// #define          0x18    
// #define REG_HOLDING_PUMP_COUNT      0x20    // 柱塞泵抽推循环次数 (配标/加标)
// #define REG_HOLDING_FILL1_TIME      0x21    // Step1 一段进水时长 单位:s
// #define REG_HOLDING_PUMP_DRAW_TIME  0x22    // Step2 柱塞泵抽母液时长 单位:s
// #define REG_HOLDING_PUMP_PUSH_TIME  0x23    // Step3 柱塞泵推母液时长 单位:s
// #define REG_HOLDING_FILL2_TIME      0x24    // Step4 二段进水时长 单位:s
// #define REG_HOLDING_MIX_TIME        0x25    // Step5 搅拌混合时长 单位:s
// #define REG_HOLDING_QC_MEASURE_TIME 0x26    // Step6 仪表测量等待时长 单位:s
// #define REG_HOLDING_DRAIN_TIME      0x27    // Step7 排水时长 单位:s
// #define REG_HOLDING_RINSE1_TIME     0x28    // Step8 润洗进水时长 单位:s
// #define REG_HOLDING_RINSE_MIX_TIME  0x29    // Step9 润洗搅拌时长 单位:s
// #define REG_HOLDING_FILL_TIMEOUT    0x2A    // 进水液位检测超时 单位:s (液位未到则报警停止)


extern volatile uint16_t input_regs[INPUT_REG_COUNT];
extern volatile uint16_t holding_regs[HOLDING_REG_COUNT];
extern volatile uint8_t g_save_result;

extern modbus_ctx_t modbus_ctxs[MAX_MODBUS_PORTS];

// 输入状态（稳定后的值）
extern volatile uint8_t g_input_stable[5];

// 保护共享数据的临界区宏
#define ENTER_CRITICAL()   __disable_irq()
#define EXIT_CRITICAL()    __enable_irq()

#endif