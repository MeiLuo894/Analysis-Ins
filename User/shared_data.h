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
#define REG_INPUT_QC_ALARM              0x09    // QC报警: bit0=进水超时, bit1=原水杯无液, bit2=母液不足, bit3=纯水不足，bit4=柱塞泵故障，bit5=探头通讯错，bit6=探头温度错，bit7=探头电源错

#define REG_INPUT_YW1                   0x0A    // 流通池液位
#define REG_INPUT_YW2                   0x0B    // 五参池液位
#define REG_INPUT_YW3                   0x0C    // 低标液位
#define REG_INPUT_YW4                   0x0D    // 中标液位
#define REG_INPUT_YW5                   0x0E    // 高标液位
#define REG_INPUT_YW6                   0x0F    // 纯水桶液位
#define REG_INPUT_YW7                   0x10    // 清洗液液位
#define REG_INPUT_YW8                   0x11    // 废液桶液位

#define REG_INPUT_O2_REAL_H             0x30    // 探头O2实时值高位(预留,已从0x0B移出避免与YW2撞地址)
#define REG_INPUT_O2_REAL_L             0x31    // 探头O2实时值低位(预留)
#define REG_INPUT_TEMP_REAL_H           0x32    // 测量池温度实时值高位(预留,已从0x0D移出避免与YW4撞地址)
#define REG_INPUT_TEMP_REAL_L           0x33    // 测量池温度实时值低位(预留)

#define REG_INPUT_NOx_HOLD_H            0x34    // 探头NOx保持值高位(预留)
#define REG_INPUT_NOx_HOLD_L            0x35    // 探头NOx保持值低位(预留)
#define REG_INPUT_O2_HOLD_H             0x36    // 探头O2保持值高位(预留)
#define REG_INPUT_O2_HOLD_L             0x37    // 探头O2保持值低位(预留)
#define REG_INPUT_TEMP_HOLD_H           0x38    // 测量池温度保持值高位(预留)
#define REG_INPUT_TEMP_HOLD_L           0x39    // 测量池温度保持值低位(预留)
#define REG_INPUT_NOx_CONV_H            0x3A    // NOx折算浓度高位(预留)
#define REG_INPUT_NOx_CONV_L            0x3B    // NOx折算浓度低位(预留)

//新增的rs485泵状态寄存器地址
#define REG_INPUT_PUMP1_STATUS          0x17    // 柱塞泵(P1)状态: 0停/1运行/2故障 (RS232回写)
#define REG_INPUT_PUMP1_FAULT           0x18    // 柱塞泵(P1)故障码: 0=无, 1=硬件错, 2=指令错, 0xFE=通讯丢失

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

// P1 抽推循环次数（标液配置/加标回收通用）：1~255，默认 1（= 原“抽1次推1次”）
// 柱塞泵为有限行程设备：抽满一筒母液→推出 才是一个完整循环；
// 配标/加标浓度越高所需母液越多，需重复 “抽→推” 循环 N 次（N 由此寄存器指定）
#define REG_HOLDING_P1_CYCLES           0x14    // P1 抽推循环次数（N，手动模式）
#define REG_HOLDING_P1_TARGET_CONC      0x15    // 目标浓度 mg/L（标液配置/加标回收想配出的浓度；固定母液100mg/L下自动算 N=ceil(目标浓度x满体积/(母液浓度x单次抽量))，0=用0x14手动值）
#define REG_HOLDING_P1_SPEED           0x16    // P1 抽/推速度幅度（16位；主机写入即可远程整定抽推速度；0=用默认 PUMP_SPEED_DEFAULT）

// 时长配置保持寄存器
#define REG_HOLDING_INIT_TIME           0x1E    // 上电初始化时长
#define REG_HOLDING_INTERVAL_TIME       0x1F    // 相邻step间隔时间
#define REG_HOLDING_STD_PREP_STEP1      0x20    // 标液配置step1  测量杯进一段纯水时长
#define REG_HOLDING_STD_PREP_STEP2      0x21    // 标液配置step2  柱塞泵抽母液时长
#define REG_HOLDING_STD_PREP_STEP3      0x22    // 标液配置step3  柱塞泵推母液时长(单轮推；抽推循环总次数由 REG_HOLDING_P1_CYCLES 控制)
#define REG_HOLDING_STD_PREP_STEP4      0x23    // 标液配置step4  测量杯进二段纯水时长
#define REG_HOLDING_STD_PREP_STEP5      0x24    // 标液配置step5  纯水母液搅拌混合时长
#define REG_HOLDING_STD_PREP_STEP6      0x25    // 标液配置step6  等待仪表测试时长
#define REG_HOLDING_STD_PREP_STEP7      0x26    // 标液配置step7  排水时长
#define REG_HOLDING_STD_PREP_STEP8      0x27    // 标液配置step8  测量杯进纯水时长
#define REG_HOLDING_STD_PREP_STEP9      0x28    // 标液配置step9  测量杯混合搅拌时长
#define REG_HOLDING_STD_PREP_STEP10     0x29    // 标液配置step10 排水（等待下一次测试）时长
#define REG_HOLDING_SPIKE_REC_STEP1     0x2A    // 加标回收step1  测量杯进一段原水时长
#define REG_HOLDING_SPIKE_REC_STEP2     0x2B    // 加标回收step2  柱塞泵抽母液
#define REG_HOLDING_SPIKE_REC_STEP3     0x2C    // 加标回收step3  柱塞泵推母液时长(单轮推；抽推循环总次数由 REG_HOLDING_P1_CYCLES 控制)
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

// ==================== 柱塞泵(P1) RS485 寄存器映射 ====================
// 说明：
//   - 保持寄存器：485 主机写入，用于控制 P1 柱塞泵
//   - 输入寄存器：485 主机读取，用于反馈 P1 柱塞泵状态
//   - 32 位数据统一拆成 高16位(H) + 低16位(L) 两个寄存器
//     485 主机按“高字在前”顺序读写

// ---- 保持寄存器：控制 P1 柱塞泵 ----
#define REG_HOLDING_P1_CMD        0x3E    // P1 命令(预留,已从0x0B移出避免与 REG_HOLDING_P1 抢地址)：0=无,1=停止,2=使能,3=失能,4=速度模式,5=绝对位置,6=相对位置,7=设原点,8=归零启动,9=归零停止,10=保存参数
#define REG_HOLDING_P1_POS_H      0x34    // 目标位置/相对位移 高16位（int32，单位：脉冲）
#define REG_HOLDING_P1_POS_L      0x35    // 目标位置/相对位移 低16位
#define REG_HOLDING_P1_SPD_H      0x36    // 运行速度 高16位（int32，单位：脉冲/秒，带符号，正=抽/负=推）
#define REG_HOLDING_P1_SPD_L      0x37    // 运行速度 低16位
#define REG_HOLDING_P1_ACC_H      0x38    // 加速度 高16位（int32，单位：脉冲/秒²）
#define REG_HOLDING_P1_ACC_L      0x39    // 加速度 低16位
#define REG_HOLDING_P1_DEC_H      0x3A    // 减速度 高16位（int32，单位：脉冲/秒²）
#define REG_HOLDING_P1_DEC_L      0x3B    // 减速度 低16位
#define REG_HOLDING_P1_TIMEOUT    0x3C    // 动作超时时间（ms），0 表示使用默认 3000ms
#define REG_HOLDING_P1_SAVE       0x3D    // 写 1 执行 sav\n，将参数保存到驱动器 Flash

// ---- 输入寄存器：P1 柱塞泵 富遥测（预留，待驱动回填）----
// 说明：0x17/0x18 为当前已启用的"简单状态/故障码"；本组为后续上报位置/速度/32位状态字预留，
//       整组平移到空闲区 0x20~0x27，避免与 0x18 的 REG_INPUT_PUMP1_FAULT 冲突。
#define REG_INPUT_P1_STATE        0x20    // P1 状态：0=空闲,1=忙,2=错误,3=超时
#define REG_INPUT_P1_POS_H        0x21    // 当前位置 高16位（int32 脉冲，单位：脉冲）
#define REG_INPUT_P1_POS_L        0x22    // 当前位置 低16位
#define REG_INPUT_P1_SPD_H        0x23    // 当前速度 高16位（IEEE754 float，高字在前）
#define REG_INPUT_P1_SPD_L        0x24    // 当前速度 低16位
#define REG_INPUT_P1_STATUS_H     0x25    // VSMD 状态字 高16位（32位状态位，见手册）
#define REG_INPUT_P1_STATUS_L     0x26    // VSMD 状态字 低16位
#define REG_INPUT_P1_ERR          0x27    // 错误码：0=无, 0xFE=通讯超时, 其他=驱动器故障码
#define REG_INPUT_P1_CYCLES       0x28    // 本次QC实际生效的抽推循环次数（进入Step2时由固件计算/锁存后回显）


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