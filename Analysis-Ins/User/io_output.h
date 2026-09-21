#ifndef __IO_OUTPUT_H
#define __IO_OUTPUT_H

#include "stm32f10x.h"

// 输出 GPIO 宏定义（对应分析仪质控仪 8 个 DO 点）
#define Output_1_5_Port     GPIOB
#define Output_6_8_Port     GPIOA

#define Output_1_Pin     GPIO_Pin_14    // Output1: P2 进样泵
#define Output_2_Pin     GPIO_Pin_13    // Output2: P3 混合泵
#define Output_3_Pin     GPIO_Pin_12    // Output3: P4 备用泵
#define Output_4_Pin     GPIO_Pin_1     // Output4: V1 进标/出标切换阀
#define Output_5_Pin     GPIO_Pin_0     // Output5: V2 原水/纯水切换阀
#define Output_6_Pin     GPIO_Pin_7     // Output6: V3 原水/加标切换阀
#define Output_7_Pin     GPIO_Pin_6     // Output7: V4 排水阀
#define Output_8_Pin     GPIO_Pin_5     // Output8: V5 备用阀

// 输出索引定义（与电气点位一一对应）
#define OUTPUT_IDX_P2           1   // Output1: P2 进样泵
#define OUTPUT_IDX_P3           2   // Output2: P3 混合泵
#define OUTPUT_IDX_P4           3   // Output3: P4 备用泵
#define OUTPUT_IDX_V1           4   // Output4: V1 进标/出标切换阀
#define OUTPUT_IDX_V2           5   // Output5: V2 原水/纯水切换阀
#define OUTPUT_IDX_V3           6   // Output6: V3 原水/加标切换阀
#define OUTPUT_IDX_V4           7   // Output7: V4 排水阀
#define OUTPUT_IDX_V5           8   // Output8: V5 备用阀

void IO_Output_Init(void);
void IO_Output_Task(void);
void SetOutput(uint8_t idx, uint8_t on);
void Timing_AllOff(void);

// ===================== 分析仪质控仪状态机定义 ========================//
// 工作模式（在线自动/单点手动）
typedef enum {
    WORK_MODE_INIT = 0,     // 上电初始化
    WORK_MODE_SINGLE = 1,   // 单点手动模式
    WORK_MODE_ONLINE = 2    // 在线自动模式
} Work_Mode_t;

// L1 设备状态（一级状态机）
typedef enum {
    L1_MODE_IDLE = 0,      // 空闲待机
    L1_MODE_RUNNING,       // 运行中（QC流程执行中）
    L1_MODE_ALARM,         // 报警状态
    L1_MODE_COMPLETE       // 流程完成
} L1_Mode_t;

// L2 工作任务（对应时序表各流程）
typedef enum {
    L2_WORK_IDLE = 0,
    L2_WORK_STD_PREP,   // 标液配置
    L2_WORK_SPIKE_REC   // 加标回收
} L2_Work_t;

// L3 QC 流程步骤（通用步骤，各流程步骤数不同）
typedef enum {
    L3_QC_IDLE = 0,
    L3_QC_STEP1,
    L3_QC_STEP2,
    L3_QC_STEP3,
    L3_QC_STEP4,
    L3_QC_STEP5,
    L3_QC_STEP6,
    L3_QC_STEP7,
    L3_QC_STEP8,
    L3_QC_STEP9,
    L3_QC_STEP10,
    L3_QC_COMPLETE
} L3_QC_Step_t;

// 时序表步骤定义
typedef struct {
    uint8_t p2;   // P2 进样泵
    uint8_t p3;   // P3 混合泵
    uint8_t p4;   // P4 备用泵
    uint8_t v1;   // V1 进标/出标切换阀
    uint8_t v2;   // V2 原水/纯水切换阀
    uint8_t v3;   // V3 原水/加标切换阀
    uint8_t v4;   // V4 排水阀
    uint8_t v5;   // V5 备用阀
} TimingStep_t;

// ==================== 上下文结构体 ====================
typedef struct {
    Work_Mode_t work_mode;      // 工作模式（0：初始化；1：单点手动；2：在线自动）
    L1_Mode_t l1_mode;          // 一级状态机（Idle/Running/Alarm/Complete）
    L2_Work_t l2_work;          // 二级工作任务
     struct {
        L3_QC_Step_t step;          // 当前 QC 步骤
        uint8_t started;            // QC 流程启动标记
        uint8_t switching;          // 1=正在切换延时中
        uint8_t precheck;           // 单个步序启动前置条件是否已检查
        uint8_t start_checked;      // 当前步骤开始前置条件是否已检查（仅检查一次）
        uint8_t applied;            // 当前步骤输出是否已应用
    } qc;
    uint32_t timer;                // 当前阶段的起始时间
    uint32_t during_time;          // 累计补偿时间（兼容保留）
    uint8_t  is_initialized;       // 单点模式进入标记
    uint8_t  current_work;         // 当前工作任务值（供外部读取）
} QcContext_t;

extern QcContext_t g_ctx;

extern uint16_t step_interval_time[1];
extern uint16_t step_duration_std_prep[10];
extern uint16_t step_duration_spike_rec[10];

// extern uint16_t step_remain_time[10];   // 每步剩余时间（秒）

#endif