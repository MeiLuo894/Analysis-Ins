#include "io_output.h"
#include "scheduler.h"
#include "shared_data.h"
#include "modbus_slave.h"
#include "sys_config.h"
#include "io_input.h"
#include "rs232.h"
#include "led.h"
#include <string.h>

// 液位检查时机
#define LEVEL_CHECK_START   0   // 步骤开始时检查（前置条件）
#define LEVEL_CHECK_TIMEOUT 1   // 步骤超时后检查（结果确认）

typedef struct {
    uint16_t pin;
    GPIO_TypeDef* port;
}OutputMap_t;

static const OutputMap_t outputMap[] = {
    {Output_1_Pin,   Output_1_5_Port},
    {Output_2_Pin,   Output_1_5_Port},
    {Output_3_Pin,   Output_1_5_Port},
    {Output_4_Pin,   Output_1_5_Port},
    {Output_5_Pin,   Output_1_5_Port},
    {Output_6_Pin,   Output_6_8_Port},
    {Output_7_Pin,   Output_6_8_Port},
    {Output_8_Pin,   Output_6_8_Port}
};

// ===================== 时序表定义 ====================
// 标液配置步序 (10步)
static const TimingStep_t seq_std_prep[] = {
    {1, 0, 0, 0, 0, 0, 0, 0},  // Step1: 排水
    {0, 0, 0, 0, 0, 0, 0, 0},  // Step2: 一次进水
    {0, 0, 0, 1, 0, 0, 0, 0},  // Step3: 一次排水
    {1, 0, 0, 0, 0, 0, 0, 0},  // Step4: 二次进水
    {0, 1, 0, 0, 0, 0, 0, 0},  // Step4: 二次进水
    {0, 0, 0, 0, 0, 1, 0, 0},  // Step4: 二次进水
    {0, 0, 0, 0, 0, 0, 1, 0},  // Step4: 二次进水
    {1, 0, 0, 0, 0, 0, 0, 0},  // Step4: 二次进水
    {0, 1, 0, 0, 0, 0, 0, 0},  // Step4: 二次进水
    {0, 0, 0, 0, 0, 0, 1, 0}   // Step4: 二次进水
};

// 加标回收步序 (10步)
static const TimingStep_t seq_spike_rec[] = {
    {0, 1, 0, 0, 1, 0, 0, 0},  // Step1: 排水
    {0, 0, 0, 0, 0, 0, 0, 0},  // Step2: 进清洗液
    {0, 0, 0, 1, 0, 0, 0, 0},  // Step3: 气泡清洗
    {0, 1, 0, 0, 1, 0, 0, 0},  // Step4: 排放清洗液
    {0, 0, 1, 0, 0, 0, 0, 0},  // Step5: 进纯水
    {0, 0, 0, 0, 0, 1, 0, 0},  // Step6: 待机（等待测试）
    {0, 0, 0, 0, 0, 0, 1, 0},  // Step6: 待机（等待测试）
    {1, 0, 0, 0, 0, 0, 0, 0},  // Step6: 待机（等待测试）
    {0, 1, 0, 0, 0, 0, 0, 0},  // Step6: 待机（等待测试）
    {0, 0, 0, 0, 0, 0, 1, 0}   // Step6: 待机（等待测试）
};

// ===================== 每步持续时间表（单位：秒） ====================
// 与各时序表数组一一对应，为每一步设置泵阀打开的持续时间
uint16_t step_interval_time[1];
uint16_t step_duration_std_prep[10];
uint16_t step_duration_spike_rec[10];

// ===================== 内部函数声明 ====================
// static void Handle_InitState(void);
static void Handle_IdleState(void);
static void Handle_RunningState(void);
static void Handle_AlarmState(void);
static uint8_t YW_Monitor(void);
static void YW_Alarm_Clean(uint8_t yw, uint16_t alarm);
static void Handle_CompleteState(void);
static void Handle_SingleMode(void);
static void Handle_TimingFsm(void);
static void Update_Task_Register(void);
static void ApplyTimingStep(const TimingStep_t *step);
//static void Timing_AllOff(void);
static void Timing_EnterStep(uint8_t step);
static void Timing_Start(uint8_t cmd);
static void Timing_Stop(void);
static uint32_t Get_Step_Timeout_ms(void);
static uint16_t Check_Level_Alarm(uint8_t check_type);
static void EnterAlarmState(uint16_t alarm);
static void Update_Step_Remain_Time(void);

// ===================== 内部变量定义 ====================


// ===================== 初始化 ====================
void IO_Output_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    Timing_AllOff();

    GPIO_InitStruct.GPIO_Pin = Output_1_Pin | Output_2_Pin |
                               Output_3_Pin | Output_4_Pin |
                               Output_5_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Output_1_5_Port, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = Output_6_Pin | Output_7_Pin | Output_8_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Output_6_8_Port, &GPIO_InitStruct);

    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.work_mode = WORK_MODE_INIT;    // 上电进入初始化状态，初始化完成自动进入在线模式
    g_ctx.l1_mode = L1_MODE_IDLE;
    g_ctx.l2_work = L2_WORK_IDLE;
    g_ctx.qc.step = L3_QC_IDLE;
    g_ctx.timer = Scheduler_GetTick();
    g_ctx.current_work = 0;
}

// ===================== 输出位控制 ====================
void SetOutput(uint8_t idx, uint8_t on)
{
    if (idx < 1 || idx > 8)
    {
       return;
    }

    uint16_t pin = outputMap[idx - 1].pin;
    GPIO_TypeDef* port = outputMap[idx - 1].port;

    if (on)
        GPIO_SetBits(port, pin);
    else
        GPIO_ResetBits(port, pin);
}

// ===================== 主任务入口 ====================
void IO_Output_Task(void)
{
    uint8_t mode;   // 工作模式，单点/在线
    static uint8_t last_mode = 0xFF;

    // 等待初始化完成
    if (g_ctx.work_mode == WORK_MODE_INIT)
    {
        return;
    }
    ENTER_CRITICAL();
    mode = holding_regs[REG_HOLDING_MODE];
    EXIT_CRITICAL();

    // 检测工作模式是否发生改变
    if (mode != last_mode)
    {
        // 模式切换：停止当前流程，回到空闲
        Timing_Stop();
        g_ctx.is_initialized = 0;
        g_ctx.l1_mode = L1_MODE_IDLE;
    }
    last_mode = mode;
    if (mode == MODE_SINGLE)    // 单点手动
    {
        g_ctx.work_mode = WORK_MODE_SINGLE;
        Handle_SingleMode();
    }
    else if (mode == MODE_ONLINE)    // 在线自动
    {
        g_ctx.work_mode = WORK_MODE_ONLINE;

        // ---- 一级状态机分发 ----
        switch (g_ctx.l1_mode)
        {
            case L1_MODE_IDLE:      Handle_IdleState();     break;
            case L1_MODE_RUNNING:   Handle_RunningState();  break;
            case L1_MODE_ALARM:     Handle_AlarmState();    break;
            case L1_MODE_COMPLETE:  Handle_CompleteState(); break;
            default:
                g_ctx.l1_mode = L1_MODE_IDLE;
                break;
        }
    }
    Update_Task_Register();
    Update_Step_Remain_Time();
}

// ===================== L1: 初始化状态 ====================
// static void Handle_InitState(void)
// {
//     // 上电初始化：关闭所有输出，等待初始化完成
//     Timing_AllOff();
//     g_ctx.l2_work = L2_WORK_IDLE;
//     g_ctx.qc.step = L3_QC_IDLE;

//     // 初始化完成（可加延时，此处直接进入空闲）
//     g_ctx.l1_mode = L1_MODE_IDLE;
//     g_ctx.timer = Scheduler_GetTick();
// }

// ===================== L1: 空闲状态 ====================
static void Handle_IdleState(void)
{
    uint8_t qc_cmd; // 外部输入工作任务

    // 获取分析仪工作任务命令
    ENTER_CRITICAL();
    qc_cmd = holding_regs[REG_HOLDING_QC_TASK];
    EXIT_CRITICAL();
    // if (YW8 == 1)
    // {
    //     LED_ERR_Alarm(ON);
    //     EnterAlarmState(0x80);
    // }
    if (qc_cmd != 0)
    {
        Timing_Start(qc_cmd);
    }
    else
    {
        // 空闲：全部输出关闭
        Timing_AllOff();
    }
}

// ===================== L1: 运行状态 ====================
static void Handle_RunningState(void)
{
    // 监测任务
    ENTER_CRITICAL();
    uint8_t qc_cmd = holding_regs[REG_HOLDING_QC_TASK];
    EXIT_CRITICAL();
    if (qc_cmd == 0)
    {
        Timing_Stop();
        return;
    }
    // 运行中执行时序流程状态机
    Handle_TimingFsm();
}

// ===================== L1: 报警状态 ====================
static void Handle_AlarmState(void)
{
    uint16_t bit = 0x01;
    // 报警：停止所有输出，等待清除报警后回到空闲
    Timing_AllOff();
    // RS232_Pump_Stop();
    // 根据报警监测对应液位计
    ENTER_CRITICAL();
    uint16_t alarm = input_regs[REG_INPUT_QC_ALARM];
    EXIT_CRITICAL();
    for (uint8_t i = 0; i < 4 ; i++) {
        if (alarm & bit) {
            switch (bit)
            {
            case 0x01:
                YW_Alarm_Clean(YW1, alarm);
                break;
            case 0x02:
                YW_Alarm_Clean(YW2, alarm);
                break;
            case 0x04:
                YW_Alarm_Clean(YW3, alarm);
                break;
            case 0x08:
                YW_Alarm_Clean(YW4, alarm);
                break;
            
            default:
                break;
            }
        }
        bit <<= 1;
    }

    // 复位报警监测
    ENTER_CRITICAL();
    uint8_t reset_alarm = holding_regs[REG_HOLDING_RESET_ALARM];
    EXIT_CRITICAL();
    if (reset_alarm == 1)
    {
        reset_alarm = 0;
        ENTER_CRITICAL();
        holding_regs[REG_HOLDING_RESET_ALARM] = 0;
        input_regs[REG_INPUT_QC_ALARM] = 0;
        EXIT_CRITICAL();
        g_ctx.l1_mode = L1_MODE_IDLE;
        g_ctx.l2_work = L2_WORK_IDLE;
        g_ctx.qc.step = L3_QC_IDLE;
        LED_ERR_Alarm(OFF);
    }
}

// ===================== L1: 完成状态 ====================
static void Handle_CompleteState(void)
{
    // 流程完成：保持输出关闭，短暂停留后回到空闲
    Timing_AllOff();
    // RS232_Pump_Stop();

    // 停留 2 秒后回到空闲
    if (Scheduler_IsTimeout(&g_ctx.timer, 2000))
    {
        g_ctx.l1_mode = L1_MODE_IDLE;
        g_ctx.l2_work = L2_WORK_IDLE;
        g_ctx.qc.step = L3_QC_IDLE;
        ENTER_CRITICAL();
        input_regs[REG_INPUT_QC_STEP] = 0;
        holding_regs[REG_HOLDING_QC_TASK] = 0;
        EXIT_CRITICAL();
    }
}

/// @brief 运行过程中监测液位，当为高液位时返回1
/// @param  
/// @return 
static uint8_t YW_Monitor(void)
{
    uint8_t ret = 0;

    switch (g_ctx.l2_work)
    {
        case L2_WORK_STD_PREP:  // 标液配置步序
            switch (g_ctx.qc.step) {
                case L3_QC_STEP1:   // STEP1 测量杯进一段纯水后检查 YW1 流通池高液位
                case L3_QC_STEP4:   // STEP4 测量杯进二段纯水后检查 YW1 流通池高液位
                case L3_QC_STEP8:   // STEP8 测量杯进纯水后检查 YW1 流通池高液位
                    if (YW1 == 1) ret = 1;  // YW1已达到高液位
                    break;
                default:
                    break;
            }
            break;
        case L2_WORK_SPIKE_REC: // 加标回收步序
            switch (g_ctx.qc.step) {
                case L3_QC_STEP1:   // STEP1 测量杯进一段纯水后检查 YW1 流通池高液位
                case L3_QC_STEP4:   // STEP4 测量杯进二段纯水后检查 YW1 流通池高液位
                case L3_QC_STEP8:   // STEP8 测量杯进纯水后检查 YW1 流通池高液位
                    if (YW1 == 1) ret = 1;  // YW1已达到高液位
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
    return ret;
}

/// @brief 对应液位计为高时，清除对应报警位
/// @param yw 液位计值
static void YW_Alarm_Clean(uint8_t yw, uint16_t alarm)
{
    if (yw)
    {
        input_regs[REG_INPUT_QC_ALARM] &= alarm;
        // g_ctx.l1_mode = L1_MODE_IDLE;
        // g_ctx.l2_work = L2_WORK_IDLE;
        // g_ctx.qc.step = L3_QC_IDLE;
    }
}

// ===================== 时序流程启动 ====================
static void Timing_Start(uint8_t cmd)
{
    // 复位 QC 上下文
    memset(&g_ctx.qc, 0, sizeof(g_ctx.qc));
    g_ctx.timer = Scheduler_GetTick();
    g_ctx.during_time = 0;

    // ENTER_CRITICAL();
    // holding_regs[REG_HOLDING_QC_ENABLE] = 0;   // 命令自动清零
    // EXIT_CRITICAL();

    // 根据命令位选择流程
    if (cmd == 1)
    {
        // bit0: 标液配置步序
        g_ctx.l2_work = L2_WORK_STD_PREP;
    }
    else if (cmd == 2)
    {
        // bit1: 加标回收步序
        g_ctx.l2_work = L2_WORK_SPIKE_REC;
    }
    else
    {
        g_ctx.l2_work = L2_WORK_IDLE;
        return;
    }

    g_ctx.qc.started = 1;
    g_ctx.l1_mode = L1_MODE_RUNNING;
    Timing_EnterStep(L3_QC_STEP1);
}

// ===================== 时序流程状态机 ====================
static void Handle_TimingFsm(void)
{
    const TimingStep_t *seq = NULL;
    uint8_t step_count = 0;

    // 根据当前工作任务选择时序表
    switch (g_ctx.l2_work)
    {
        case L2_WORK_STD_PREP:  // 标液配置步序
            seq = seq_std_prep;
            step_count = sizeof(seq_std_prep) / sizeof(seq_std_prep[0]);
            break;
        case L2_WORK_SPIKE_REC: // 加标回收步序
            seq = seq_spike_rec;
            step_count = sizeof(seq_spike_rec) / sizeof(seq_spike_rec[0]);
            break;
        default:
            Timing_Stop();
            g_ctx.l1_mode = L1_MODE_IDLE;
            return;
    }

    // ==== 情况1: 切换延时中，延时结束进入下一步 ====
    if (g_ctx.qc.switching)
    {
        if (Scheduler_IsTimeout(&g_ctx.timer, step_interval_time[0] * SCALE_s_ms))
        {
            g_ctx.qc.switching = 0;
            Timing_EnterStep((uint8_t)g_ctx.qc.step + 1);
        }
        return;   // 切换期间不执行其他逻辑
    }

    // ==== 情况2: 步骤开始前置条件检查（仅进入步骤时检查一次） ====
    uint16_t alarm = Check_Level_Alarm(LEVEL_CHECK_START); 

    // ==== 情况3: 应用当前步骤的输出 ====
    if (alarm == 0 && g_ctx.qc.applied == 0 &&
        g_ctx.qc.step >= L3_QC_STEP1 && g_ctx.qc.step <= L3_QC_STEP10)
    {
        g_ctx.qc.applied = 1;
        uint8_t idx = (uint8_t)g_ctx.qc.step - 1;
        if (idx < step_count)
        {
            ApplyTimingStep(&seq[idx]);
            input_regs[REG_INPUT_QC_STEP] = idx + 1;
        }
    }

    // 跳过当前步骤
    ENTER_CRITICAL();
    uint8_t jump_step = holding_regs[REG_HOLDING_JUMP_STEP];
    EXIT_CRITICAL();
    if (jump_step && !alarm)
    {
        holding_regs[REG_HOLDING_JUMP_STEP] = 0;
        jump_step = 0;
        if (g_ctx.qc.step < (L3_QC_Step_t)step_count)
            {
                // 步骤切换：先关闭所有输出，进入切换延时
                Timing_AllOff();
                g_ctx.qc.switching = 1;
                g_ctx.timer = Scheduler_GetTick();
            }
            else
            {
                // 流程完成（先关闭所有输出）
                Timing_Stop();
                g_ctx.l1_mode = L1_MODE_COMPLETE;
                g_ctx.timer = Scheduler_GetTick();
            }
    }

    // ==== 情况4: 液位检查和步骤超时处理 ====
    if (alarm == 0 && Scheduler_IsTimeout(&g_ctx.timer, Get_Step_Timeout_ms()))
    {
        // 检查液位结果，满足则切换下一步或完成流程
        alarm = Check_Level_Alarm(LEVEL_CHECK_TIMEOUT);
        if (alarm == 0)
        {
            if (g_ctx.qc.step < (L3_QC_Step_t)step_count)
            {
                // 步骤切换：先关闭所有输出，进入切换延时
                Timing_AllOff();
                g_ctx.qc.switching = 1;
                g_ctx.timer = Scheduler_GetTick();
            }
            else
            {
                // 流程完成（先关闭所有输出）
                Timing_Stop();
                g_ctx.l1_mode = L1_MODE_COMPLETE;
                g_ctx.timer = Scheduler_GetTick();
            }
        }
    }
    else if (YW_Monitor())
    // 根据高液位停泵，切换下一流程
    {
        if (g_ctx.qc.step < (L3_QC_Step_t)step_count)
        {
            // 步骤切换：先关闭所有输出，进入切换延时
            Timing_AllOff();
            g_ctx.qc.switching = 1;
            g_ctx.timer = Scheduler_GetTick();
        }
        else
        {
            // 流程完成（进入停止时序）
            Timing_Stop();
            g_ctx.l1_mode = L1_MODE_COMPLETE;
            g_ctx.timer = Scheduler_GetTick();
        }
    }

    // ==== 情况5: 液位报警统一处理 ====
    if (alarm != 0)
    {
        LED_ERR_Alarm(ON);
        EnterAlarmState(alarm);
    }
}

// ===================== 单点（手动）模式 ====================
static void Handle_SingleMode(void)
{
    uint16_t manual_out[8];

    if (g_ctx.is_initialized == 0)
    {
        Timing_AllOff();
        // RS232_Pump_Stop();
        g_ctx.qc.precheck = 0;
        g_ctx.is_initialized = 1;
        g_ctx.l1_mode = L1_MODE_IDLE;
        g_ctx.l2_work = L2_WORK_IDLE;
        g_ctx.qc.started = 0;
        input_regs[REG_INPUT_QC_STEP] = 0;
        input_regs[REG_INPUT_QC_ALARM] = 0;
        LED_ERR_Alarm(OFF);
    }

    // 读取 8 路手动输出寄存器
    ENTER_CRITICAL();
    for (int i = 0; i < 8; i++)
    {
        manual_out[i] = holding_regs[REG_HOLDING_P2 + i];
    }
    EXIT_CRITICAL();

    for (uint8_t i = OUTPUT_IDX_P2; i <= OUTPUT_IDX_V5; i++)
    {
        SetOutput(i, (manual_out[i - 1] == 1));
    }
}

// ===================== 应用时序表步骤输出 ====================
static void ApplyTimingStep(const TimingStep_t *step)
{
    // 先关闭所有输出
    Timing_AllOff();

    // 输出
    SetOutput(OUTPUT_IDX_P2, step->p2);
    SetOutput(OUTPUT_IDX_P3, step->p3);
    SetOutput(OUTPUT_IDX_P4, step->p4);
    SetOutput(OUTPUT_IDX_V1, step->v1);
    SetOutput(OUTPUT_IDX_V2, step->v2);
    SetOutput(OUTPUT_IDX_V3, step->v3);
    SetOutput(OUTPUT_IDX_V4, step->v4);
    SetOutput(OUTPUT_IDX_V5, step->v5);

    holding_regs[REG_HOLDING_P2] = step->p2;
    holding_regs[REG_HOLDING_P3] = step->p3;
    holding_regs[REG_HOLDING_P4] = step->p4;
    holding_regs[REG_HOLDING_V1] = step->v1;
    holding_regs[REG_HOLDING_V2] = step->v2;
    holding_regs[REG_HOLDING_V3] = step->v3;
    holding_regs[REG_HOLDING_V4] = step->v4;
    holding_regs[REG_HOLDING_V5] = step->v5;
}

/// @brief 关闭全部泵阀输出
/// @param  
void Timing_AllOff(void)
{
    SetOutput(OUTPUT_IDX_P2, OFF);
    SetOutput(OUTPUT_IDX_P3, OFF);
    SetOutput(OUTPUT_IDX_P4, OFF);
    SetOutput(OUTPUT_IDX_V1, OFF);
    SetOutput(OUTPUT_IDX_V2, OFF);
    SetOutput(OUTPUT_IDX_V3, OFF);
    SetOutput(OUTPUT_IDX_V4, OFF);
    SetOutput(OUTPUT_IDX_V5, OFF);
    for (uint8_t i=0; i<8; i++)
    {
        holding_regs[REG_HOLDING_P2 + i] = 0;
    }
}

/// @brief 进入新步骤：复位计时器
/// @param step 
static void Timing_EnterStep(uint8_t step)
{
    g_ctx.qc.step = (L3_QC_Step_t)step;
    g_ctx.qc.switching = 0;      // 清除切换标记
    g_ctx.qc.start_checked = 0;  // 新步骤前置条件未检查
    g_ctx.qc.applied = 0;
    g_ctx.timer = Scheduler_GetTick();
    g_ctx.during_time = 0;
    input_regs[REG_INPUT_QC_STEP] = step;
}

// 停止时序流程
static void Timing_Stop(void)
{
    Timing_AllOff();
    g_ctx.qc.precheck = 0;
    g_ctx.qc.started = 0;
    g_ctx.qc.switching = 0;    // 清除切换标记
    g_ctx.qc.applied = 0;
    g_ctx.l2_work = L2_WORK_IDLE;
    g_ctx.qc.step = L3_QC_IDLE;
    ENTER_CRITICAL();
    input_regs[REG_INPUT_QC_STEP] = 0;
    EXIT_CRITICAL();
}

// 进入报警状态：设置报警位、停止流程、切换状态
static void EnterAlarmState(uint16_t alarm)
{
    ENTER_CRITICAL();
    holding_regs[REG_HOLDING_RESET_ALARM] = 0;  // 触发报警后，先清空复位标志，防止误触发
    input_regs[REG_INPUT_QC_ALARM] |= alarm;
    EXIT_CRITICAL();
    Timing_Stop();
    g_ctx.l1_mode = L1_MODE_ALARM;
}

// ===================== 工作任务寄存器更新 ====================
static void Update_Task_Register(void)
{
    uint8_t task_value = 0;

    switch (g_ctx.l2_work)
    {
        case L2_WORK_IDLE:         task_value = 0; break;   // 空闲
        case L2_WORK_STD_PREP:     task_value = 1; break;   // 标液配置
        case L2_WORK_SPIKE_REC:    task_value = 2; break;   // 加标回收步序
        default:                   task_value = 0; break;
    }

    g_ctx.current_work = task_value;
    ENTER_CRITICAL();
    input_regs[REG_INPUT_L1_MODE] = g_ctx.l1_mode;
    holding_regs[REG_HOLDING_QC_TASK] = task_value;
    EXIT_CRITICAL();
}

// ===================== 液位检查 ====================
// 根据当前流程和步骤检查对应的液位条件
// check_type: LEVEL_CHECK_START=步骤开始时检查, LEVEL_CHECK_TIMEOUT=步骤超时后检查
// 返回 0=正常, 非0=报警位
static uint16_t Check_Level_Alarm(uint8_t check_type)
{
    uint16_t alarm = 0;
    uint16_t error_flag = 0;
    
    switch (g_ctx.l2_work)
    {
        case L2_WORK_STD_PREP:  // 标液配置步序
            // 首先检查原系统五参数池液位计、
            // if (g_ctx.qc.precheck ==0) {
            //     g_ctx.qc.precheck = 1;
            //     if (YW2 == 0) {
            //         error_flag |= 0x02;
            //     }
            //     if (error_flag != 0) {
            //         alarm |= error_flag;
            //         break;
            //     }
            // }
            // switch (g_ctx.qc.step)
            // {
            //     case L3_QC_STEP1:
            //     case L3_QC_STEP4:
            //         if (check_type == LEVEL_CHECK_START && g_ctx.qc.start_checked == 0)
            //         {
            //             g_ctx.qc.start_checked = 1; // 标记已检查
            //             if (YW2 == 0) {
            //                 alarm |= 0x0002;  // bit1: 原水未到高液位
            //             }
            //         }
            //         break;
            //     default:
            //         break;
            // }
            if (check_type == LEVEL_CHECK_TIMEOUT && 
                (g_ctx.qc.step == L3_QC_STEP1 || g_ctx.qc.step == L3_QC_STEP4 || g_ctx.qc.step == L3_QC_STEP8))
            {
                if (YW1 == 0) alarm |= 0x0001;  // bit0: 流通池未到高液位
            }
            break;

        case L2_WORK_SPIKE_REC: // 加标回收步序
            // 首先检查一次原水
            if (g_ctx.qc.precheck ==0) {
                g_ctx.qc.precheck = 1;
                if (error_flag != 0) {
                    alarm |= error_flag;
                    break;
                }
            }
            if (check_type == LEVEL_CHECK_TIMEOUT &&
                (g_ctx.qc.step == L3_QC_STEP1 || g_ctx.qc.step == L3_QC_STEP4 || g_ctx.qc.step == L3_QC_STEP8))
            {
                if (YW1 == 0) alarm |= 0x0001;  // bit0: 流通池未到高液位
            }
            break;
        default:
            break;
    }

    return alarm;
}

// ===================== 步骤超时时间 ====================
// 每步使用独立的持续时间（来自 step_duration_xxx 表），
// 若表中无对应步骤或值为0，则回退到 REG_HOLDING_FILL_TIMEOUT（默认10秒）
static uint32_t Get_Step_Timeout_ms(void)
{
    const uint16_t *duration_table = NULL;
    uint8_t table_len = 0;
    uint8_t step_idx = 0;
    uint16_t duration = 0;

    // 根据当前工作任务选择持续时间表
    switch (g_ctx.l2_work)
    {
        case L2_WORK_STD_PREP:
            duration_table = step_duration_std_prep;
            table_len = sizeof(step_duration_std_prep) / sizeof(step_duration_std_prep[0]);
            break;
        case L2_WORK_SPIKE_REC:
            duration_table = step_duration_spike_rec;
            table_len = sizeof(step_duration_spike_rec) / sizeof(step_duration_spike_rec[0]);
            break;
        default:
            break;
    }

    // 获取当前步骤索引（Step1 对应索引0）
    if (g_ctx.qc.step >= L3_QC_STEP1 && g_ctx.qc.step <= L3_QC_STEP10)
    {
        step_idx = (uint8_t)g_ctx.qc.step - 1;
    }

    // 从表中获取本步骤持续时间
    if (duration_table != NULL && step_idx < table_len)
    {
        duration = duration_table[step_idx];
    }

    // 表中未配置或为0时，回退到寄存器配置（默认10秒）
    // if (duration == 0)
    // {
    //     duration = holding_regs[REG_HOLDING_FILL_TIMEOUT];
    //     if (duration == 0) duration = 10;
    // }

    return (uint32_t)duration * SCALE_s_ms;
}

// ===================== 当前步骤剩余时间更新 ====================
// 在 IO_Output_Task 末尾调用，每秒更新一次 REG_INPUT_STEP_REMAIN_TIME
static void Update_Step_Remain_Time(void)
{
    uint32_t elapsed_ms;
    uint32_t timeout_ms;
    uint32_t remain_ms;
    uint16_t remain_s;

    // 空闲/停止 → 清0
    if (g_ctx.l2_work == L2_WORK_IDLE || g_ctx.qc.step == L3_QC_IDLE)
    {
        ENTER_CRITICAL();
        input_regs[REG_INPUT_STEP_REMAIN_TIME] = 0;
        input_regs[REG_INPUT_STEP_INTERVAL_TIME] = 0;
        EXIT_CRITICAL();
        return;
    }

    // 在STEP切换间隔中 清0
    if (g_ctx.qc.switching) {
        // 总间隔时长（与 Handle_TimingFsm 中切换超时同一数值）
        timeout_ms = step_interval_time[0] * SCALE_s_ms;
        // 切换已过去的时间
        elapsed_ms = Scheduler_GetTick() - g_ctx.timer;
        // 剩余时间
        remain_ms = (elapsed_ms >= timeout_ms) ? 0 : (timeout_ms - elapsed_ms);
        remain_s  = (uint16_t)((remain_ms + 500) / 1000);

        ENTER_CRITICAL();
        input_regs[REG_INPUT_STEP_REMAIN_TIME] = 0;
        input_regs[REG_INPUT_STEP_INTERVAL_TIME] = remain_s;
        EXIT_CRITICAL();
        return;
    }

    // 当前步骤已过去的时间
    elapsed_ms = Scheduler_GetTick() - g_ctx.timer;

    // 当前步骤总超时（毫秒）
    timeout_ms = Get_Step_Timeout_ms();

    // 剩余时间
    remain_ms = (elapsed_ms >= timeout_ms) ? 0 : (timeout_ms - elapsed_ms);

    // 转为秒（四舍五入）
    remain_s = (uint16_t)((remain_ms + 500) / 1000);

    ENTER_CRITICAL();
    input_regs[REG_INPUT_STEP_REMAIN_TIME] = remain_s;
    input_regs[REG_INPUT_STEP_INTERVAL_TIME] = 0;
    EXIT_CRITICAL();
}
