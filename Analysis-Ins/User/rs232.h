#ifndef __RS232_H
#define __RS232_H

#include "stm32f10x.h"
#include "stdio.h"

// ==================== 柱塞泵(P1) RS232 控制 ====================
// 柱塞泵动作:抽母液(进样)/推母液(出样)
#define PUMP_ACTION_NONE    0
#define PUMP_ACTION_DRAW    1   // 抽母液（柱塞泵吸）
#define PUMP_ACTION_PUSH    2   // 推母液（柱塞泵排）

// void RS232_Init(uint32_t boaduate);
// void RS232_Task(void);

// 柱塞泵控制接口（非阻塞，仅设置动作；实际运行由 RS232_Task 轮询执行）
// void RS232_Pump_SetAction(uint8_t action, uint32_t duration_ms);
// 停止柱塞泵
// void RS232_Pump_Stop(void);
// 查询柱塞泵是否正在运行
// uint8_t RS232_Pump_IsRunning(void);

#endif