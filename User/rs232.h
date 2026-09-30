#ifndef __RS232_H
#define __RS232_H

#include "stm32f10x.h"
#include "stdio.h"

// ==================== 柱塞泵(P1) RS232 控制 ====================
// 柱塞泵动作:抽母液(进样)/推母液(出样)
#define PUMP_ACTION_NONE    0   // 无动作（停止）
#define PUMP_ACTION_DRAW    1   // 抽母液（柱塞泵吸）
#define PUMP_ACTION_PUSH    2   // 推母液（柱塞泵排）

// 泵状态（回写 input[0x17]）
#define PUMP_STATUS_STOP  0 // 停止
#define PUMP_STATUS_RUN   1 // 运行     
#define PUMP_STATUS_FAULT 2 // 故障（硬件错/指令错/通讯丢失）
#define PUMP_BAUDRATE        9600   // 柱塞泵 RS232 波特率
#define PUMP_COMM_TIMEOUT_MS 3000   // 通讯看门狗：超此时间无反馈→故障
#define PUMP_SPEED_DEFAULT   6400   // 抽/推默认速度幅度(保持寄存器 0x16 为0或未配置时使用)

//---------！！！-- 重要说明：
//柱塞泵 RS232 接口仅用于控制 P1 
#define PUMP_BCC_CHECK    1         // 1=校验反馈BCC; 若泵反馈校验不符请改为0

void RS232_Init(uint32_t baudrate);
void RS232_Task(void);
void RS232_RxByte(uint8_t b);          // 由 USART2 中断调用

uint8_t RS232_Pump_GetStatus(void);   // 返回 PUMP_STATUS_*
uint8_t RS232_Pump_GetFault(void);    // 返回故障码(0=无)

// 柱塞泵控制接口（非阻塞，仅设置动作；实际运行由 RS232_Task 轮询执行）
void RS232_Pump_SetAction(uint8_t action, uint32_t duration_ms); // duration=0 表示持续直到显式停止
// 停止柱塞泵
void RS232_Pump_Stop(void);
// 查询柱塞泵是否正在运行
uint8_t RS232_Pump_IsRunning(void);

#endif