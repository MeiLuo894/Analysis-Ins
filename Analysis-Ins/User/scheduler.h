#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "stm32f10x.h"
#include <stdbool.h>

typedef void (*task_func_t)(void);

void Scheduler_Init(void);
void Scheduler_AddTask(task_func_t func, uint32_t period_ms);
void Scheduler_Run(void);
uint32_t Scheduler_GetTick(void);
bool Scheduler_IsTimeout(uint32_t *last_tick, uint32_t delay_ms);

// 在 SysTick 中断中调用
void Scheduler_Tick(void);

#endif