#ifndef __RTC_H
#define __RTC_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

// RTC 时间结构体（内部为十进制，可直接打印/显示）
typedef struct {
    uint16_t year;    // 四位年份，如 2024
    uint8_t  month;   // 1~12
    uint8_t  day;     // 1~31
    uint8_t  hour;    // 0~23
    uint8_t  minute;  // 0~59
    uint8_t  second;  // 0~59
    uint8_t  week;    // 1~7（周一=1），可选
} rtc_time_t;

// 全局共享的当前时间（由 RTC_Task 每秒刷新一次）
extern volatile rtc_time_t g_rtc_time;

// RTC 初始化（幂等：已初始化且备份域有效时只恢复）
void RTC_Init(void);

// 读取当前时间（读硬件计数器并换算）
void RTC_GetTime(volatile rtc_time_t *t);

// 校时：写入新时间到 RTC
uint8_t RTC_SetTime(const rtc_time_t *t);

// 调度器周期性任务（建议周期 1000ms），刷新 g_rtc_time
void RTC_Task(void);

// 时间换算工具（独立实现，便于验证闰年/跨年）
uint32_t RTC_TimeToSec(const rtc_time_t *t);
void     RTC_SecToTime(uint32_t sec, rtc_time_t *t);

#endif /* __RTC_H */
