#include "rtc.h"

// 基准：2000-01-01 00:00:00（配合 32 位 RTC 计数，约可到2106年）
#define RTC_EPOCH_YEAR   2000u
// BKP 备份寄存器魔术字，用于判断 RTC 是否首次配置 / 备份域是否有效
#define RTC_MAGIC        0xA5A5u

// 全局共享的当前时间，由 RTC_Task 每秒刷新
volatile rtc_time_t g_rtc_time = { 2000, 1, 1, 0, 0, 0, 6 };

// 每月的天数（平年）
static const uint8_t s_days_in_month[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

// 判断是否闰年
static bool IsLeapYear(uint16_t y)
{
    return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

// 获取某年某月的天数
static uint8_t DaysInMonth(uint16_t y, uint8_t m)
{
    if (m == 2 && IsLeapYear(y)) {
        return 29;
    }
    return s_days_in_month[m - 1];
}

// 由年月日计算星期（蔡勒公式，格里高利历），返回 1~7（周一=1）
static uint8_t CalcWeek(uint16_t y, uint8_t m, uint8_t d)
{
    uint16_t yy = y;
    uint8_t  mm = m;
    if (m < 3) {
        yy = y - 1;
        mm = m + 12;         // 1、2月视为上一年13、14月
    }
    // h = 0:周六 1:周日 2:周一 ... 6:周五
    uint16_t K = (uint16_t)(yy % 100);   // 年后两位
    uint16_t J = (uint16_t)(yy / 100);   // 世纪
    uint16_t h = (d + (13u*(mm+1u))/5u + K + K/4u + J/4u - 2u*J) % 7u;
    // 转成 周一=1 ... 周日=7
    return (uint8_t)(((h + 5u) % 7u) + 1u);
}

// 日历时间 -> 自 2000-01-01 00:00:00 以来的秒数
uint32_t RTC_TimeToSec(const rtc_time_t *t)
{
    uint32_t days = 0;
    uint16_t y;

    for (y = RTC_EPOCH_YEAR; y < t->year; y++) {
        days += IsLeapYear(y) ? 366u : 365u;
    }
    for (uint8_t m = 1; m < t->month; m++) {
        days += DaysInMonth(t->year, m);
    }
    days += (t->day - 1);

    return days * 86400u
         + (uint32_t)t->hour   * 3600u
         + (uint32_t)t->minute * 60u
         + (uint32_t)t->second;
}

// 自 2000-01-01 00:00:00 以来的秒数 -> 日历时间
void RTC_SecToTime(uint32_t sec, rtc_time_t *t)
{
    uint32_t days = sec / 86400u;
    uint32_t rem  = sec % 86400u;
    uint16_t y;

    t->second = (uint8_t)(rem % 60u);
    rem /= 60u;
    t->minute = (uint8_t)(rem % 60u);
    rem /= 60u;
    t->hour   = (uint8_t)rem;

    // 计算年月日
    y = RTC_EPOCH_YEAR;
    while (days >= (IsLeapYear(y) ? 366u : 365u)) {
        days -= IsLeapYear(y) ? 366u : 365u;
        y++;
    }
    t->year = y;

    uint8_t m = 1;
    while (days >= DaysInMonth(y, m)) {
        days -= DaysInMonth(y, m);
        m++;
    }
    t->month = m;
    t->day   = (uint8_t)(days + 1);

    t->week = CalcWeek(t->year, t->month, t->day);
}

// 手动初始化 RTC（首次上电时调用一次）
static void RTC_ConfigPeripheral(void)
{
    // 使能 PWR 和 BKP 时钟后才能访问备份域
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    // 打开备份域写保护
    PWR_BackupAccessCmd(ENABLE);

    // 使能 LSE，并等待就绪（带超时容错，避免晶振故障卡死）
    RCC_LSEConfig(RCC_LSE_ON);
    uint32_t timeout = 0x00100000u;
    while ((RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) && (timeout-- > 0)) {
        // 空等待
    }

    if (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != RESET) {
        // LSE 起振成功：选择 LSE 作为 RTC 时钟源并启用
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        RCC_RTCCLKCmd(ENABLE);

        // 等待 RTC APB1 接口同步
        RTC_WaitForSynchro();
        RTC_WaitForLastTask();

        // 分频器：32768 -> 1Hz
        RTC_SetPrescaler(32767);
        RTC_WaitForLastTask();

        // 首次上电，设置为默认时间
        rtc_time_t init = { 2024, 1, 1, 0, 0, 0, 0 };
        uint32_t sec = RTC_TimeToSec(&init);
        RTC_SetCounter(sec);
        RTC_WaitForLastTask();
    }
    // LSE 起振失败时不写计数器，避免对未启用时钟做不可靠的写操作
    // （此时 RTC 不工作，后续可通过 USART 打印告警）

    // 写魔术字，标记 RTC 已完成初始化
    BKP_WriteBackupRegister(BKP_DR1, RTC_MAGIC);
}

// RTC 初始化入口（幂等）
void RTC_Init(void)
{
    uint16_t magic;

    // 使能 PWR 和 BKP 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    // 打开备份域写保护（读 BKP 也需要）
    PWR_BackupAccessCmd(ENABLE);

    magic = BKP_ReadBackupRegister(BKP_DR1);

    if (magic != RTC_MAGIC) {
        // 首次上电 或 备份域已丢失 -> 全新配置
        RTC_ConfigPeripheral();
    } else {
        // 已初始化且备份域有效
        // 若 RTC 时钟源未使能（异常情况），重新配置；否则直接恢复
        if (RCC_GetFlagStatus(RCC_FLAG_RTCCLKSEL) == RESET) {
            RTC_ConfigPeripheral();
        } else {
            RTC_WaitForSynchro();
        }
    }
}

// 读取当前时间
void RTC_GetTime(volatile rtc_time_t *t)
{
    if (t != NULL) {
        RTC_WaitForSynchro();
        // RTC_GetCounter 为标准外设库提供的函数（位于 stm32f10x_rtc.c）
        rtc_time_t tmp;
        RTC_SecToTime(RTC_GetCounter(), &tmp);
        *t = tmp;
    }
}

// 校时：写入新时间
uint8_t RTC_SetTime(const rtc_time_t *t)
{
    if (t == NULL) return 1;

    // 基本范围校验
    if (t->year < 2000 || t->year > 2106) return 1;
    if (t->month == 0 || t->month > 12)   return 1;
    if (t->day == 0 || t->day > DaysInMonth(t->year, t->month)) return 1;
    if (t->hour   > 23) return 1;
    if (t->minute > 59) return 1;
    if (t->second > 59) return 1;

    uint32_t sec = RTC_TimeToSec(t);

    // 备份域写保护（校时时确保可写）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    RTC_WaitForLastTask();
    RTC_SetCounter(sec);
    RTC_WaitForLastTask();

    return 0;
}

// 调度器任务：每秒刷新全局时间
void RTC_Task(void)
{
    RTC_GetTime(&g_rtc_time);
}
