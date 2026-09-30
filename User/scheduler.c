#include "scheduler.h"

#define MAX_TASKS   32

typedef struct {
    task_func_t func;
    uint32_t period_ms;
    uint32_t last_run;
} sched_task_t;

static sched_task_t task_list[MAX_TASKS];
static uint8_t task_cnt = 0;
static volatile uint32_t sys_tick_ms = 0;

void Scheduler_Tick(void)
{
    sys_tick_ms++;
}

uint32_t Scheduler_GetTick(void)
{
    return sys_tick_ms;
}

void Scheduler_Init(void)
{
    task_cnt = 0;
}

/**
 * @brief 向调度器中添加一个周期性任务
 * 
 * 该函数将任务函数及其执行周期注册到全局任务列表中。任务的实际调度执行
 * 通常由另一个函数（如 Scheduler_Run）负责遍历 task_list，利用 Scheduler_IsTimeout
 * 判断每个任务是否达到了执行时机。
 * 
 * @param func       需要被周期性调用的任务函数指针（无参数、无返回值）
 * @param period_ms  任务的期望执行周期（单位：毫秒）
 * 
 * @note 任务数量不能超过 MAX_TASKS，否则添加失败（静默丢弃）。
 * @note last_run 初始化为 0，意味着第一次调用 Scheduler_IsTimeout 时，
 *       由于 (now - 0) >= period_ms 通常会立即成立，因此任务会首次快速执行。
 *       如果希望首次执行有固定延时，可以修改 last_run 为当前 sys_tick_ms。
 */
void Scheduler_AddTask(task_func_t func, uint32_t period_ms)
{
    // 检查任务表是否还有空位
    if (task_cnt < MAX_TASKS) {
        // 保存任务函数指针
        task_list[task_cnt].func = func;
        // 保存期望的执行周期
        task_list[task_cnt].period_ms = period_ms;
        // 上次执行时间戳初始化为 0（系统启动时刻）
        // 这样第一次调用调度判断时会立即执行一次任务
        task_list[task_cnt].last_run = 0;
        // 增加任务计数
        task_cnt++;
    }
    // 如果任务表已满，忽略添加（可以扩展返回错误码或断言）
}

/**
 * @brief 调度并执行所有到期的任务（协作式非抢占调度）
 * 
 * 该函数必须在主循环中频繁调用（通常每个循环一次）。它会一次性检查所有已注册任务，
 * 对于满足 `当前时间 - 上次运行时间 >= 周期` 的任务，立即执行其函数，并更新上次运行时间。
 * 
 * @note 所有任务在同一线程（主循环）中顺序执行，没有抢占。因此：
 *       1. 任务函数应尽快返回，不能阻塞或长时间运行，否则会影响其他任务的实时性。
 *       2. 任务函数内不应执行耗时操作（如延时、大循环、阻塞式 I/O）。
 * 
 * @note 一个循环内可能会同时执行多个到期任务，执行顺序按照添加任务的先后次序。
 * 
 * @note 时间判断使用了无符号减法回绕特性，因此即使 sys_tick_ms 发生溢出，
 *       只要任务周期小于 2^32 毫秒，就能正确判断超时。
 * 
 * @warning 该实现中，每个任务的 `last_run` 在本次循环开始时统一使用同一个 `now` 值。
 *          这意味着当多个任务在同一轮调度中执行时，后执行的任务看到的 `now` 仍然是
 *          本次循环开始时的时刻，这会造成：
 *          - 如果任务函数执行时间较长，后续任务的上次运行时间会被更新为“循环开始”的时刻，
 *            而不是“该任务实际执行完成”的时刻，可能导致该任务的周期计算略微偏差（变快）。
 *          - 对于周期较长、或者对时间精度不敏感的任务，这种偏差可以接受。
 *          - 如需更高精度，可在每个任务执行前重新获取 `now = sys_tick_ms`。
 * 
 * @see Scheduler_AddTask    注册任务
 * @see Scheduler_IsTimeout  底层超时判断函数（本函数实际上直接内联了相同的判断逻辑）
 */
void Scheduler_Run(void)
{
    // 获取当前系统 tick（所有任务共用同一个“当前时刻”，避免反复读取带来的抖动）
    uint32_t now = sys_tick_ms;

    // 遍历所有已注册的任务
    for (int i = 0; i < task_cnt; i++) {
        // 判断该任务是否到期：当前时间距离上次运行时间是否 >= 周期
        if ((now - task_list[i].last_run) >= task_list[i].period_ms) {
            // 执行任务函数（用户定义的业务逻辑）
            task_list[i].func();
            // 更新该任务的上次运行时间为当前时刻（注意：不是执行完后的时间）
            task_list[i].last_run = now;
        }
    }
}

/**
 * @brief 检查从上次记录的时刻开始，是否已经过了指定的毫秒数
 * 
 * 该函数用于实现非阻塞超时判断。它记录一个“上次时间戳”指针，每次调用时
 * 对比当前系统 tick 与上次时间戳的差值是否达到或超过 delay_ms。
 * 若超时，则自动更新时间戳为当前时刻，并返回 true；否则返回 false。
 * 
 * @note 依赖外部全局变量 sys_tick_ms，需要系统每 1ms 递增一次该变量。
 * @note 利用无符号整数减法回绕特性，即使 sys_tick_ms 溢出（回绕到 0），
 *       只要两次调用间隔不超过 2^32-1 毫秒（约 49.7 天），计算结果仍然正确。
 * 
 * @param last_tick   指向上次记录的时间戳（单位：ms）的指针，函数会读取并可能修改它
 * @param delay_ms    需要判断的超时时间（单位：ms）
 * @return true       如果从上次时间戳到当前时刻的间隔 >= delay_ms
 * @return false      如果间隔尚未达到 delay_ms
 */
bool Scheduler_IsTimeout(uint32_t *last_tick, uint32_t delay_ms)
{
    // 获取当前系统运行毫秒计数，由系统滴答定时器中断维护
    uint32_t now = sys_tick_ms;
    
    // 计算时间差：当 now 小于 *last_tick 时（发生溢出），减法仍能得到正确的差值
    if ((now - *last_tick) >= delay_ms) {
        // 超时发生：更新上次时间戳为当前时刻，以便下一次超时判断
        *last_tick = now;
        return true;
    }
    // 尚未超时
    return false;
}