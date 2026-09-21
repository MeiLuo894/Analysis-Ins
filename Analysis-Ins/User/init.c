#include "init.h"
#include "io_output.h"
#include "led.h"
#include "scheduler.h"
#include "modbus_slave.h"
#include "shared_data.h"

void Init_Task(void)
{
    if (g_ctx.work_mode == WORK_MODE_INIT)
    {
        static uint32_t last_init_tick = 0;
        static uint16_t init_time = 0;
        if (g_ctx.work_mode != WORK_MODE_INIT)
        {
            return;
        }
        // 上电初始化：关闭所有输出，等待初始化完成
        ENTER_CRITICAL();
        init_time = holding_regs[REG_HOLDING_INIT_TIME];
        EXIT_CRITICAL();
        Timing_AllOff();
        g_ctx.l2_work = L2_WORK_IDLE;
        g_ctx.qc.step = L3_QC_IDLE;
        GPIO_ResetBits(LED_RUN_PORT, LED_RUN);
        GPIO_ResetBits(LED_ERR_PORT, LED_ERR);
        if (Scheduler_IsTimeout(&last_init_tick, init_time * SCALE_s_ms))
        {
            // 初始化完成（可加延时，此处直接进入空闲）
            GPIO_SetBits(LED_RUN_PORT, LED_RUN);
            GPIO_SetBits(LED_ERR_PORT, LED_ERR);
            holding_regs[REG_HOLDING_MODE] = WORK_MODE_ONLINE;
            g_ctx.work_mode = WORK_MODE_ONLINE;
            g_ctx.l1_mode = L1_MODE_IDLE;
            g_ctx.timer = Scheduler_GetTick();
        }
    }
}