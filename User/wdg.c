#include "wdg.h"

// TaskHealth_t health_modbus, health_can_tx, health_can_rx;

// void WWDG_Init(void)
// {
//     // 1. 启用WWDG时钟
//     RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
    
//     // 2. 设置预分频器
//     WWDG_SetPrescaler(WWDG_Prescaler_8);  // 8分频
    
//     // 3. 设置窗口值（必须大于0x40）
//     WWDG_SetWindowValue(0x60);  // 设置窗口上限
    
//     // 4. 设置计数器初始值（必须大于窗口值且大于0x3F）
//     WWDG_Enable(0x7F);  // 启动看门狗并设置初始值
// }

// void WWDG_Feed(void)
// {
//     // 当前计数器值(WWDG->CR & 0x7F) <= 窗口值(WWDG->CFR & 0x7F) 才允许喂狗，防止程序高速运行过早喂狗
//     if ((WWDG->CR & 0x7F) <= (WWDG->CFR & 0x7F)) {
//         WWDG_SetCounter(0x7F);
//     }
// }

/* 初始化 IWDG */
void IWDG_Init(void)
{
   /* 使能 LSI */
//    RCC_LSICmd(ENABLE);
//    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

   /* 允许访问 IWDG 寄存器 */
   IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

   /* 设置预分频器 */
   IWDG_SetPrescaler(IWDG_Prescaler_64);
   // 分频频率 = 40 kHz / 64 = 625 Hz
   /* 设置重载值 */
   IWDG_SetReload(1250);
    // 超时时间 = 1250 / 625Hz = 2 s
   /* 重新装载计数器 */
   IWDG_ReloadCounter();

   /* 启动 IWDG */
   IWDG_Enable();
#ifdef DEBUG
    DBGMCU_CR_DBG_IWDG_STOP
#endif
}

void IWDG_Feed(void)
{
	IWDG_ReloadCounter();
}
