#ifndef _WDG_H_
#define _WDG_H_

#include "stm32f10x.h"

typedef struct {
    uint32_t last_iwdg_tick;
    uint16_t max_interval;
    uint8_t healthy;
}TaskHealth_t;
// extern TaskHealth_t health_modbus, health_can_tx, health_can_rx;

// void WWDG_Init(void);
// void WWDG_Feed(void);
void IWDG_Init(void);
void IWDG_Feed(void);

#endif