#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#define LED_RUN_PORT    GPIOB
#define LED_ERR_PORT    GPIOC
#define LED_RUN         GPIO_Pin_15    // PB15
#define LED_ERR         GPIO_Pin_6     // PC6

void LEDInit(void);
void LED_ERR_Alarm(uint8_t on);
void LED_Task(void);

#endif
