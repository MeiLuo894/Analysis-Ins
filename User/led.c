#include "led.h"
#include "string.h"
#include "scheduler.h"
#include "shared_data.h"

// 初始化LED
void LEDInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
 	GPIO_SetBits(LED_RUN_PORT, LED_RUN);
 	GPIO_SetBits(LED_ERR_PORT, LED_ERR);
	GPIO_InitStructure.GPIO_Pin = LED_RUN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_RUN_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED_ERR;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_ERR_PORT, &GPIO_InitStructure);
}

void LED_ERR_Alarm(uint8_t on)
{
	if (on) GPIO_WriteBit(LED_ERR_PORT, LED_ERR, Bit_RESET);
	else 	GPIO_WriteBit(LED_ERR_PORT, LED_ERR, Bit_SET);
}

void LED_Task(void)   
{
    static uint32_t last_led_tick = 0;
    if (Scheduler_IsTimeout(&last_led_tick, 1000)) {
        GPIO_WriteBit(LED_RUN_PORT, LED_RUN, !GPIO_ReadOutputDataBit(LED_RUN_PORT, LED_RUN));
    }
}
