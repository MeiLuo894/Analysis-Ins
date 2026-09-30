#include "io_input.h"
#include "scheduler.h"
#include "shared_data.h"

void IO_Input_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Pin = IN1_Pin | IN2_Pin | IN3_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(IN123_Port, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = IN4_Pin | IN5_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(IN45_Port, &GPIO_InitStruct);
}

void IO_Input_Task(void)
{
    static uint8_t last_state[5] = {0};
    static uint8_t debounce_cnt[5] = {0};
    static uint32_t debounce_tick[5] = {0};
    
    static GPIO_TypeDef* ports[5] = {IN123_Port, IN123_Port, IN123_Port, IN45_Port, IN45_Port};
    static uint16_t pins[5] = {IN1_Pin, IN2_Pin, IN3_Pin, IN4_Pin, IN5_Pin};
    for (int i = 0; i < 5; i++) {
        uint8_t cur = GPIO_ReadInputDataBit(ports[i], pins[i]) ? 0 : 1;
        if (cur != last_state[i]) {
            debounce_cnt[i] = 0;
            last_state[i] = cur;
            debounce_tick[i] = Scheduler_GetTick();
        } else {
            if (debounce_cnt[i] < 5 && Scheduler_IsTimeout(&debounce_tick[i], 10)) {
                debounce_cnt[i]++;
                if (debounce_cnt[i] >= 5) {
                    ENTER_CRITICAL();
                    g_input_stable[i] = cur;
                    input_regs[REG_INPUT_YW1 + i] = cur;
                    EXIT_CRITICAL();
                }
            }
        }
    }
}