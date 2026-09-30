#ifndef __IO_INPUT_H
#define __IO_INPUT_H

#define IN123_Port  GPIOC
#define IN45_Port   GPIOA
#define IN1_Pin     GPIO_Pin_7
#define IN2_Pin     GPIO_Pin_8
#define IN3_Pin     GPIO_Pin_9
#define IN4_Pin     GPIO_Pin_8
#define IN5_Pin     GPIO_Pin_11

#define YW1 g_input_stable[0]    // 测量杯液位控制进样泵
#define YW2 g_input_stable[1]    // 测量杯加标回收控制进样泵
#define YW3 g_input_stable[2]    // 标液母液告警
#define YW4 g_input_stable[3]    // 纯水告警
#define YW5 g_input_stable[4]    // 备用

void IO_Input_Init(void);
void IO_Input_Task(void);

#endif