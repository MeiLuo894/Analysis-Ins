#include "rs232.h"
#include "shared_data.h"
#include "scheduler.h"

// ==================== 柱塞泵(P1) RS232 控制 ====================
// 柱塞泵通过 RS232(USART2) 控制，协议需按实际泵型号填写
// 此处实现非阻塞控制框架：设置动作+时长，RS232_Task 轮询计时，超时自动停止
static volatile uint8_t  g_pump_action = PUMP_ACTION_NONE;
static volatile uint32_t g_pump_start_tick = 0;
static volatile uint32_t g_pump_duration_ms = 0;
static volatile uint8_t  g_pump_running = 0;

void RS232_Init(uint32_t boaduate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	// NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    // 2. 配置USART2 TX引脚(PA2)为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 3. 配置USART2 Rx (PA3)为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    USART_InitStructure.USART_BaudRate = boaduate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
    USART_Init(USART2, &USART_InitStructure);	
    USART_Cmd(USART2, ENABLE);
}

// 设置柱塞泵动作（非阻塞）
void RS232_Pump_SetAction(uint8_t action, uint32_t duration_ms)
{
    g_pump_action = action;
    g_pump_duration_ms = duration_ms;
    g_pump_start_tick = Scheduler_GetTick();
    g_pump_running = 1;

    // TODO: 根据实际柱塞泵 RS232 协议发送启动指令
    // 示例（需替换为实际协议帧）：
    // uint8_t cmd[8];
    // if (action == PUMP_ACTION_DRAW) {
    //     // 发送"抽"指令
    // } else if (action == PUMP_ACTION_PUSH) {
    //     // 发送"推"指令
    // }
    // for (int i = 0; i < len; i++) {
    //     while (!(USART_GetFlagStatus(USART2, USART_FLAG_TXE)));
    //     USART_SendData(USART2, cmd[i]);
    // }
}

// 停止柱塞泵
void RS232_Pump_Stop(void)
{
    g_pump_action = PUMP_ACTION_NONE;
    g_pump_running = 0;

    // TODO: 根据实际柱塞泵 RS232 协议发送停止指令
}

// 查询柱塞泵是否正在运行
uint8_t RS232_Pump_IsRunning(void)
{
    return g_pump_running;
}

void RS232_Task(void)
{
    // 非阻塞计时：运行时长达到后自动停止
    if (g_pump_running) {
        if (Scheduler_IsTimeout(&g_pump_start_tick, g_pump_duration_ms)) {
            RS232_Pump_Stop();
        }
    }
}