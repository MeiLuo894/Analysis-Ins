#include "stm32f10x.h"
#include "stdbool.h"
#include "shared_data.h"
#include "modbus_slave.h"
#include "eeprom.h"
#include "iic.h"
#include "io_input.h"
#include "io_output.h"
#include "led.h"
#include "scheduler.h"
#include "sys_config.h"
#include "usart.h"
#include "init.h"

// 共享变量定义
volatile uint16_t input_regs[INPUT_REG_COUNT] = {0};
volatile uint16_t holding_regs[HOLDING_REG_COUNT] = {0};
volatile uint8_t  g_input_stable[5] = {0};
QcContext_t g_ctx = {0};

modbus_ctx_t modbus_ctxs[MAX_MODBUS_PORTS];
// 任务包装函数（调度器需要无参函数）
static void Modbus_Task1(void) { Modbus_Task(&modbus_ctxs[0]); }
static void Modbus_Task2(void) { Modbus_Task(&modbus_ctxs[1]); }

int main(void)
{
	// 系统时钟配置（使用 HSE, 72MHz）
    SystemInit();
    // SysTick 配置 1ms 中断
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while(1);
    }
    // 使能EEPROM IIC
    I2C_Init_All();
    // 加载系统配置
    SysConfig_Load();
    // 读取配置，初始化modbus实例
    uint8_t slave_id;
    uint32_t baud;
    SysConfig_GetPort(&slave_id, &baud);
    SysConfig_GetDurationTime();
    // 初始化两个 Modbus 实例
    Modbus_Init(&modbus_ctxs[0], UART4, GPIOA, GPIO_Pin_12, 1, 9600);
    Modbus_Init(&modbus_ctxs[1], UART5, GPIOB, GPIO_Pin_5, slave_id, baud);

    // 硬件模块初始化
    IO_Output_Init();
    IO_Input_Init();
    LEDInit();
    USART1_GPIO_Init(115200);
    
    // 调度器初始化
    Scheduler_Init();
    
    // 注册任务（周期单位 ms）
    Scheduler_AddTask(Init_Task, 1);
    Scheduler_AddTask(EEPROM_Poll, 1);          // EEPROM 非阻塞状态机驱动（最高优先级）
    Scheduler_AddTask(Modbus_Task1, 10);
    Scheduler_AddTask(Modbus_Task2, 10);
    Scheduler_AddTask(SysConfig_SyncTask, 100); // 每100ms调用一次，内部判断为5秒
    Scheduler_AddTask(IO_Output_Task,   20);
    Scheduler_AddTask(IO_Input_Task,20);
    Scheduler_AddTask(LED_Task, 10);            // 实际周期由内部状态机控制
    // Scheduler_AddTask(Usart1_Task, 1000);       // 打印输出调试
    // IWDG_Init();
    // 主循环
    while (1) {
        Scheduler_Run();
        // 低功耗指令 
        // __WFI();
    }
}