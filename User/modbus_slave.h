#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include "stm32f10x.h"

#define RX_BUF_SIZE 128
#define BAUD_LIST_SIZE (sizeof(baudrate_list) / sizeof(baudrate_list[0]))

// Modbus 实例上下文结构体
typedef struct
{
    // 硬件接口
    GPIO_TypeDef* rd_port;
    USART_TypeDef* uart;
    uint16_t rd_pin;
    uint8_t slave_id;

    // 接收环形缓冲区
    uint8_t rx_buf[RX_BUF_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t last_rx_tick;

    // 帧处理
    uint8_t frame_buf[256];
    uint8_t frame_len;
    uint8_t temp_frame[256];
    uint8_t temp_len;
    enum{
        RX_IDLE,
        RX_RECEIVING
    } rx_state;
    uint32_t idle_time;

    // 非阻塞发送（中断驱动）
    uint8_t tx_buf[256];
    uint8_t tx_len;
    volatile uint8_t tx_idx;
    volatile uint8_t tx_busy;   // 1=正在发送
} modbus_ctx_t;

// 浮点型（4字节）转十六进制（4字节）
typedef union
{
    float f;
    uint8_t bytes[4];
}FloatConverter;
void float_to_bytes(float value, uint8_t *buf, uint8_t EndianType);
void Modbus_IRQHandler(modbus_ctx_t* ctx);
void Modbus_Task(modbus_ctx_t* ctx);
void Reset_RELAY_REG(void);
void Modbus_Init(modbus_ctx_t* ctx, USART_TypeDef* uart, 
                GPIO_TypeDef* rd_port, uint16_t rd_pin, 
                uint8_t slave_id, uint32_t baudrate);
void Modbus_UpdateHardware(modbus_ctx_t* ctx, uint8_t new_addr, uint32_t new_baud);
uint32_t code_to_baud(uint16_t code);
int baud_to_code(uint32_t baud);
// void Modbus_Slave_Task(void);
// void Modbus_Init(uint32_t baudrate);
// void RS485_TxEnable(void);
// void RS485_RxEnable(void);
// void Reset_RELAY_REG(void);

#endif