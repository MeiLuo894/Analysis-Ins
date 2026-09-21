#include "modbus_slave.h"
#include "scheduler.h"
#include "shared_data.h"
#include "string.h"
#include "eeprom.h"
#include "sys_config.h"
#include "io_output.h"
#include "wdg.h"

// 波特率映射列表
const uint32_t baudrate_list[] = 
{
    9600,
    14400,
    19200,
    38400,
    57600,
    115200
};

// CRC16 计算
static uint16_t crc16(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

void float_to_bytes(float value, uint8_t *buf, uint8_t EndianType)
{
    FloatConverter conv;
    conv.f = value;
    if (EndianType == 0)    // 大端模式
    {
        buf[0] = conv.bytes[3];
        buf[1] = conv.bytes[2];
        buf[2] = conv.bytes[1];
        buf[3] = conv.bytes[0];
    }
    else if (EndianType == 1)    // 小端模式
    {
        buf[0] = conv.bytes[0];
        buf[1] = conv.bytes[1];
        buf[2] = conv.bytes[2];
        buf[3] = conv.bytes[3];
    }
}
// RS485 发送使能引脚
static inline void RS485_TX_ENABLE(modbus_ctx_t* ctx)
{
    GPIO_SetBits(ctx->rd_port, ctx->rd_pin);
}
// RS485 接收使能引脚
static inline void RS485_RX_ENABLE(modbus_ctx_t* ctx)
{
    GPIO_ResetBits(ctx->rd_port, ctx->rd_pin);
}

// 非阻塞发送启动：拷贝数据到发送缓冲区，开启 TXE 中断，立即返回
static void start_async_send(modbus_ctx_t* ctx, uint8_t *data, uint8_t len)
{
    // 若上一次发送尚未完成，丢弃本次（简单策略）
    // 正常 Modbus 协议不会并发发送，此处仅防御
    if (ctx->tx_busy) return;

    for (int i = 0; i < len && i < 256; i++) {
        ctx->tx_buf[i] = data[i];
    }
    ctx->tx_len = len;
    ctx->tx_idx = 0;
    ctx->tx_busy = 1;

    RS485_TX_ENABLE(ctx);   // 切到发送方向
    // 触发第一个字节，并开启 TXE 中断
    USART_SendData(ctx->uart, ctx->tx_buf[0]);
    ctx->tx_idx = 1;
    USART_ITConfig(ctx->uart, USART_IT_TXE, ENABLE);
}

// 发送响应（非阻塞：只是把数据放到发送缓冲区，立即返回）
static void send_response(modbus_ctx_t* ctx, uint8_t *data, uint8_t len)
{
    start_async_send(ctx, data, len);
}

// 异常响应
static void send_exception(modbus_ctx_t* ctx, uint8_t func_code, uint8_t exception_code)
{
    uint8_t resp[5];
    resp[0] = ctx->slave_id;
    resp[1] = func_code | 0x80;
    resp[2] = exception_code;
    uint16_t crc = crc16(resp, 3);
    resp[3] = crc & 0xFF;
    resp[4] = (crc >> 8) & 0xFF;
    send_response(ctx, resp, 5);
}

// 解析 Modbus 帧
static void parse_frame(modbus_ctx_t* ctx, uint8_t *frame, uint8_t len)
{
    if (len < 8) return; // 最小长度
    uint16_t crc_calc = crc16(frame, len - 2);
    uint16_t crc_rcv = frame[len-2] | (frame[len-1] << 8);
    if (crc_calc != crc_rcv) return;
    
    uint8_t addr = frame[0];
    uint8_t func = frame[1];
    
    if (addr != ctx->slave_id) return;
    
    switch (func) {
        case 0x03: { // 读保持寄存器
            uint16_t reg_start = (frame[2] << 8) | frame[3];
            uint16_t reg_num   = (frame[4] << 8) | frame[5];
            if (reg_num < 1 || reg_num > HOLDING_REG_COUNT) {
                send_exception(ctx, func, 0x03);
                return;
            }
            uint8_t resp_len = 3 + reg_num * 2;
            uint8_t resp[256];
            resp[0] = addr;
            resp[1] = func;
            resp[2] = reg_num * 2;
            for (int i = 0; i < reg_num; i++) {
                if (reg_start + i < HOLDING_REG_COUNT) {
                    uint16_t val = 0;
                    // EEPROM 添加内容 start
                    uint16_t reg = reg_start + i;
                    const sys_param_t* cfg = SysConfig_GetParam();
                    switch (reg)
                    {
                    case REG_HOLDING_SLAVE_ID:
                        val = cfg->slave_id;
                        break;
                    case REG_HOLDING_BAUDRATE:
                        val = cfg->baud_code;
                        break;
                    default:
                        ENTER_CRITICAL();
                        val = holding_regs[reg_start + i];
                        EXIT_CRITICAL();
                        break;
                    }
                    resp[3 + i*2] = val >> 8;
                    resp[4 + i*2] = val & 0xFF;
                } else {
                    resp[3 + i*2] = 0;
                    resp[4 + i*2] = 0;
                }
            }
            uint16_t crc = crc16(resp, resp_len);
            resp[resp_len] = crc & 0xFF;
            resp[resp_len + 1] = (crc >> 8) & 0xFF;
            send_response(ctx, resp, resp_len + 2);
            break;
        }
        case 0x06: { // 写单个保持寄存器
            uint16_t reg_addr = (frame[2] << 8) | frame[3];
            uint16_t reg_val  = (frame[4] << 8) | frame[5];
            // 在线模式下，禁止写入阀门控制寄存器
            ENTER_CRITICAL();
            uint8_t mode = holding_regs[REG_HOLDING_MODE];
            EXIT_CRITICAL();
            if (mode == MODE_ONLINE) {
                // if (reg_addr == REG_HOLDING_MEASURE_VALVE ||
                //     reg_addr == REG_HOLDING_INNER_VALVE ||
                //     reg_addr == REG_HOLDING_OUTER_VALVE ||
                if ((reg_addr >= REG_HOLDING_P1 && reg_addr <= REG_HOLDING_V5)) {
                    // 拒绝写入，直接返回（也可返回 Modbus 异常码）
                    send_exception(ctx, func, 0x03);
                    return;
                }
            }
            if (reg_addr == REG_HOLDING_SLAVE_ID)
            {
                if (reg_val < 1 || reg_val > 247)
                {
                    send_exception(ctx, func, 0x03);
                    return;
                }
                else
                {
                    // 修改影子缓存（所有端口共用）
                    const sys_param_t* cfg = SysConfig_GetParam();
                    int baud_code = cfg->baud_code;
                    SysConfig_SetPort((uint8_t)reg_val, baud_code);
                }
            }
            else if (reg_addr == REG_HOLDING_BAUDRATE)
            {
                if (reg_val > 4)
                {
                    send_exception(ctx, func, 0x03);
                    return;
                }
                else
                {
                    const sys_param_t* cfg = SysConfig_GetParam();
                    uint8_t slave_id = cfg->slave_id;
                    int baud_code = reg_val;
                    SysConfig_SetPort(slave_id, baud_code);
                }
            }
            else if (reg_addr == REG_HOLDING_DATABITS ||
                     reg_addr == REG_HOLDING_STOPBITS ||
                     reg_addr == REG_HOLDING_PARITY)
            {
                // 空实现，不支持修改数据位、停止位、校验位
            }
            else
            {
                if (reg_addr < HOLDING_REG_COUNT) {
                    ENTER_CRITICAL();
                    holding_regs[reg_addr] = reg_val;
                    SysConfig_SetDurationTime(reg_addr, reg_val);
                    EXIT_CRITICAL();
                }
            }
            // 回应帧就是请求帧本身
            send_response(ctx, frame, len);
            // Modbus_Init(9600);
            break;
        }
        case 0x10: { // 写多个保持寄存器
            uint16_t reg_start = (frame[2] << 8) | frame[3];    // 寄存器起始地址
            uint16_t reg_num   = (frame[4] << 8) | frame[5];    // 寄存器数量
            uint8_t  byte_num  = frame[6];                      // 数据字节数

            // 1. 基本参数校验
            if (reg_num == 0 || 
                reg_start + reg_num > HOLDING_REG_COUNT || 
                byte_num != 2 * reg_num ||
                len < 7 + byte_num) {
                // 参数错误，返回异常响应（可选，这里简单丢弃或返回异常码 0x03/0x04）
                send_exception(ctx, func, 0x02); // 非法数据地址
                break;
            }
            // 2. 获取当前工作模式
            ENTER_CRITICAL();
            uint8_t nox_mode = holding_regs[REG_HOLDING_MODE];
            EXIT_CRITICAL();
            // 3. 预校验所有寄存器是否合法
            uint16_t temp_values[HOLDING_REG_COUNT];
            for (int i = 0; i < reg_num; i++)
            {
                uint16_t reg_addr = reg_start + i;
                uint16_t reg_val  = (frame[7 + 2*i] << 8 | frame[8 + 2*i]);

                // 寄存器合法检查
                // 3.1 Slave ID 检查
                if (reg_addr == REG_HOLDING_SLAVE_ID)
                {
                    if (reg_val < 1 || reg_val > 247)
                    {
                        send_exception(ctx, func, 0x03);
                        return;
                    }
                }
                // 3.2 波特率检查
                else if (reg_addr == REG_HOLDING_BAUDRATE)
                {
                    if (reg_val > 4)
                    {
                        send_exception(ctx, func, 0x03);
                        return;
                    }
                }                
                // 3.3 在线模式防阀门/手动输出修改
                // else if (reg_addr == REG_HOLDING_MEASURE_VALVE ||
                //          reg_addr == REG_HOLDING_INNER_VALVE ||
                //          reg_addr == REG_HOLDING_OUTER_VALVE ||
                else if ((reg_addr >= REG_HOLDING_P1 && reg_addr <= REG_HOLDING_V5))
                {
                    if (nox_mode == MODE_ONLINE)
                    {
                        send_exception(ctx, func, 0x03);
                        return;
                    }
                }

                // 暂存写入值
                temp_values[i] = reg_val;
            }
            // 4.校验通过，进行实际写入
            ENTER_CRITICAL();
            for (uint8_t i = 0; i < reg_num; i++) 
            {
                uint16_t reg_addr = reg_start + i;
                uint16_t reg_val = temp_values[i];
                if (reg_addr == REG_HOLDING_SLAVE_ID)
                {
                    const sys_param_t* cfg = SysConfig_GetParam();
                    int baud_code = cfg->baud_code;
                    SysConfig_SetPort((uint8_t)reg_val, baud_code);
                }
                else if (reg_addr == REG_HOLDING_BAUDRATE)
                {
                    const sys_param_t* cfg = SysConfig_GetParam();
                    uint8_t slave_id = cfg->slave_id;
                    SysConfig_SetPort(slave_id, (uint8_t)reg_val);
                }
                else if (reg_addr == REG_HOLDING_DATABITS ||
                         reg_addr == REG_HOLDING_STOPBITS ||
                         reg_addr == REG_HOLDING_PARITY)
                {
                    // 空实现，不支持修改数据位、停止位、校验位
                }
                else
                {
                    holding_regs[reg_addr] = reg_val;
                    SysConfig_SetDurationTime(reg_addr, reg_val);
                }
            }
            EXIT_CRITICAL();
            // 使用10功能码波特率一起修改才生效
            // if (reg_start == REG_HOLDING_SLAVE_ID)
            // {
            //     if (byte_num == 6)
            //     {
            //         uint16_t addr = (frame[7] << 8) | frame[8];
            //         int baud_code = (frame[9] << 8) | frame[10];
            //         // 01 10 00 1E 00 03 06 00 01 00 00 25 80
            //         // 0   1  2  3  4  5  6  7  8  9  10
            //         ENTER_CRITICAL();
            //         if (addr >= 2 && addr <= 127)
            //         {
            //             // 修改影子缓存
            //             // const sys_param_t* cfg = SysConfig_GetParam();
            //             // uint32_t baud = cfg->baudrates[1];
            //             SysConfig_SetPort((uint8_t)addr, baud_code);
            //         }
            //     }
            //     else
            //     {
            //         send_exception(ctx, func, 0x10);
            //         break;
            //     }
            // }
            // else
            // {
            //     // 2. 进入临界区，逐个写入寄存器
            //     ENTER_CRITICAL();
            //     for (uint8_t i = 0; i < reg_num; i++) {
            //         uint16_t val = (frame[7 + 2*i] << 8) | frame[8 + 2*i];
            //         holding_regs[reg_start + i] = val;
            //     }
            //     EXIT_CRITICAL();
            // }
            // 3. 构造并发送响应帧（功能码、起始地址、寄存器数量）
            uint8_t resp[8]; // 响应帧长度固定 8 字节
            resp[0] = frame[0];          // 从站地址
            resp[1] = 0x10;              // 功能码
            resp[2] = (reg_start >> 8) & 0xFF;
            resp[3] = reg_start & 0xFF;
            resp[4] = (reg_num >> 8) & 0xFF;
            resp[5] = reg_num & 0xFF;
            // 添加 CRC
            uint16_t crc = crc16(resp, 6);
            resp[6] = crc & 0xFF;
            resp[7] = (crc >> 8) & 0xFF;
            send_response(ctx, resp, 8);
            break;
        }
        case 0x04: { //读输入寄存器
            uint16_t reg_start = (frame[2] << 8) | frame[3];
            uint16_t reg_num   = (frame[4] << 8) | frame[5];
            if (reg_num < 1 || reg_num > 125) {
                send_exception(ctx, func, 0x04);
                return;
            }
            uint8_t resp_len = 3 + reg_num * 2;
            uint8_t resp[256];
            resp[0] = addr;
            resp[1] = func;
            resp[2] = reg_num * 2;
            for (int i = 0; i < reg_num; i++) {
                if (reg_start + i < INPUT_REG_COUNT) {
                    uint16_t val;
                    ENTER_CRITICAL();
                    val = input_regs[reg_start + i];
                    EXIT_CRITICAL();
                    resp[3 + i*2] = val >> 8;
                    resp[4 + i*2] = val & 0xFF;
                } else {
                    resp[3 + i*2] = 0;
                    resp[4 + i*2] = 0;
                }
            }
            uint16_t crc = crc16(resp, resp_len);
            resp[resp_len] = crc & 0xFF;
            resp[resp_len + 1] = (crc >> 8) & 0xFF;
            send_response(ctx, resp, resp_len + 2);
            break;
        }
        default:
            send_exception(ctx, func, 0x01);
            break;
    }
}

// 中断处理（RXNE 接收 + TXE/TC 非阻塞发送）
void Modbus_IRQHandler(modbus_ctx_t* ctx)
{
    // ---- 接收 ----
    if (USART_GetITStatus(ctx->uart, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(ctx->uart);
        uint16_t next = (ctx->rx_head + 1) % RX_BUF_SIZE;
        if (next != ctx->rx_tail) {
            ctx->rx_buf[ctx->rx_head] = ch;
            ctx->rx_head = next;
        }
        ctx->last_rx_tick = Scheduler_GetTick();
        USART_ClearITPendingBit(ctx->uart, USART_IT_RXNE);
    }

    // ---- 发送：TXE 中断逐字节发送 ----
    if (ctx->tx_busy && USART_GetITStatus(ctx->uart, USART_IT_TXE) != RESET) {
        USART_ClearITPendingBit(ctx->uart, USART_IT_TXE);
        if (ctx->tx_idx < ctx->tx_len) {
            // 还有字节待发
            USART_SendData(ctx->uart, ctx->tx_buf[ctx->tx_idx]);
            ctx->tx_idx++;
            if (ctx->tx_idx >= ctx->tx_len) {
                // 最后一个字节已装入移位寄存器，改用 TC 中断检测发送完成
                USART_ITConfig(ctx->uart, USART_IT_TXE, DISABLE);
                USART_ClearITPendingBit(ctx->uart, USART_IT_TC);
                USART_ITConfig(ctx->uart, USART_IT_TC, ENABLE);
            }
        }
    }

    // ---- 发送完成：TC 中断关闭 485 方向 ----
    if (ctx->tx_busy && USART_GetITStatus(ctx->uart, USART_IT_TC) != RESET) {
        USART_ClearITPendingBit(ctx->uart, USART_IT_TC);
        USART_ITConfig(ctx->uart, USART_IT_TC, DISABLE);
        RS485_RX_ENABLE(ctx);           // 恢复接收方向
        ctx->tx_busy = 0;              // 标记发送完成
    }
}

// Modbus 任务处理
void Modbus_Task(modbus_ctx_t* ctx)
{
    // static enum { RX_IDLE, RX_RECEIVING } state = RX_IDLE;
    // static uint32_t idle_time;
    // static uint8_t temp_frame[256];
    // static uint8_t temp_len;
    
    uint8_t data;
    while (ctx->rx_head != ctx->rx_tail) {
        data = ctx->rx_buf[ctx->rx_tail];
        ctx->rx_tail = (ctx->rx_tail + 1) % RX_BUF_SIZE;
        switch (ctx->rx_state) {
            case RX_IDLE:
                ctx->temp_frame[0] = data;
                ctx->temp_len = 1;
                ctx->idle_time = Scheduler_GetTick();
                ctx->rx_state = RX_RECEIVING;
                break;
            case RX_RECEIVING:
                if (ctx->temp_len < sizeof(ctx->temp_frame)-1) {
                    ctx->temp_frame[ctx->temp_len++] = data;
                }
                ctx->idle_time = Scheduler_GetTick();
                break;
        }
    }
    
    if (ctx->rx_state == RX_RECEIVING) {
        uint32_t now = Scheduler_GetTick();
        if ((now - ctx->idle_time) >= 5) { // 3.5字符时间，5ms足够
            // 复制到全局 frame_buf 并解析
            memcpy(ctx->frame_buf, ctx->temp_frame, ctx->temp_len);
            ctx->frame_len = ctx->temp_len;
            parse_frame(ctx, ctx->frame_buf, ctx->frame_len);
            ctx->rx_state = RX_IDLE;
        }
    }
    // health_modbus.last_iwdg_tick = Scheduler_GetTick();
    // health_modbus.healthy = 1;
}

// void Reset_RELAY_REG(void)
// {
//     holding_regs[REG_HOLDING_MEASURE_VALVE] = 0;
//     holding_regs[REG_HOLDING_INNER_VALVE] = 0;
//     holding_regs[REG_HOLDING_OUTER_VALVE] = 0;
// }

// 硬件初始化
void Modbus_Init(modbus_ctx_t* ctx, USART_TypeDef* uart, 
                GPIO_TypeDef* rd_port, uint16_t rd_pin, 
                uint8_t slave_id, uint32_t baudrate)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->uart = uart;
    ctx->rd_port = rd_port;
    ctx->rd_pin = rd_pin;
    ctx->slave_id = slave_id;
    ctx->rx_head = 0;
    ctx->rx_tail = 0;
    ctx->rx_state = RX_IDLE;

    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    if (uart == UART4)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
        
        // TX PC10, RX PC11
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStruct);
        
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOC, &GPIO_InitStruct);

        ENTER_CRITICAL();
        holding_regs[REG_HOLDING_SLAVE_ID] = slave_id;
        holding_regs[REG_HOLDING_BAUDRATE] = baud_to_code(baudrate);
        holding_regs[REG_HOLDING_DATABITS] = 8;
        holding_regs[REG_HOLDING_STOPBITS] = 1;
        holding_regs[REG_HOLDING_PARITY] = 0;
        EXIT_CRITICAL();
    }
    else if (uart == UART5)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
        
        // TX PC12, RX PD2
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStruct);
        
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOD, &GPIO_InitStruct);

        ENTER_CRITICAL();
        holding_regs[REG_HOLDING_SLAVE_ID] = slave_id;
        holding_regs[REG_HOLDING_BAUDRATE] = baud_to_code(baudrate);
        holding_regs[REG_HOLDING_DATABITS] = 8;
        holding_regs[REG_HOLDING_STOPBITS] = 1;
        holding_regs[REG_HOLDING_PARITY] = 0;
        EXIT_CRITICAL();
    }

    // 485方向控制引脚
    uint32_t rcc_gpio = 0;
    if (rd_port == GPIOA)
        rcc_gpio = RCC_APB2Periph_GPIOA;
    else if (rd_port == GPIOB)
        rcc_gpio = RCC_APB2Periph_GPIOB;
    else if (rd_port == GPIOC)
        rcc_gpio = RCC_APB2Periph_GPIOC;
    else if (rd_port == GPIOD)
        rcc_gpio = RCC_APB2Periph_GPIOD;
    else if (rd_port == GPIOE)
        rcc_gpio = RCC_APB2Periph_GPIOE;
    RCC_APB2PeriphClockCmd(rcc_gpio, ENABLE);
    GPIO_InitStruct.GPIO_Pin = rd_pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(rd_port, &GPIO_InitStruct);
    RS485_RX_ENABLE(ctx);

    USART_DeInit(uart);
    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart, &USART_InitStruct);
    
    USART_ITConfig(uart, USART_IT_RXNE, ENABLE);
    USART_Cmd(uart, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStruct;
    if (uart == UART4)
    {
        NVIC_InitStruct.NVIC_IRQChannel = UART4_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
    }
    else if (uart == UART5)
    {
        NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
    }
    // 添加串口中断变量

    else
        return;    
    
}

// 更新 Modbus 实例的硬件配置（立即生效）
void Modbus_UpdateHardware(modbus_ctx_t* ctx, uint8_t new_addr, uint32_t new_baud)
{
    if (ctx == NULL) return;

    // 1. 更新软件地址（影响 Modbus 帧解析）
    ctx->slave_id = new_addr;
    
    // 2. 如果波特率变化，重新初始化 USART
    // 注意：这里只修改波特率，不改变其他参数（如数据位、停止位）
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = new_baud;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    // 重新初始化 USART（会暂时关闭和重新开启，但不会影响 GPIO）
    USART_Cmd(ctx->uart, DISABLE);
    USART_Init(ctx->uart, &USART_InitStruct);
    USART_Cmd(ctx->uart, ENABLE);

    ENTER_CRITICAL();
    holding_regs[REG_HOLDING_SLAVE_ID] = new_addr;
    holding_regs[REG_HOLDING_BAUDRATE] = baud_to_code(new_baud);
    holding_regs[REG_HOLDING_DATABITS] = 8;
    holding_regs[REG_HOLDING_STOPBITS] = 1;
    holding_regs[REG_HOLDING_PARITY] = 0;
    EXIT_CRITICAL();
}

/**
 * @brief 输入编号，返回波特率值 (编号 -> 值)
 * @param code 寄存器编号 (如 1)
 * @return 波特率数值 (如 9600)，无效返回 0
 */
uint32_t code_to_baud(uint16_t code) {
    if (code < BAUD_LIST_SIZE) {
        return baudrate_list[code];  // 直接数组下标取值
    }
    return 0; // 无效编号
}

/**
 * @brief 输入波特率值，返回对应编号 (值 -> 编号)
 * @param baud 波特率数值 (如 9600)
 * @return 编号 (如 1)，未找到返回 -1
 */
int baud_to_code(uint32_t baud) {
    for (uint32_t i = 0; i < BAUD_LIST_SIZE; i++) {
        if (baudrate_list[i] == baud) {
            return i;  // 返回下标即为编号
        }
    }
    return -1; // 不支持的波特率
}
