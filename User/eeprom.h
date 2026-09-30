#ifndef __EEPROM_H
#define __EEPROM_H

#include "stm32f10x.h"
#include "stdint.h"
#include <stdbool.h>

// EEPROM设备地址（M24M01）
// M24M01 器件地址格式: 1010 E2 E1 A16 R/W，bit1 为 A16 地址扩展位
#define EEPROM_DEV_ADDR     0xA0   // M24M01 基地址（A16=0）
#define EEPROM_MAX_ADDR     0x1FFFF // M24M01 最大地址（128KB，17位地址）
#define EEPROM_PAGE_SIZE    256    // M24M01 页大小（256 字节）

// 返回值
#define EEPROM_OK       0
#define EEPROM_ERROR    1
#define EEPROM_TIMEOUT  2
#define EEPROM_BUSY     3

// 操作类型
typedef enum {
    EEPROM_OP_NONE = 0,
    EEPROM_OP_WRITE,
    EEPROM_OP_READ
} eeprom_op_t;

// 状态机状态
typedef enum {
    EEPROM_STATE_IDLE = 0,
    EEPROM_STATE_START,
    EEPROM_STATE_SEND_DEV_W,
    EEPROM_STATE_SEND_ADDR_HI,
    EEPROM_STATE_SEND_ADDR_LO,
    EEPROM_STATE_SEND_DATA,
    EEPROM_STATE_SEND_DATA_NEXT,
    EEPROM_STATE_REPEAT_START,
    EEPROM_STATE_SEND_DEV_R,
    EEPROM_STATE_READ_DATA,
    EEPROM_STATE_READ_BYTE,
    EEPROM_STATE_WAIT_STANDBY,
    EEPROM_STATE_PROBE_ACK,
    EEPROM_STATE_PROBE_RESULT,
    EEPROM_STATE_STOP,
    EEPROM_STATE_DONE,
    EEPROM_STATE_ERROR
} eeprom_state_t;

// 非阻塞任务控制块
typedef struct {
    uint32_t  addr;             // 目标地址
    uint8_t  *buf;              // 写：源数据；读：目标缓冲区
    uint16_t  len;              // 总字节数
    uint16_t  offset;           // 已处理偏移
    uint16_t  page_remain;      // 当前页剩余可写字节数
    uint8_t   op;               // eeprom_op_t
    volatile uint8_t  state;    // eeprom_state_t
    volatile uint8_t  busy;     // 1=任务进行中
    volatile uint8_t  result;   // 最终结果
    volatile uint8_t  dev_addr; // 当前器件地址（含A16位）
    uint32_t  timeout_tick;     // 超时起点
    uint8_t   standby_retry;    // 等待就绪重试计数
} eeprom_job_t;

void I2C_Init_All(void);

/* ---------------- 异步（非阻塞）API ---------------- */

// 异步写多字节（跨页自动拆分），立即返回
uint8_t EEPROM_WriteRequest(uint32_t addr, const uint8_t *data, uint16_t len);

// 异步写单字节，立即返回
uint8_t EEPROM_WriteByteRequest(uint32_t addr, uint8_t data);

// 异步读多字节，立即返回
uint8_t EEPROM_ReadRequest(uint32_t addr, uint8_t *data, uint16_t len);

// 查询是否忙
bool EEPROM_IsBusy(void);

// 获取上次任务结果（成功后清零）
uint8_t EEPROM_GetResult(void);

// 状态机驱动函数（调度器周期调用，建议1ms）
void EEPROM_Poll(void);

/* ---------------- 同步（阻塞）API ---------------- */
// 仅建议在系统启动阶段（无并发任务）使用

// 写一个字节到 EEPROM（addr: 0~0x1FFFF）
uint8_t EEPROM_WriteByte(uint32_t addr, uint8_t data);

// 从 EEPROM 读一个字节
uint8_t EEPROM_ReadByte(uint32_t addr, uint8_t *data);

// 写多字节（自动跨页，不超过一页内扫描）
uint8_t EEPROM_WritePage(uint32_t addr, uint8_t *data, uint16_t len);

// 写多字节（跨页自动拆分）
uint8_t EEPROM_WriteMultiPage(uint32_t addr, uint8_t *data, uint16_t len);

// 读多字节（任意长度）
uint8_t EEPROM_ReadBuffer(uint32_t addr, uint8_t *data, uint16_t len);

// 等待 EEPROM 内部写完成（轮询 ACK）
uint8_t EEPROM_WaitStandby(void);

#endif