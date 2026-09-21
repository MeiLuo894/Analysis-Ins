#include "eeprom.h"
#include "scheduler.h"

// 步骤超时：单个状态等待事件的最大 tick 数（约1ms/tick）
#define EEPROM_STEP_TIMEOUT_MS   100
// EEPROM 写周期最长约 5ms，探测重试次数上限（5ms × 50 = 250ms）
#define EEPROM_STANDBY_RETRY_MAX 50
// 任务总超时
#define EEPROM_JOB_TIMEOUT_MS    1000

static eeprom_job_t g_job = {0};
static uint32_t g_step_start_tick = 0;
static uint8_t  g_single_byte = 0;   // WriteByteRequest 用的静态缓冲

// 根据地址的 A16 位和读写方向计算 M24M01 器件地址
static uint8_t EEPROM_GetDevAddr(uint32_t addr, uint8_t isRead)
{
    uint8_t devAddr = EEPROM_DEV_ADDR;  // 基地址 0xA0（A16=0，写）
    if (addr & 0x10000) {               // A16 = 1
        devAddr |= 0x02;                // bit1 置1
    }
    if (isRead) {
        devAddr |= 0x01;                // bit0 = 1 表示读
    }
    return devAddr;
}

// ---------- 异步（非阻塞）接口 ----------

bool EEPROM_IsBusy(void)
{
    return (g_job.busy != 0);
}

uint8_t EEPROM_GetResult(void)
{
    uint8_t r = g_job.result;
    g_job.result = EEPROM_OK;
    return r;
}

// 启动一个任务
static void EEPROM_StartJob(uint32_t addr, uint8_t *buf, uint16_t len, uint8_t op)
{
    g_job.addr         = addr;
    g_job.buf          = buf;
    g_job.len          = len;
    g_job.offset       = 0;
    g_job.page_remain  = 0;
    g_job.op           = op;
    g_job.state        = EEPROM_STATE_START;
    g_job.busy         = 1;
    g_job.result       = EEPROM_OK;
    g_job.standby_retry= 0;
    g_step_start_tick  = Scheduler_GetTick();
}

uint8_t EEPROM_WriteRequest(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (addr > EEPROM_MAX_ADDR) return EEPROM_ERROR;
    if (len == 0) return EEPROM_OK;
    if (g_job.busy) return EEPROM_BUSY;
    EEPROM_StartJob(addr, (uint8_t*)data, len, EEPROM_OP_WRITE);
    return EEPROM_OK;
}

uint8_t EEPROM_WriteByteRequest(uint32_t addr, uint8_t data)
{
    if (addr > EEPROM_MAX_ADDR) return EEPROM_ERROR;
    if (g_job.busy) return EEPROM_BUSY;
    g_single_byte = data;
    return EEPROM_WriteRequest(addr, &g_single_byte, 1);
}

uint8_t EEPROM_ReadRequest(uint32_t addr, uint8_t *data, uint16_t len)
{
    if (addr > EEPROM_MAX_ADDR) return EEPROM_ERROR;
    if (len == 0) return EEPROM_OK;
    if (g_job.busy) return EEPROM_BUSY;
    EEPROM_StartJob(addr, data, len, EEPROM_OP_READ);
    return EEPROM_OK;
}

// ---------- 状态机驱动 ----------

void EEPROM_Poll(void)
{
    uint32_t now;
    if (!g_job.busy) return;

    // 步骤级超时检查
    now = Scheduler_GetTick();
    if ((now - g_step_start_tick) >= EEPROM_JOB_TIMEOUT_MS) {
        g_job.state  = EEPROM_STATE_ERROR;
        g_job.result = EEPROM_TIMEOUT;
        g_job.busy   = 0;   // 直接清零 busy，防止状态机永远卡住
        I2C_GenerateSTOP(I2C1, ENABLE);
        return;
    }

    switch (g_job.state)
    {
    // 发送起始信号
    case EEPROM_STATE_START:
        if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) return; // 总线仍忙
        g_step_start_tick = Scheduler_GetTick();
        if (g_job.op == EEPROM_OP_READ) {
            // 读：伪写地址
            I2C_GenerateSTART(I2C1, ENABLE);
            g_job.state = EEPROM_STATE_SEND_DEV_W;
        } else {
            // 写：直接写地址
            I2C_GenerateSTART(I2C1, ENABLE);
            g_job.state = EEPROM_STATE_SEND_DEV_W;
        }
        break;

    // 发送器件地址（写）
    case EEPROM_STATE_SEND_DEV_W:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) return;
        I2C_Send7bitAddress(I2C1, EEPROM_GetDevAddr(g_job.addr + g_job.offset, 0),
                            I2C_Direction_Transmitter);
        g_job.state = EEPROM_STATE_SEND_ADDR_HI;
        break;

    // 发送地址高字节
    case EEPROM_STATE_SEND_ADDR_HI:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
        I2C_SendData(I2C1, (uint8_t)((g_job.addr + g_job.offset) >> 8));
        g_job.state = EEPROM_STATE_SEND_ADDR_LO;
        break;

    // 发送地址低字节
    case EEPROM_STATE_SEND_ADDR_LO:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
        I2C_SendData(I2C1, (uint8_t)((g_job.addr + g_job.offset) & 0xFF));
        if (g_job.op == EEPROM_OP_READ) {
            g_job.state = EEPROM_STATE_REPEAT_START;      // 读：进入重复起始
        } else {
            g_job.state = EEPROM_STATE_SEND_DATA;         // 写：开始发数据
        }
        break;

    // ---- 写模式 ----

    // 发送数据（第一次进入本状态时发本页第一个字节）
    case EEPROM_STATE_SEND_DATA:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
        {
            uint16_t remaining   = g_job.len - g_job.offset;
            uint16_t page_remain = EEPROM_PAGE_SIZE - ((g_job.addr + g_job.offset) % EEPROM_PAGE_SIZE);
            uint16_t chunk       = (remaining < page_remain) ? remaining : page_remain;

            I2C_SendData(I2C1, g_job.buf[g_job.offset]);
            g_job.offset++;
            g_job.page_remain = chunk - 1;   // 本页剩余的待发字节数

            if (g_job.page_remain > 0) {
                g_job.state = EEPROM_STATE_SEND_DATA_NEXT;
            } else {
                g_job.state = EEPROM_STATE_STOP; // 本页已满
            }
        }
        break;

    // 连续发送本页剩余字节
    case EEPROM_STATE_SEND_DATA_NEXT:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
        I2C_SendData(I2C1, g_job.buf[g_job.offset]);
        g_job.offset++;
        g_job.page_remain--;
        if (g_job.page_remain == 0) {
            g_job.state = EEPROM_STATE_STOP;
        }
        break;

    // 发送停止信号（一页写完）
    case EEPROM_STATE_STOP:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
        I2C_GenerateSTOP(I2C1, ENABLE);
        // 等待 STOP 时序完成后再探测
        g_job.state = EEPROM_STATE_WAIT_STANDBY;
        break;

    // 等待总线释放后探测 EEPROM 就绪
    case EEPROM_STATE_WAIT_STANDBY:
        if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) return; // 等 STOP 完成
        I2C_GenerateSTART(I2C1, ENABLE);
        g_job.state = EEPROM_STATE_PROBE_ACK;
        break;

    // 探测 ACK：发送器件地址（写）看是否有 ACK
    case EEPROM_STATE_PROBE_ACK:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) return;
        I2C_Send7bitAddress(I2C1, EEPROM_DEV_ADDR, I2C_Direction_Transmitter);
        g_job.state = EEPROM_STATE_PROBE_RESULT;
        break;

    // 探测结果判断
    case EEPROM_STATE_PROBE_RESULT:
        if (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
            // 收到 ACK：EEPROM 就绪
            I2C_GenerateSTOP(I2C1, ENABLE);
            if (g_job.offset >= g_job.len) {
                // 全部写完
                g_job.state = EEPROM_STATE_DONE;
            } else {
                // 还有下一页要写
                g_job.state = EEPROM_STATE_START;
            }
        } else if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
            // 收到 NACK：EEPROM 还在内部写周期
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            I2C_GenerateSTOP(I2C1, ENABLE);
            g_job.standby_retry++;
            if (g_job.standby_retry >= EEPROM_STANDBY_RETRY_MAX) {
                g_job.state  = EEPROM_STATE_ERROR;
                g_job.result = EEPROM_TIMEOUT;
            } else {
                g_job.state = EEPROM_STATE_WAIT_STANDBY;  // 重试探测
            }
        }
        // 否则等待 ADDR 或 AF 稳定
        break;

    // ---- 读模式 ----

    // 重复起始
    case EEPROM_STATE_REPEAT_START:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return; // 伪写地址完成
        I2C_GenerateSTART(I2C1, ENABLE);
        g_job.state = EEPROM_STATE_SEND_DEV_R;
        break;

    // 发送器件地址（读）
    case EEPROM_STATE_SEND_DEV_R:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) return;
        I2C_Send7bitAddress(I2C1, EEPROM_GetDevAddr(g_job.addr + g_job.offset, 1),
                            I2C_Direction_Receiver);
        g_job.state = EEPROM_STATE_READ_DATA;
        break;

    // 读模式就绪
    case EEPROM_STATE_READ_DATA:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return;
        if ((g_job.len - g_job.offset) == 1) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);   // 最后一个字节前禁 ACK
        }
        g_job.state = EEPROM_STATE_READ_BYTE;
        break;

    // 读取一个字节
    case EEPROM_STATE_READ_BYTE:
        if (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) return;
        g_job.buf[g_job.offset] = I2C_ReceiveData(I2C1);
        g_job.offset++;
        if (g_job.offset >= g_job.len) {
            // 全部读完
            I2C_AcknowledgeConfig(I2C1, ENABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
            g_job.state = EEPROM_STATE_DONE;
            g_job.result = EEPROM_OK;
        } else {
            // 继续等下一个字节；若下一字节是最后一个，禁 ACK
            if ((g_job.len - g_job.offset) == 1) {
                I2C_AcknowledgeConfig(I2C1, DISABLE);
            }
            g_job.state = EEPROM_STATE_READ_BYTE;
        }
        break;

    // 完成
    case EEPROM_STATE_DONE:
        g_job.busy = 0;
        g_job.result = EEPROM_OK;
        break;

    // 错误
    case EEPROM_STATE_ERROR:
        g_job.busy = 0;
        // result 已在进入该状态时设置
        break;

    default:
        g_job.busy = 0;
        g_job.result = EEPROM_ERROR;
        break;
    }
}

// ---------- 同步（阻塞）接口：仅供启动阶段使用 ----------

// 等待当前异步任务完成
static uint8_t EEPROM_SyncWait(void)
{
    uint32_t timeout = Scheduler_GetTick() + EEPROM_JOB_TIMEOUT_MS;
    while (g_job.busy) {
        EEPROM_Poll();
        if (Scheduler_GetTick() > timeout) {
            g_job.busy = 0;
            g_job.result = EEPROM_TIMEOUT;
            I2C_GenerateSTOP(I2C1, ENABLE);
            return EEPROM_TIMEOUT;
        }
    }
    return g_job.result;
}

uint8_t EEPROM_WriteByte(uint32_t addr, uint8_t data)
{
    uint8_t ret = EEPROM_WriteByteRequest(addr, data);
    if (ret != EEPROM_OK) return ret;
    return EEPROM_SyncWait();
}

uint8_t EEPROM_ReadByte(uint32_t addr, uint8_t *data)
{
    uint8_t ret = EEPROM_ReadRequest(addr, data, 1);
    if (ret != EEPROM_OK) return ret;
    return EEPROM_SyncWait();
}

uint8_t EEPROM_WritePage(uint32_t addr, uint8_t *data, uint16_t len)
{
    if (len > EEPROM_PAGE_SIZE) len = EEPROM_PAGE_SIZE;
    if (len == 0) return EEPROM_OK;
    uint8_t ret = EEPROM_WriteRequest(addr, data, len);
    if (ret != EEPROM_OK) return ret;
    return EEPROM_SyncWait();
}

uint8_t EEPROM_WriteMultiPage(uint32_t addr, uint8_t *data, uint16_t len)
{
    if (len == 0) return EEPROM_OK;
    uint8_t ret = EEPROM_WriteRequest(addr, data, len);
    if (ret != EEPROM_OK) return ret;
    return EEPROM_SyncWait();
}

uint8_t EEPROM_ReadBuffer(uint32_t addr, uint8_t *data, uint16_t len)
{
    if (len == 0) return EEPROM_OK;
    uint8_t ret = EEPROM_ReadRequest(addr, data, len);
    if (ret != EEPROM_OK) return ret;
    return EEPROM_SyncWait();
}

// 等待 EEPROM 内部写完成（同步轮询 ACK，仅启动阶段使用）
uint8_t EEPROM_WaitStandby(void)
{
    uint32_t timeout = Scheduler_GetTick() + 200; // 最长 200ms
    while (1) {
        I2C_GenerateSTART(I2C1, ENABLE);
        {
            uint32_t t = Scheduler_GetTick() + EEPROM_STEP_TIMEOUT_MS;
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
                if (Scheduler_GetTick() > t) {
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    return EEPROM_TIMEOUT;
                }
            }
        }
        I2C_Send7bitAddress(I2C1, EEPROM_DEV_ADDR, I2C_Direction_Transmitter);
        {
            uint32_t t = Scheduler_GetTick() + EEPROM_STEP_TIMEOUT_MS;
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
                if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    I2C_ClearFlag(I2C1, I2C_FLAG_AF);
                    break;
                }
                if (Scheduler_GetTick() > t) {
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    return EEPROM_TIMEOUT;
                }
            }
        }
        if (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
            I2C_GenerateSTOP(I2C1, ENABLE);
            return EEPROM_OK;
        }
        if (Scheduler_GetTick() > timeout) {
            return EEPROM_TIMEOUT;
        }
    }
}