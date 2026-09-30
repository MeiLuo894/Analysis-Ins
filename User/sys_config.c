#include "sys_config.h"
#include "eeprom.h"
#include "scheduler.h"
#include "shared_data.h"
#include "modbus_slave.h"
#include "io_output.h"
#include <string.h>
#include <stddef.h>

#define CRC_CALC_LEN     offsetof(sys_param_t, crc16)
#define BATCH_BLOCKS     4   // 每次读4个块（1024字节） 读 480/4 = 120 次
#define SCAN_LAST_BLOCKS 16  // 只读取一次最后16个块，加快读取速度
// ---------- 静态变量 ----------
static sys_param_t g_param;               // 当前生效的参数（影子缓存）
static uint8_t    g_current_block = 0;    // 当前使用的块索引（0~7）
static volatile uint8_t g_dirty = 0;      // 1=需要保存，0=已同步
static volatile uint8_t g_sync_busy = 0;  // 防止重入
volatile uint8_t g_save_result = 0;

// ---- 异步保存相关 ----
// EEPROM 异步写期间，数据缓冲区必须保持有效，
// 因此使用静态缓冲区保存待写入的一份参数拷贝。
static sys_param_t  g_async_buf;          // 异步写缓冲区（拷贝占位）
static uint8_t      g_async_next_idx = 0; // 异步写的目标块索引
static volatile uint8_t g_async_saving = 0; // 1=异步写进行中
static uint8_t      g_async_need_retry = 0; // 上次因BUSY未启动，待重试

// 静态函数声明

// ---------- 默认出厂配置 ----------
static const sys_param_t default_param = {
    .magic         = 0x5A5A,
    .seq           = 0,
    .slave_id      = 1,
    .baud_code     = 0,
    .init_time     = 5,
    .interval_t    = 2,
    .std_prep_step1  = 20,
    .std_prep_step2  = 10,
    .std_prep_step3  = 20,
    .std_prep_step4  = 10,
    .std_prep_step5  = 20,
    .std_prep_step6  = 10,
    .std_prep_step7  = 30,
    .std_prep_step8  = 20,
    .std_prep_step9  = 10,
    .std_prep_step10 = 20,
    .spike_rec_step1 = 20,
    .spike_rec_step2 = 20,
    .spike_rec_step3 = 20,
    .spike_rec_step4 = 20,
    .spike_rec_step5 = 20,
    .spike_rec_step6 = 20,
    .spike_rec_step7 = 20,
    .spike_rec_step8 = 20,
    .spike_rec_step9 = 20,
    .spike_rec_step10= 20,
    .crc16      = 0   // 由函数计算
};

// ---------- CRC16（Modbus标准） ----------
static uint16_t crc16_calc(const uint8_t* buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

static void update_crc(sys_param_t* p)
{
    // 计算从 magic 开始，到crc
    p->crc16 = 0;  // 先清零
    p->crc16 = crc16_calc((uint8_t*)p, CRC_CALC_LEN);
}

// 检查 crc 是否有效
static bool is_crc_valid(const sys_param_t* p)
{
    // uint16_t len = sizeof(sys_param_t) - sizeof(uint16_t);
    uint16_t calc = crc16_calc((uint8_t*)p, CRC_CALC_LEN);
    return (calc == p->crc16);
}

// ---------- 读写块（基于字节函数） ----------
static void read_block(uint8_t idx, sys_param_t* out)
{
    uint16_t base = CONFIG_AREA_BASE + idx * CONFIG_BLOCK_SIZE;
    // 一次批量读取整个块（256字节），失败立即返回，不逐字节重试
    if (EEPROM_ReadBuffer(base, (uint8_t*)out, sizeof(sys_param_t)) != EEPROM_OK) {
        memset(out, 0xFF, sizeof(sys_param_t));  // 失败填充0xFF
    }
}

// 写一个块字节
static uint8_t write_block(uint16_t idx, const sys_param_t* in)
{
    uint16_t base = CONFIG_AREA_BASE + idx * CONFIG_BLOCK_SIZE;
    // const uint8_t* p = (const uint8_t*)in;
    uint16_t ret;
    __disable_irq();
    // uint8_t ret = EEPROM_WritePage(base, (uint8_t*)p, sizeof(sys_param_t));
    sys_param_t local_param = *in;
    __enable_irq();
    // ret = EEPROM_WriteMultiPage(base, (uint8_t*)&local_param, sizeof(sys_param_t));
    ret = EEPROM_WritePage(base, (uint8_t*)&local_param, sizeof(sys_param_t));
    return ret;
}

// 格式化所有块（全部擦除，仅在seq溢出时调用）
static void format_all_blocks(void)
{
    sys_param_t blank = {0};
    for (uint16_t i = 0; i < CONFIG_BLOCK_COUNT; i++) {
        write_block(i, &blank);
    }
}

// ---------- 寻找最新有效块 ----------
static uint16_t find_latest_block(void)
{
    // 读取每个块前4字节，magic + seq，根据seq最大值定位块
    uint16_t found_idx = 0xFFFF;
    uint32_t max_seq = 0;
    bool     found = false;
    uint8_t header[6];
    // sys_param_t temp;
    for (uint16_t i = 0; i < CONFIG_BLOCK_COUNT; i++) {
        uint16_t addr = CONFIG_AREA_BASE + i * CONFIG_BLOCK_SIZE;
        if (EEPROM_ReadBuffer(addr, header, 6) != EEPROM_OK) {
            return 0xFFFF;
        }
        uint16_t magic = header[0] | (header[1] << 8);
        uint32_t seq   = header[2]        | 
                        (header[3] << 8)  |
                        (header[4] << 16) |
                        (header[5] << 24);
        if (magic == 0x5A5A) {
            if (!found || seq > max_seq) {
                max_seq = seq;
                found_idx = i;
                found = true;
            }
        }
    }
    return found ? found_idx : 0xFFFF;
}

// ---------- 对外接口：加载配置 ----------
bool SysConfig_Load(void)
{
    // 查找有效块
    uint16_t idx = find_latest_block();
    if (idx != 0xFFFF) {
        // 读到有效块
        read_block(idx, &g_param);
        if (is_crc_valid(&g_param)) {
            g_current_block = idx;
            g_dirty = 0;
            return true;
        }
    }
    // 未找到有效块，使用默认配置
    // format_all_blocks();
    g_param = default_param;
    update_crc(&g_param);
    g_current_block = 0;  // 将写入块0
    g_dirty = 1;          // 标记需要初始化写入
    // 立即执行一次保存，以初始化EEPROM
    SysConfig_Save();
    return true;
}

// ---------- 获取配置指针 ----------
const sys_param_t* SysConfig_GetParam(void)
{
    return &g_param;
}

// ---------- 获取端口配置（兼容旧接口） ----------
bool SysConfig_GetPort(uint8_t* addr, uint32_t* baud)
{
    *addr = g_param.slave_id;
    *baud = code_to_baud(g_param.baud_code);
    return true;
}

// ---------- 修改端口配置（只改RAM，置dirty） ----------
bool SysConfig_SetPort(uint8_t addr, int baud_code)
{
    if (addr < 1 || addr > 247) return false;
    if (baud_code < 0 || baud_code > 4) return false;
    
    // 修改影子缓存
    g_param.slave_id = addr;
    g_param.baud_code = baud_code;
    // 更新crc
    update_crc(&g_param);
    // 标记脏
    g_dirty = 1;
    // 更新modbus端口
    uint32_t baudrate = code_to_baud(baud_code);
    Modbus_UpdateHardware(&modbus_ctxs[1], addr, baudrate);
    // Modbus_UpdateHardware(&modbus_ctxs[2], addr, baudrate);
    return true;
}

// ---------- 启动异步保存 ----------
// 将参数拷贝到静态缓冲区，交给 EEPROM 异步写任务执行，函数立即返回（非阻塞）
static bool SysConfig_SaveAsync(void)
{
    if (g_sync_busy) return false;
    if (g_async_saving) return false;

    // 1. 计算下一块索引
    uint16_t next_idx = (g_current_block + 1) % CONFIG_BLOCK_COUNT;

    // 2. 更新序列号
    if (g_param.seq == 0xFFFFFFFF) {
        format_all_blocks();
        g_param.seq = 1;
        next_idx = 0;
    } else {
        g_param.seq++;
    }
    update_crc(&g_param);
    // memset(g_param.padding, 0, sizeof(g_param.padding));

    // 3. 拷贝到静态缓冲区，防止 g_param 后续被修改
    g_async_buf = g_param;
    g_async_next_idx = next_idx;
    g_sync_busy = 1;
    g_async_saving = 1;

    // 4. 提交异步写任务
    uint16_t base = CONFIG_AREA_BASE + next_idx * CONFIG_BLOCK_SIZE;
    if (EEPROM_WriteRequest(base, (uint8_t*)&g_async_buf, sizeof(sys_param_t)) == EEPROM_OK) {
        return true;
    } else {
        // 提交失败（如 EEPROM 忙），标记待重试
        g_sync_busy = 0;
        g_async_saving = 0;
        g_async_need_retry = 1;
        return false;
    }
}

// ---------- 立即保存（同步写入，仅供启动阶段使用） ----------
bool SysConfig_Save(void)
{
    if (g_sync_busy) return false;
    g_sync_busy = 1;

    // 1. 计算下一块索引
    uint16_t next_idx = (g_current_block + 1) % CONFIG_BLOCK_COUNT;
    
    // 2. 更新序列号
    if (g_param.seq == 0xFFFFFFFF) {
        format_all_blocks();
        g_param.seq = 1;
        next_idx = 0;
    } else {
        g_param.seq++;
    }
    update_crc(&g_param);
    
    // 3. 写入，并检查结果
    // 在 SysConfig_Save 中，写入前：
    // memset(g_param.padding, 0, sizeof(g_param.padding));
    uint8_t ret = write_block(next_idx, &g_param);
    if (ret == EEPROM_OK) {
        g_current_block = next_idx;
        g_dirty = 0;
        g_sync_busy = 0;
        g_save_result = 1;
        return true;
    } else {
        // 写入失败，不更新 g_current_block，保留脏标记，稍后重试
        g_sync_busy = 0;
        g_save_result = 2;
        return false;
    }
}

// ---------- 定时同步任务（供调度器调用，异步非阻塞） ----------
void SysConfig_SyncTask(void)
{
    // 1. 若前一次异步写仍在进行，则等待其完成
    if (g_async_saving) {
        // 检查 EEPROM 异步任务是否完成
        if (EEPROM_IsBusy()) return;   // 仍在写，下次再来
        // 异步写已完成，检查结果
        uint8_t res = EEPROM_GetResult();
        g_async_saving = 0;
        g_sync_busy = 0;
        if (res == EEPROM_OK) {
            g_current_block = g_async_next_idx;
            g_dirty = 0;
            g_save_result = 1;
        } else {
            // 写失败，保留脏标记，稍后重试
            g_save_result = 2;
            g_async_need_retry = 1;
        }
    }

    // 2. 如果上次因 EEPROM 忙未启动，此时重试
    if (g_async_need_retry) {
        g_async_need_retry = 0;
        SysConfig_SaveAsync();
        return;
    }

    // 3. 常规周期检查：每5秒检查一次是否需要保存
    static uint32_t last_sync_tick = 0;
    if (Scheduler_IsTimeout(&last_sync_tick, 5000)) {
        if (g_dirty && !g_sync_busy) {
            // 异步提交，立即返回，不阻塞调度器
            SysConfig_SaveAsync();
        }
    }
}

// 从 EEPROM 中获取配置时长
void SysConfig_GetDurationTime(void)
{
    ENTER_CRITICAL();
    holding_regs[REG_HOLDING_INIT_TIME] = g_param.init_time;
    holding_regs[REG_HOLDING_INTERVAL_TIME] = g_param.interval_t;
    holding_regs[REG_HOLDING_STD_PREP_STEP1] = g_param.std_prep_step1;
    holding_regs[REG_HOLDING_STD_PREP_STEP2] = g_param.std_prep_step2;
    holding_regs[REG_HOLDING_STD_PREP_STEP3] = g_param.std_prep_step3;
    holding_regs[REG_HOLDING_STD_PREP_STEP4] = g_param.std_prep_step4;
    holding_regs[REG_HOLDING_STD_PREP_STEP5] = g_param.std_prep_step5;
    holding_regs[REG_HOLDING_STD_PREP_STEP6] = g_param.std_prep_step6;
    holding_regs[REG_HOLDING_STD_PREP_STEP7] = g_param.std_prep_step7;
    holding_regs[REG_HOLDING_STD_PREP_STEP8] = g_param.std_prep_step8;
    holding_regs[REG_HOLDING_STD_PREP_STEP9] = g_param.std_prep_step9;
    holding_regs[REG_HOLDING_STD_PREP_STEP10] = g_param.std_prep_step10;

    holding_regs[REG_HOLDING_SPIKE_REC_STEP1] = g_param.spike_rec_step1;
    holding_regs[REG_HOLDING_SPIKE_REC_STEP2] = g_param.spike_rec_step2;
    holding_regs[REG_HOLDING_SPIKE_REC_STEP3] = g_param.spike_rec_step3;
    holding_regs[REG_HOLDING_SPIKE_REC_STEP4] = g_param.spike_rec_step4;
    holding_regs[REG_HOLDING_SPIKE_REC_STEP5] = g_param.spike_rec_step5;
    holding_regs[REG_HOLDING_SPIKE_REC_STEP6] = g_param.spike_rec_step6;
    holding_regs[REG_HOLDING_SPIKE_REC_STEP7] = g_param.spike_rec_step7;
    holding_regs[REG_HOLDING_SPIKE_REC_STEP8] = g_param.spike_rec_step8;
    holding_regs[REG_HOLDING_SPIKE_REC_STEP9] = g_param.spike_rec_step9;
    holding_regs[REG_HOLDING_SPIKE_REC_STEP10] = g_param.spike_rec_step10;
    EXIT_CRITICAL();
    // STEP间隔时长
    step_interval_time[0] = holding_regs[REG_HOLDING_INTERVAL_TIME];
    // 标液配置步序
    for (uint8_t i = 0; i < sizeof(step_duration_std_prep) / sizeof(step_duration_std_prep[0]); i++)
    {
        step_duration_std_prep[i] = holding_regs[REG_HOLDING_STD_PREP_STEP1 + i];
    }
    // 加标回收步序
    for (uint8_t i = 0; i < sizeof(step_duration_spike_rec) / sizeof(step_duration_spike_rec[0]); i++)
    {
        step_duration_spike_rec[i] = holding_regs[REG_HOLDING_SPIKE_REC_STEP1 + i];
    }

}
// 修改 EEPROM 中配置时长
void SysConfig_SetDurationTime(uint16_t reg, uint16_t val)
{
    uint16_t *dur = NULL;
    uint16_t *param = NULL;
    uint16_t idx = 0;

    if (reg == REG_HOLDING_INIT_TIME) {
        g_param.init_time = val;
        update_crc(&g_param);
        g_dirty = 1;
        return;
    }

    if (reg == REG_HOLDING_INTERVAL_TIME) {
        dur   = step_interval_time;
        param = &g_param.interval_t;
        idx = reg - REG_HOLDING_INTERVAL_TIME;
    }
    else if (reg >= REG_HOLDING_STD_PREP_STEP1  && reg <= REG_HOLDING_STD_PREP_STEP10) {
        dur   = step_duration_std_prep;
        param = &g_param.std_prep_step1;
        idx   = reg - REG_HOLDING_STD_PREP_STEP1;
    }
    else if (reg >= REG_HOLDING_SPIKE_REC_STEP1 && reg <= REG_HOLDING_SPIKE_REC_STEP10) {
        dur   = step_duration_spike_rec;
        param = &g_param.spike_rec_step1;
        idx   = reg - REG_HOLDING_SPIKE_REC_STEP1;
    }
    else {
        return;    // 不在时长区间内：不动任何配置，也不置 dirty
    }

    dur[idx]   = val;
    param[idx] = val;

    update_crc(&g_param);
    g_dirty = 1;
}