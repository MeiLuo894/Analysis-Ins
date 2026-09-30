#ifndef __SYS_CONFIG_H
#define __SYS_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "shared_data.h"
#include "eeprom.h"

#define CONFIG_BLOCK_SIZE  256         // 固定块大小，必须 ≥ sizeof(sys_param_t) 64字节
#define CONFIG_BLOCK_COUNT 480         // 环形块数量（共占用 480 x 256 = 120KB）
#define CONFIG_AREA_BASE   0x01000     // 从 EEPROM 地址 0x01000 开始 必须是 page size 的整数倍
// #define CONFIG_INDEX_ADDR  0x00000     // 保存当前块索引值的EEPROM固定地址

// 参数结构体（1字节对齐，避免填充）
// #pragma pack(1)
// typedef struct {
//     uint16_t magic;        // 固定 0x5A5A
//     uint16_t seq;          // 序列号（每次写入+1）
//     uint8_t  slave_id;     // 从机地址（1~247）
//     uint32_t baudrate;     // 波特率（如 9600, 19200, 115200）
//     uint16_t reserved;
//     uint16_t crc16;        // 从 magic 到 reserved 的 CRC16
// } sys_param_t;
// #pragma pack()
// 系统配置结构体，固定32字节
#pragma pack(1)
typedef struct {
    uint16_t magic;             // 固定 0x5A5A
    uint32_t seq;               // 序列号（每次写入+1）
    uint8_t slave_id;           // 地址
    uint8_t baud_code;          // 波特率
    uint16_t init_time;         // 上电初始化时长

    uint16_t interval_t;        // 相邻step间隔时长
    uint16_t std_prep_step1;    // 标液配置step1  测量杯进一段纯水时长
    uint16_t std_prep_step2;    // 标液配置step2  柱塞泵抽母液时长
    uint16_t std_prep_step3;    // 标液配置step3  柱塞泵推母液时长（示配标浓度确定抽推次数）
    uint16_t std_prep_step4;    // 标液配置step4  测量杯进二段纯水时长
    uint16_t std_prep_step5;    // 标液配置step5  纯水母液搅拌混合时长
    uint16_t std_prep_step6;    // 标液配置step6  等待仪表测试时长
    uint16_t std_prep_step7;    // 标液配置step7  排水时长
    uint16_t std_prep_step8;    // 标液配置step8  测量杯进纯水时长
    uint16_t std_prep_step9;    // 标液配置step9  测量杯混合搅拌时长
    uint16_t std_prep_step10;   // 标液配置step10 排水（等待下一次测试）时长
    uint16_t spike_rec_step1;   // 加标回收step1  测量杯进一段原水时长
    uint16_t spike_rec_step2;   // 加标回收step2  柱塞泵抽母液
    uint16_t spike_rec_step3;   // 加标回收step3  柱塞泵推母液时长（示配标浓度确定抽推次数）
    uint16_t spike_rec_step4;   // 加标回收step4  测量杯进二段原水时长
    uint16_t spike_rec_step5;   // 加标回收step5  原水母液搅拌混合时长
    uint16_t spike_rec_step6;   // 加标回收step6  等待仪表测试时长
    uint16_t spike_rec_step7;   // 加标回收step7  排水时长
    uint16_t spike_rec_step8;   // 加标回收step8  测量杯进纯水时长
    uint16_t spike_rec_step9;   // 加标回收step9  测量杯混合搅拌时长
    uint16_t spike_rec_step10;  // 加标回收step10 排水时长（等待下一次测试）

    uint16_t crc16;        // 从 magic 到 reserved 的 CRC16
} sys_param_t;
#pragma pack()

// 对外接口：加载配置（上电调用）
bool SysConfig_Load(void);

// 对外接口：获取配置指针
const sys_param_t* SysConfig_GetParam(void);

// 对外接口：获取某个端口的配置（兼容旧接口）
bool SysConfig_GetPort(uint8_t* addr, uint32_t* baud);

// 对外接口：修改端口配置（只改RAM，置dirty）
bool SysConfig_SetPort(uint8_t addr, int baud_code);

// 对外接口：手动触发保存（立即写入EEPROM）
bool SysConfig_Save(void);

// 对外接口：定时同步任务（供调度器周期性调用，如每5秒）
void SysConfig_SyncTask(void);

// 对外接口：step时长
void SysConfig_GetDurationTime(void);
void SysConfig_SetDurationTime(uint16_t reg, uint16_t val);
#endif