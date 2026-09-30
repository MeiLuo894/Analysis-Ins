#include "rs232.h"
#include "shared_data.h"
#include "scheduler.h"
#include "stm32f10x.h"

// ==================== 柱塞泵(P1) RS232 控制 (VSMD 协议) ====================
// 协议要点（依据 VSMD系列协议篇 / VSMD102 使用指南手册）：
//  - RS232 指令以 '\n'(0x0A) 结尾；RS232 不带设备号前缀（仅 RS485 才加 CID）
//  - 使能: "ena\n"   失能: "off\n"
//  - 速度模式运行: "mov spd=±值\n"  —— 方向由速度符号决定(正/负)
//  - 停止: "stp\n"   读状态: "sts\n"
//  - 反馈帧: FF [dev=0] [fb#] [data] [BH][BL] FE
//      fb#=2 时 data = 速度(5B) 位置(5B) 状态字(5B)，多字节按 7-bit 重组：
//      v = b0 | b1<<7 | b2<<14 | b3<<21 | b4<<28
//      状态字: bit6=硬件错误(flt) bit8=运行(0)/停止(1) bit9=指令错误 bit13=使能(pwr)
//  - BCC = 异或(dev,fb#,data...)，接收端 (BH<<7)|BL 还原

// ---- 内部状态 ----
static volatile uint8_t  g_pump_action    = PUMP_ACTION_NONE;   // 当前动作(抽/推/停止)
static volatile uint8_t  g_pump_running   = 0;                  // 0=停止,1=运行中(抽/推)
static volatile uint8_t  g_pump_status    = PUMP_STATUS_STOP;   // 0=停止,1=运行,2=故障
static volatile uint8_t  g_pump_fault     = 0;      // 0=无故障
static volatile uint8_t  g_pump_enabled   = 0;      // 是否已发送 ena使能
static volatile uint32_t g_pump_start_tick = 0;     // 柱塞泵动作开始时刻(用于超时检测)
static volatile uint32_t g_pump_duration_ms = 0;    // 柱塞泵动作持续时长(ms)，0=持续到显式停止
static volatile uint32_t g_last_rx_tick   = 0;      // 最近收到有效帧的时刻
static volatile uint8_t  g_rs232_inited   = 0;       // USART2 是否已初始化
static volatile uint16_t g_pump_speed_cache = PUMP_SPEED_DEFAULT; // 当前生效速度幅度(与 0x16 同步)

// 富遥测(由 sts 帧解析，任务级写回输入寄存器 0x20~0x27)
static volatile int32_t  g_pump_pos       = 0;   // 当前位置(脉冲, int32, 符号数)
static volatile float    g_pump_spd       = 0.0f;// 当前速度(IEEE754 float)
static volatile uint32_t g_pump_statword  = 0;   // VSMD 32 位状态字

// ---- RX 帧捕获（由 USART2 中断填充） ----
#define PUMP_RX_BUF_LEN 32                              // 接收缓冲区长度
static volatile uint8_t g_rx_buf[PUMP_RX_BUF_LEN];      // 接收缓冲区
static volatile uint8_t g_rx_len = 0;                   // 当前接收帧长度
static volatile uint8_t g_rx_cap = 0;                   // 1=正在接收帧,0=空闲等待起始符

// ---- 发送 ----
static void pump_send_str(const char *s)                // 发送字符串（ISR 上下文可调用，非阻塞）
{
    if (!g_rs232_inited) return; // USART2 未初始化，直接丢弃
    while (*s) 
    {
        USART_SendData(USART2, (uint8_t)(*s)); // 发送一个字节
        volatile uint32_t guard = 0; // 防止硬件异常导致死锁
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) // 等待发送完成
        {
            if (++guard > 0x20000u) return;     // 防止硬件异常导致死锁
        }
        s++; // 继续发送下一个字节
    }
}

//
// 发送 "mov spd=±值\n"（不依赖库函数，自行格式化整数）
static void pump_send_mov(int32_t spd)
{
    char buf[24];//1.组装命令字符串
    int i = 0; 
    buf[i++] = 'm'; buf[i++] = 'o'; buf[i++] = 'v'; // mov
    buf[i++] = ' '; buf[i++] = 's'; buf[i++] = 'p'; buf[i++] = 'd'; buf[i++] = '='; //spd=
    //2.如果是负数，先输出负号，然后取绝对值
    if (spd < 0) { buf[i++] = '-'; spd = -spd; }
    char tmp[12]; int j = 0;
    // 3.如果速度值为0，直接输出'0'
    if (spd == 0) tmp[j++] = '0';
    //4.当速度值大于0时，将速度值整数转换为字符串型式，存入临时数组 tmp 中
    while (spd > 0) { tmp[j++] = (char)('0' + (spd % 10)); spd /= 10; }
    //5.将临时数组 tmp 中的数字倒序输出到 buf 中
    while (j > 0) buf[i++] = tmp[--j];
    buf[i++] = '\n';
    buf[i] = '\0';
    // 6.发送命令字符串
    pump_send_str(buf);//输出的是 "mov spd=±值\n"这个字符串
}

// 取当前速度幅度：优先用 485 保持寄存器 REG_HOLDING_P1_SPEED(0x16)，
// 为0或未配置时回退到 PUMP_SPEED_DEFAULT，保证向后兼容。
static uint16_t pump_speed_get(void)
{
    uint16_t v;
    ENTER_CRITICAL();
    v = holding_regs[REG_HOLDING_P1_SPEED]; // 0x16:此处就是去读取保持寄存器
    EXIT_CRITICAL();
    if (v == 0) v = PUMP_SPEED_DEFAULT; // 0x16=0时回退到默认值6400，保证向后兼容
    return v;
}

// 组装并发送控制命令（调用上下文：任务级，非中断）
static void pump_send_cmd(uint8_t action)
{
    if (action == PUMP_ACTION_NONE) //1.动作是停止柱塞泵时
	{
        if (g_pump_enabled)     // 仅使能过才发停止，避免上电噪声
        {        
            pump_send_str("stp\n"); // 发送停止指令
            g_pump_enabled = 0; // 发送停止指令后，标记为未使能
        }
        return; //柱塞泵动作非0时，才执行后面操作（即非PUMP_ACTION_NONE时）
    }

    if (!g_pump_enabled)    // 先使能
    {           
        pump_send_str("ena\n");
        g_pump_enabled = 1;
    }

    // 速度幅度来自 485 保持寄存器 REG_HOLDING_P1_SPEED(0x16)，主机可远程整定；
    // 方向由动作决定：抽=正速，推=负速。
    uint16_t spd_mag = pump_speed_get();
    g_pump_speed_cache = spd_mag;

    if (action == PUMP_ACTION_DRAW)  //2.动作时抽母液时，抽母液: 正速
    {
        pump_send_mov((int32_t)spd_mag);
    }
    else // 3.动作是推母液时，推母液: 负速
    {
        pump_send_mov(-(int32_t)spd_mag);
    }
}

// 5 字节 7-bit 重组为 32 位无符号值：v = b0 | b1<<7 | b2<<14 | b3<<21 | b4<<28
static uint32_t regroup_u32(const uint8_t *p)   //富遥测 sts 帧解析
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 7)
         | ((uint32_t)p[2] << 14)
         | ((uint32_t)p[3] << 21)
         | ((uint32_t)p[4] << 28);
}

// ---- 解析反馈帧（ISR 上下文，只更新模块内部状态） ----
static void parse_frame(const uint8_t *buf, uint8_t len)//解析反馈帧，显示当前转速、脉冲、故障等
{
    if (len < 4) return;

    g_last_rx_tick = Scheduler_GetTick();   // 任何完整帧都表示链路存活

    uint8_t fb = buf[1];

    // BCC 校验（可选）：BCC = 异或(dev, fb#, data)，不含末尾 BH/BL
    uint8_t bcc = 0;
    for (uint8_t i = 0; i < (uint8_t)(len - 2); i++) bcc ^= buf[i];
    uint8_t bh = buf[len - 2];
    uint8_t bl = buf[len - 1];
    uint8_t bcc_rx = (uint8_t)(((uint16_t)bh << 7) | (bl & 0x7F));
    if (PUMP_BCC_CHECK && (bcc != bcc_rx)) return;   // 校验失败，本帧不采纳

    if (fb == 2 && len >= 19) {
        // data: 速度(5B) buf[2..6]  位置(5B) buf[7..11]  状态字(5B) buf[12..16]
        uint32_t raw_spd = regroup_u32(&buf[2]);   // IEEE754 float 的 32 位位模式
        uint32_t raw_pos = regroup_u32(&buf[7]);   // int32 脉冲数
        uint32_t status  = regroup_u32(&buf[12]);  // 32 位状态字

        // 速度：4 字节位模式按 IEEE754 解释为 float（Cortex-M3 小端，无需浮点运算）
        union { uint32_t u; float f; } su;
        su.u = raw_spd;
        g_pump_spd      = su.f;                    // 当前速度(IEEE754 float)
        g_pump_pos      = (int32_t)raw_pos;        // 当前位置 (int32_t符号扩展)
        g_pump_statword = status;                  // VSMD 32 位状态字

        uint8_t fault = 0;
        if (status & (1u << 6)) fault |= 0x01;   // 硬件错误 flt
        if (status & (1u << 9)) fault |= 0x02;   // 指令错误 cmd_wrg
        g_pump_fault = fault;

        if (fault) {
            g_pump_status = PUMP_STATUS_FAULT;
        } 
        else if (status & (1u << 8)) {
            // bit8 stp: 0=运行中, 1=停止中
            g_pump_status = PUMP_STATUS_STOP;
        } 
        else {
            g_pump_status = PUMP_STATUS_RUN;
        }
    }
}

// ---- 初始化 ----
void RS232_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // USART2 挂在 APB1 总线！(此前错挂在 APB2/USART1，导致外设不工作)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // PA2=TX(复用推挽)  PA3=RX(浮空输入)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   // 浮空输入
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate; // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; //  奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // 发送+接收
    USART_Init(USART2, &USART_InitStructure); 

    // 开启 RX 中断 + NVIC
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; // USART2 中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // NVIC使能
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE); // 使能 USART2

    g_rs232_inited = 1;  // 标记已初始化
    g_last_rx_tick = Scheduler_GetTick(); // 初始化时刻
}

// ---- 对外接口 ----
void RS232_Pump_SetAction(uint8_t action, uint32_t duration_ms)
{
    g_pump_action = action; // 仅记录动作，实际执行由 RS232_Task() 周期轮询
    //g_pump_running = (action != PUMP_ACTION_NONE);
    g_pump_duration_ms = duration_ms;
    g_pump_start_tick = Scheduler_GetTick();
    if (action == PUMP_ACTION_NONE) 
	{
        g_pump_running = 0;
        pump_send_cmd(PUMP_ACTION_NONE);
    } 
	else 
	{
        g_pump_running = 1;     // 仅设置运行标志，实际执行由 RS232_Task() 周期轮询
        pump_send_cmd(action);   // ena + mov spd=±X
    }
}

void RS232_Pump_Stop(void)
{
    g_pump_action = PUMP_ACTION_NONE;
    g_pump_running = 0;
    g_pump_duration_ms = 0;
    pump_send_cmd(PUMP_ACTION_NONE);
}

uint8_t RS232_Pump_IsRunning(void)
{
    return g_pump_running;
}

uint8_t RS232_Pump_GetStatus(void)
{
    return g_pump_status;
}

uint8_t RS232_Pump_GetFault(void)
{
    return g_pump_fault;
}

// ---- 中断接收：USART2 每收到一字节调用一次 ----
// 在 USART2 中断里逐字节收，按 FF … FE 边界拼成一帧。
void RS232_RxByte(uint8_t b)
{
    if (b == 0xFF) {            // 帧头
        g_rx_len = 0;
        g_rx_cap = 1;
        return;
    }
    if (b == 0xFE) {            // 帧尾
        if (g_rx_cap) 
        {
            parse_frame(g_rx_buf, g_rx_len);    // 解析反馈帧，更新状态
            g_rx_cap = 0;                       // 结束接收
        }
        return;
    }
    if (g_rx_cap && g_rx_len < PUMP_RX_BUF_LEN)     // 正在接收帧且缓冲区未满
    {
        g_rx_buf[g_rx_len++] = b;
    }
}

// ---- 周期任务（20ms） ----
void RS232_Task(void)
{
    uint32_t now = Scheduler_GetTick();     // 当前时刻(ms)

    // 1) 周期性查询状态，保持状态新鲜并喂看门狗
    static volatile uint32_t sts_tick = 0;
    if (Scheduler_IsTimeout(&sts_tick, 1000))   // 每秒查询一次状态 
    {
        sts_tick = now;     // 更新查询时刻
        if (g_rs232_inited) pump_send_str("sts\n");     // 查询状态指令
    }

    // 2) 通讯看门狗：运行状态下超 PUMP_COMM_TIMEOUT_MS 无反馈 → 故障(0xFE)
    if (g_pump_running &&
        Scheduler_IsTimeout(&g_last_rx_tick, PUMP_COMM_TIMEOUT_MS)) //运行状态下超时 
    {
        g_pump_status = PUMP_STATUS_FAULT;      //柱塞泵状态设置为故障
        g_pump_fault  = 0xFE;     // 通讯丢失
    }

    // 2b) 速度热更新：主机在 485 寄存器 0x16 改速度时，运行中即生效
    
        uint16_t sv = pump_speed_get();     // 读取保持寄存器 0x16 的速度幅度
        if (sv != g_pump_speed_cache)       // 速度幅度有变化，立即下发 mov spd=±X 指令
        {
            g_pump_speed_cache = sv;        // 更新缓存
            if (g_pump_running && g_pump_action != PUMP_ACTION_NONE)    // 仅在运行中才下发 mov 指令
            {
                int32_t s = (g_pump_action == PUMP_ACTION_DRAW)     // 方向由动作决定：抽=正速，推=负速
                            ? (int32_t)sv : -(int32_t)sv;
                pump_send_mov(s);   // 下发 mov spd=±X 指令，立即生效
            }
        }
    

    // 3) 定时自动停止（duration 模式；在线流程通常 duration=0，由时序机在步切换时停止）
    if (g_pump_running && g_pump_duration_ms != 0) 
    {
        if (Scheduler_IsTimeout(&g_pump_start_tick, g_pump_duration_ms))    // 超时，自动停止
        {
            RS232_Pump_Stop();  // 自动停止柱塞泵
        }
    }

    // 4) 回写输入寄存器（任务级，避免中断直接写共享区）

    // 4a) 简单状态 / 故障（已启用，保持不变）
    input_regs[REG_INPUT_PUMP1_STATUS] = g_pump_status;     // 0=停止,1=运行,2=故障
    input_regs[REG_INPUT_PUMP1_FAULT]  = g_pump_fault;      // 0=无故障,0x01=硬件错,0x02=指令错,0xFE=通讯超时

    // 4b) 富遥测映射到 0x20~0x27
    //     通讯有效性用只读比较，绝不调用 Scheduler_IsTimeout(&g_last_rx_tick)，
    //     以免干扰上方看门狗对 g_last_rx_tick 的重置语义。
    uint8_t comm_ok = (uint8_t)((now - g_last_rx_tick) < PUMP_COMM_TIMEOUT_MS);

    uint8_t state;
    if (!comm_ok)                              state = 3;   // 超时(链路丢失)
    else if (g_pump_fault)                     state = 2;   // 错误
    else if (g_pump_status == PUMP_STATUS_RUN) state = 1;   // 忙(运行中)
    else                                       state = 0;   // 空闲
    input_regs[REG_INPUT_P1_STATE] = state;

    // 错误码：0=无；0xFE=通讯超时；其余=状态字故障位映射(0x01 硬件错 / 0x02 指令错)
    input_regs[REG_INPUT_P1_ERR] = comm_ok ? g_pump_fault : 0xFE;

    // 位置 int32 → 高/低 16 位
    int32_t pos = g_pump_pos;
    input_regs[REG_INPUT_P1_POS_H] = (uint16_t)(pos >> 16);
    input_regs[REG_INPUT_P1_POS_L] = (uint16_t)(pos & 0xFFFF);

    // 速度 IEEE754 float → 32 位位模式 → 高/低 16 位（高字在前，见 shared_data.h 注释）
    union { uint32_t u; float f; } spd;
    spd.f = g_pump_spd;
    input_regs[REG_INPUT_P1_SPD_H] = (uint16_t)(spd.u >> 16);
    input_regs[REG_INPUT_P1_SPD_L] = (uint16_t)(spd.u & 0xFFFF);

    // 状态字 uint32 → 高/低 16 位
    uint32_t sw = g_pump_statword;
    input_regs[REG_INPUT_P1_STATUS_H] = (uint16_t)(sw >> 16);
    input_regs[REG_INPUT_P1_STATUS_L] = (uint16_t)(sw & 0xFFFF);
}
