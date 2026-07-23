#include "stm32f10x.h"
#include "delay.h"
#include "oled.h"
#include "CAN_User.h"
#include "W5500.h"
#include "ModbusTCP.h"
#include <stdio.h>
#include <string.h>

/* ======================== OLED 显示缓冲区 ======================== */
static char g_oled_line[4][17];     // 4行×16字符

/* ======================== 周期任务定时器 ======================== */
static uint32_t g_last_oled_tick      = 0;
static uint32_t g_last_hbcheck_tick   = 0;
static uint32_t g_last_sync_tick      = 0;

/* OLED 刷新间隔 */
#define OLED_UPDATE_MS              300    // 300ms 刷新一次
#define HB_CHECK_MS                 200    // 200ms 检测一次心跳
#define SYNC_MODBUS_MS              100    // 100ms 同步一次寄存器

/* ======================== CAN 回调实现 ======================== */

/* 错误事件回调 */
void CAN_User_OnError(uint8_t level)
{
    (void)level;
}

/* 报警事件回调 */
void CAN_User_OnAlarm(uint8_t node_id, uint8_t func)
{
    (void)node_id;
    (void)func;
}

/* 从站数据更新回调 */
void CAN_User_OnNodeUpdate(uint8_t node_id)
{
    (void)node_id;
}

/* ======================== OLED 显示更新 ======================== */

static void OLED_UpdateDisplay(void)
{
    uint8_t  i;
    int16_t  temp_c1, humi_pct1;
    uint8_t  online1, fault1;

    /* ---- 第1行: 主站状态 ---- */
    /* "M:CAN OK Eth:Con"  或 "M:CAN ERR Eth:Dis" */
    sprintf(g_oled_line[0], "M:%s E:%s",
            (g_can_error.error_level == 0) ? "CAN OK" : "CAN ERR",
            W5500_IsConnected() ? "CON" : "DIS");

    /* ---- 第2行: 从站1 温度 ---- */
    /* "S1:25.3C Onln   " 或 "S1:OFFLINE     " */
    online1 = g_slave_nodes[0].online;
    fault1  = g_slave_nodes[0].fault_flag;
    if (online1) {
        temp_c1 = (int16_t)g_slave_nodes[0].temp_int * 10 + g_slave_nodes[0].temp_dec;
        sprintf(g_oled_line[1], "S1:%d.%dC%s%s",
                g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec,
                fault1 ? " ALM" : "    ",
                g_slave_nodes[0].blacklist ? "BLK" : "   ");
    } else {
        sprintf(g_oled_line[1], "S1:OFFLN    ");
    }

    /* ---- 第3行: 从站1 湿度 + CAN 负载率 ---- */
    /* "H:62.1% Ld:12% " */
    if (g_slave_nodes[0].online) {
        humi_pct1 = (int16_t)g_slave_nodes[0].humi_int * 10 + g_slave_nodes[0].humi_dec;
        sprintf(g_oled_line[2], "H:%d.%d%% Ld:%d%% ",
                g_slave_nodes[0].humi_int, g_slave_nodes[0].humi_dec,
                g_can_error.bus_load / 10);  // 负载率 0~1000 → 0.0%~100.0%
    } else {
        sprintf(g_oled_line[2], "Ld:%d%% HB:%d  ",
                g_can_error.bus_load / 10,
                g_slave_nodes[0].heartbeat_count);
    }

    /* ---- 第4行: CAN 错误状态 + 以太网IP ---- */
    /* "TEC:%3d REC:%3d " */
    if (g_can_error.error_level > 0) {
        sprintf(g_oled_line[3], "T:%d R:%d Lv%d",
                g_can_error.tec, g_can_error.rec, g_can_error.error_level);
    } else {
        sprintf(g_oled_line[3], "192.168.1.100  ");
    }

    /* ---- 刷新 OLED ---- */
    for (i = 0; i < 4; i++) {
        g_oled_line[i][16] = '\0';            // 截断到16字符
        OLED_ShowString(i + 1, 1, g_oled_line[i]);
    }
}

/* ======================== 主函数 ======================== */

int main(void)
{
    /* ---- 系统初始化 ---- */
    Delay_InitTick();             // SysTick 1ms 滴答 (必须在最前)
    Delay_ms(200);                // 上电稳定延时

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "M: Init...");

    CAN_User_Init();
    OLED_ShowString(2, 1, "   CAN OK");

    W5500_Init();
    W5500_ConfigNetwork();
    W5500_TCPServer_Start(MODBUS_PORT);
    OLED_ShowString(3, 1, "   ETH OK");

    ModbusTCP_Init();
    OLED_ShowString(4, 1, "   Ready!");

    Delay_ms(1000);
    OLED_Clear();

    /* 初始化定时器 */
    g_last_oled_tick    = Delay_GetTick();
    g_last_hbcheck_tick = Delay_GetTick();
    g_last_sync_tick    = Delay_GetTick();

    /* ---- 主循环 ---- */
    while (1)
    {
        uint32_t now = Delay_GetTick();

        /* --- 1. CAN 接收处理 (每次循环都检查) --- */
        if (g_can_rx_flag) {
            CAN_ProcessRxFrame();
        }

        /* --- 2. 心跳超时检测 (200ms) --- */
        if (now - g_last_hbcheck_tick >= HB_CHECK_MS) {
            CAN_HeartBeatCheck();
            CAN_ErrorMonitor();
            CAN_CalcBusLoad();
            g_last_hbcheck_tick = now;
        }

        /* --- 3. W5500 TCP Server 轮询 --- */
        W5500_TCPServer_Run();

        /* --- 4. Modbus 寄存器同步 (100ms) --- */
        if (now - g_last_sync_tick >= SYNC_MODBUS_MS) {
            ModbusTCP_SyncFromCAN();
            g_last_sync_tick = now;
        }

        /* --- 5. Modbus TCP 请求处理 --- */
        ModbusTCP_Process();

        /* --- 6. OLED 刷新 (300ms) --- */
        if (now - g_last_oled_tick >= OLED_UPDATE_MS) {
            OLED_UpdateDisplay();
            g_last_oled_tick = now;
        }

        /* --- 7. 简单看门狗喂狗 (如果启用了 IWDG) --- */
        /*  (当前阶段未用硬件看门狗) */
    }
}
