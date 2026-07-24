#include "stm32f10x.h"
#include "delay.h"
#include "oled.h"
#include "CAN_User.h"
#include "W5500.h"
#include "ModbusTCP.h"
#include <stdio.h>
#include <string.h>

#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle; };
FILE __stdout = {0};
void _sys_exit(int x) { x = x; }

static char     g_oled_line[4][17];
static uint32_t g_last_oled_tick    = 0;
static uint32_t g_last_hbcheck_tick = 0;
static uint32_t g_last_sync_tick    = 0;

#define OLED_UPDATE_MS   300
#define HB_CHECK_MS      200
#define SYNC_MODBUS_MS   100

void CAN_User_OnError(uint8_t level)      { (void)level; }
void CAN_User_OnAlarm(uint8_t node_id, uint8_t func) { (void)node_id; (void)func; }
void CAN_User_OnNodeUpdate(uint8_t node_id) { (void)node_id; }

/*
 * 获取以太网状态字符串
 *   "FAIL"=芯片异常  "NOLK"=无网线  "CON"=已连接
 *   "LSN"=监听中    "CLS"=已关闭    "SR:XX"=其他状态
 */
static const char *ETH_StateStr(void)
{
    if (!W5500_IsOnline())                   return "FAIL";
    if (!W5500_LinkUp())                     return "NOLK";
    if (W5500_IsConnected())                 return "CON ";

    uint8_t sr = W5500_ReadByte(BSB_SOCK0_REG, OFF_SN_SR);
    if (sr == SOCK_LISTEN)                   return "LSN ";
    if (sr == SOCK_CLOSED)                   return "CLS ";

    static char buf[6];
    sprintf(buf, "SR:%02X", sr);
    return buf;
}

static void OLED_UpdateDisplay(void)
{
    uint8_t online1 = g_slave_nodes[0].online;
    uint8_t fault1  = g_slave_nodes[0].fault_flag;
    const char *can = (g_can_error.error_level == 0) ? "OK" : "ERR";

    /* 第1行: CAN状态 + 以太网状态 */
    sprintf(g_oled_line[0], "CAN:%-3s ETH:%-4s", can, ETH_StateStr());

    /* 第2行: 从站1 温度 + 总线负载 */
    if (online1) {
        uint16_t load = g_can_error.bus_load;  /* 0~10000 = 0.00%~100.00% */
        if (load < 1000) {
            /* <10%: 显示两位小数, 如 L:0.06% */
            sprintf(g_oled_line[1], "S1:%d.%dC L:%u.%02u%%",
                    g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec,
                    load / 100, load % 100);
        } else {
            /* >=10%: 四舍五入到一位小数, 节省1字符, 如 L:90.7% */
            uint16_t r = (load + 5) / 10;  /* 0~1000, 单位0.1% */
            if (r >= 1000) {
                sprintf(g_oled_line[1], "S1:%d.%dC L:100%%",
                        g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec);
            } else {
                sprintf(g_oled_line[1], "S1:%d.%dC L:%u.%u%%",
                        g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec,
                        r / 10, r % 10);
            }
        }
    } else
        sprintf(g_oled_line[1], "S1:--.-C L:--%% ");

    /* 第3行: 湿度 + 心跳 */
    sprintf(g_oled_line[2], "H:%d.%d%% HB:%-5u",
            g_slave_nodes[0].humi_int, g_slave_nodes[0].humi_dec,
            g_slave_nodes[0].heartbeat_count);

    /* 第4行: R=CAN帧 T=温湿度帧 + 状态 */
    {
        char r_str[5], t_str[4];
        const char *status;
        uint32_t rx = g_can_rx_int_count;
        uint32_t th = g_can_rx_temp_count;

        if      (rx >= 1000000) sprintf(r_str, "%luM", rx/1000000);
        else if (rx >= 10000)   sprintf(r_str, "%luK", rx/1000);
        else if (rx >= 1000)    sprintf(r_str, "%lu.%luK", rx/1000, (rx%1000)/100);
        else                    sprintf(r_str, "%lu", rx);

        if      (th >= 1000000) sprintf(t_str, "%luM", th/1000000);
        else if (th >= 10000)   sprintf(t_str, "%luK", th/1000);
        else if (th >= 1000)    sprintf(t_str, "%lu.%luK", th/1000, (th%1000)/100);
        else                    sprintf(t_str, "%lu", th);

        if (!online1)               status = "OFF";
        else if (fault1)            status = "ALM";
        else if (g_slave_nodes[0].blacklist) status = "BLK";
        else                        status = "OK ";

        sprintf(g_oled_line[3], "R%-4s T%-3s %-3s", r_str, t_str, status);
    }

    uint8_t i;
    for (i = 0; i < 4; i++) {
        g_oled_line[i][16] = '\0';
        OLED_ShowString(i + 1, 1, g_oled_line[i]);
    }
}

int main(void)
{
    int8_t ret;

    Delay_InitTick();
    Delay_ms(200);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "M: Init...");

    CAN_User_Init();
    OLED_ShowString(2, 1, "   CAN OK");

    /* --- W5500 初始化 --- */
    ret = W5500_Init();
    if (ret != W5500_OK) {
        OLED_ShowString(3, 1, "   W5500 FAIL!");
        char diag[17];
        sprintf(diag, "   Ver:0x%02X", g_w5500_version);
        OLED_ShowString(4, 1, diag);
        while (1);  /* 芯片异常 → 停在这里 */
    }

    ret = W5500_ConfigNetwork();
    if (ret != W5500_OK) {
        OLED_ShowString(3, 1, "   NET CFG ERR");
    } else {
        OLED_ShowString(3, 1, "   ETH OK");
        OLED_ShowString(4, 1, "Server OK");
        W5500_TCPServer_Start(MODBUS_PORT);
    }

    Delay_ms(1500);
    ModbusTCP_Init();
    OLED_Clear();

    g_last_oled_tick    = Delay_GetTick();
    g_last_hbcheck_tick = Delay_GetTick();
    g_last_sync_tick    = Delay_GetTick();

    while (1)
    {
        uint32_t now = Delay_GetTick();

        if (now - g_last_hbcheck_tick >= HB_CHECK_MS) {
            CAN_HeartBeatCheck();
            CAN_ErrorMonitor();
            CAN_CalcBusLoad();
            g_last_hbcheck_tick = now;
        }

        W5500_TCPServer_Run();

        if (now - g_last_sync_tick >= SYNC_MODBUS_MS) {
            ModbusTCP_SyncFromCAN();
            g_last_sync_tick = now;
        }

        ModbusTCP_Process();

        if (now - g_last_oled_tick >= OLED_UPDATE_MS) {
            OLED_UpdateDisplay();
            g_last_oled_tick = now;
        }
    }
}
