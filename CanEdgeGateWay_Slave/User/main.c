#include "stm32f10x.h"
#include "delay.h"
#include "oled.h"
#include "dht11.h"
#include "CAN_User.h"
#include "KEY.h"
#include <stdio.h>
#include <string.h>

#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle; };
FILE __stdout = {0};
void _sys_exit(int x) { x = x; }

static char g_oled_line[4][17];

static uint32_t g_last_oled_ms     = 0;
static uint32_t g_last_heartbeat   = 0;
static uint32_t g_last_dht11       = 0;

#define OLED_UPDATE_MS           300
#define HEARTBEAT_INTERVAL_MS    500
#define DHT11_INTERVAL_MS        2000

static uint32_t g_can_tx_count   = 0;
static uint32_t g_key_evt_count  = 0;

/* 缓存DHT11数据, 只在主循环2s采集一次, OLED不直接读传感器 */
static uint8_t g_temp_int  = 0, g_temp_dec  = 0;
static uint8_t g_humi_int  = 0, g_humi_dec  = 0;
static uint8_t g_dht11_ok  = 0;

static void OLED_UpdateDisplay(void)
{
    sprintf(g_oled_line[0], "S:Node#%02u %s",
            SLAVE_NODE_ID, g_dht11_ok ? "Ready" : "FAIL");

    if (g_dht11_ok) {
        sprintf(g_oled_line[1], "Tmp:%u.%u C   ", g_temp_int, g_temp_dec);
        sprintf(g_oled_line[2], "Hum:%u.%u %%RH ", g_humi_int, g_humi_dec);
    } else {
        sprintf(g_oled_line[1], "Tmp:--.- C   ");
        sprintf(g_oled_line[2], "Hum:--.- %%RH ");
    }

    sprintf(g_oled_line[3], "TX:%05lu K:%lu ", g_can_tx_count, g_key_evt_count);

    for (uint8_t i = 0; i < 4; i++) {
        g_oled_line[i][16] = '\0';
        OLED_ShowString(i + 1, 1, g_oled_line[i]);
    }
}

int main(void)
{
    Delay_InitTick();
    Delay_ms(200);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "S: Init...");

    DHT11_GPIO_Init();
    OLED_ShowString(2, 1, "   DHT11 OK");

    CAN_User_Init();
    OLED_ShowString(3, 1, "   CAN OK");

    KEY_Init();
    OLED_ShowString(4, 1, "   Key OK");

    Delay_ms(1000);
    OLED_Clear();

    g_last_oled_ms   = Delay_GetTick();
    g_last_heartbeat = Delay_GetTick();
    g_last_dht11     = Delay_GetTick();

    while (1)
    {
        uint32_t now = Delay_GetTick();

        /* DHT11采集, 2s一次, 结果缓存到全局变量 */
        if (now - g_last_dht11 >= DHT11_INTERVAL_MS) {
            g_dht11_ok = (DHT11_Read_Data(&g_humi_int, &g_humi_dec,
                                          &g_temp_int, &g_temp_dec) == 0);
            if (g_dht11_ok) {
                CAN_SendTempHumi(g_temp_int, g_temp_dec,
                                 g_humi_int, g_humi_dec);
                g_can_tx_count++;
            }
            g_last_dht11 = now;
        }

        /* 心跳 + 温湿度一起发送 (500ms) */
        if (now - g_last_heartbeat >= HEARTBEAT_INTERVAL_MS) {
            CAN_SendHeartBeat();
            CAN_SendTempHumi(g_temp_int, g_temp_dec,
                             g_humi_int, g_humi_dec);
            g_can_tx_count++;
            g_last_heartbeat = now;
        }

        /* 按键扫描 */
        {
            uint8_t key_evt = KEY_Scan();
            if (key_evt == KEY_EVT_KEY1_SHRT) {
                CAN_SendAlarm(); g_can_tx_count++; g_key_evt_count++;
            } else if (key_evt == KEY_EVT_KEY2_SHRT) {
                CAN_SendRecover(); g_can_tx_count++; g_key_evt_count++;
            }
        }

        /* OLED刷新 (300ms), 只读缓存不碰传感器 */
        if (now - g_last_oled_ms >= OLED_UPDATE_MS) {
            OLED_UpdateDisplay();
            g_last_oled_ms = now;
        }
    }
}
