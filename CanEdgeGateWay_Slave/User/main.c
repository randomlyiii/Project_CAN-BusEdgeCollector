#include "stm32f10x.h"
#include "delay.h"
#include "oled.h"
#include "dht11.h"
#include "CAN_User.h"
#include "KEY.h"
#include <stdio.h>
#include <string.h>

/* ======================== OLED 显示缓冲区 ======================== */
static char g_oled_line[4][17];

/* ======================== 周期任务定时器 ======================== */
static uint32_t g_last_oled_ms     = 0;
static uint32_t g_last_heartbeat   = 0;
static uint32_t g_last_dht11       = 0;

#define OLED_UPDATE_MS           300
#define HEARTBEAT_INTERVAL_MS    500
#define DHT11_INTERVAL_MS        2000

/* ======================== CAN TX 计数 ======================== */
static uint32_t g_can_tx_count = 0;

/* ======================== OLED 更新 ======================== */

static void OLED_UpdateDisplay(void)
{
    uint8_t temp_int, temp_dec, humi_int, humi_dec;
    uint8_t dht11_ok = !DHT11_Read_Data(&humi_int, &humi_dec, &temp_int, &temp_dec);

    /* 第1行: 节点信息 */
    sprintf(g_oled_line[0], "S:Node#%02u %s",
            SLAVE_NODE_ID,
            dht11_ok ? "Ready" : "FAIL");

    /* 第2行: 温度 */
    if (dht11_ok)
        sprintf(g_oled_line[1], "Tmp:%u.%u C   ", temp_int, temp_dec);
    else
        sprintf(g_oled_line[1], "Tmp:--.- C   ");

    /* 第3行: 湿度 */
    if (dht11_ok)
        sprintf(g_oled_line[2], "Hum:%u.%u %%RH ", humi_int, humi_dec);
    else
        sprintf(g_oled_line[2], "Hum:--.- %%RH ");

    /* 第4行: CAN 发送计数 + 状态 */
    sprintf(g_oled_line[3], "TX:%05lu      ", g_can_tx_count);

    /* 刷新 OLED */
    for (uint8_t i = 0; i < 4; i++) {
        g_oled_line[i][16] = '\0';
        OLED_ShowString(i + 1, 1, g_oled_line[i]);
    }
}

/* ======================== 主函数 ======================== */

int main(void)
{
    uint8_t dht11_temp_int = 0, dht11_temp_dec = 0;
    uint8_t dht11_humi_int = 0, dht11_humi_dec = 0;

    /* ---- 系统初始化 ---- */
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

    /* 初始化定时器 */
    g_last_oled_ms     = Delay_GetTick();
    g_last_heartbeat   = Delay_GetTick();
    g_last_dht11       = Delay_GetTick();

    /* ---- 主循环 ---- */
    while (1)
    {
        uint32_t now = Delay_GetTick();

        /* --- 1. DHT11 温湿度采集 (每 2s) --- */
        if (now - g_last_dht11 >= DHT11_INTERVAL_MS) {
            if (DHT11_Read_Data(&dht11_humi_int, &dht11_humi_dec,
                                &dht11_temp_int, &dht11_temp_dec) == 0) {
                /* 采集成功 → 发送温湿度帧 */
                CAN_SendTempHumi(dht11_temp_int, dht11_temp_dec,
                                 dht11_humi_int, dht11_humi_dec);
                g_can_tx_count++;
            }
            g_last_dht11 = now;
        }

        /* --- 2. 心跳发送 (每 500ms) --- */
        if (now - g_last_heartbeat >= HEARTBEAT_INTERVAL_MS) {
            CAN_SendHeartBeat();
            g_can_tx_count++;
            g_last_heartbeat = now;
        }

        /* --- 3. 按键扫描 (非阻塞) --- */
        {
            uint8_t key_evt = KEY_Scan();
            switch (key_evt) {
            case KEY_EVT_KEY1_SHRT:
                CAN_SendAlarm();
                g_can_tx_count++;
                break;
            case KEY_EVT_KEY2_SHRT:
                CAN_SendRecover();
                g_can_tx_count++;
                break;
            default:
                break;
            }
        }

        /* --- 4. OLED 刷新 (每 300ms) --- */
        if (now - g_last_oled_ms >= OLED_UPDATE_MS) {
            OLED_UpdateDisplay();
            g_last_oled_ms = now;
        }

        /* --- 5. 简单喂狗 / 空闲延时 --- */
        /* 不加延时: 让 CPU 全速跑循环, 保证按键响应及时 */
    }
}
