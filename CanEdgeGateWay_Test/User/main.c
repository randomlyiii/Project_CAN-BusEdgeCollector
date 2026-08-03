/**
 * CAN TX Stress / Slave Sim — 主站测试工具 (裸机)
 *
 * 模式 (Config/test_config.h):
 *   TEST_SIM_SLAVE==1  模拟从站: 周期心跳 + 固定温湿度 (帧与从站1 完全一致)
 *   TEST_SIM_SLAVE==0  压测: 按帧率灌帧, 测主站接收能力
 *
 * 发送计数由 CAN_User.c 内部维护 (g_can_tx_success_count / g_can_tx_fail_count).
 *
 * OLED 显示 (16 字符/行, 无闪烁 — 刷新只改数字, 不清屏):
 *   Line1  模式/节点
 *   Line2  正在发送的数据帧: 温湿度 或 心跳/告警
 *   Line3  S=成功发送 R=测试站自收帧数
 *   Line4  F=发送失败 TEC=CAN发送错误计数
 */

#include "stm32f10x.h"
#include "delay.h"
#include "test_config.h"
#include "CAN_User.h"
#include "oled.h"

/* ==================== OLED (无闪烁: 初始化写静态, 刷新只写数字) ==================== */

#if (TEST_DISP_ENABLE == 1)

static void Disp_Refresh(void);   /* 前向声明 — Disp_Init 调用 */

/* 行2: 显示最近发送帧内容 (温湿度数值实时在线) */
static void Disp_ShowLastFrame(void)
{
    OLED_ShowString(2, 1, "                ");   /* 16 空格清残留, 防帧类型切换残影 */
    switch (g_last_tx[CAN_DATA_FUNC_IDX]) {
    case CAN_FUNC_TEMP_HUMI:
        OLED_ShowString(2, 1,  "T:");
        OLED_ShowNum   (2, 3,  g_last_tx[3], 2);
        OLED_ShowChar  (2, 5,  '.');
        OLED_ShowNum   (2, 6,  g_last_tx[4], 1);
        OLED_ShowChar  (2, 7,  'C');
        OLED_ShowChar  (2, 8,  ' ');
        OLED_ShowChar  (2, 9,  'H');
        OLED_ShowChar  (2, 10, ':');
        OLED_ShowNum   (2, 11, g_last_tx[5], 2);
        OLED_ShowChar  (2, 13, '%');
        break;
    case CAN_FUNC_HEARTBEAT:
        OLED_ShowString(2, 1,  "HB SRC:");
        OLED_ShowNum   (2, 8,  g_last_tx[1], 2);
        break;
    case CAN_FUNC_ALARM:
        OLED_ShowString(2, 1,  "ALM SRC:");
        OLED_ShowNum   (2, 9,  g_last_tx[1], 2);
        break;
    default:
        OLED_ShowString(2, 1,  "FRM SRC:");
        OLED_ShowNum   (2, 9,  g_last_tx[1], 2);
        break;
    }
}

static void Disp_Init(void)
{
    OLED_Clear();
#if (TEST_SIM_SLAVE == 1)
    OLED_ShowString(1, 1, "SIM SLAVE");
    OLED_ShowChar  (1, 11, 'N');
    OLED_ShowNum   (1, 12, TEST_NODE_ID_BASE, 2);
#else
    OLED_ShowString(1, 1, "RATE:");
    if (TEST_FRAME_RATE > 0) {
        OLED_ShowNum(1, 6, TEST_FRAME_RATE, 4);
        OLED_ShowChar(1, 10, '/');
        OLED_ShowChar(1, 11, 's');
    } else {
        OLED_ShowString(1, 6, "MAX ");
    }
#endif
    Disp_ShowLastFrame();
    Disp_Refresh();
}

/* 刷新: 只写数字字段, 不清屏 → 无闪烁.
 * Line3: S=成功发送 R=测试站自收帧数 (总线双向验证)
 * Line4: F=发送失败 TEC=CAN发送错误计数 (>0 = 信号/接线异常)
 */
static void Disp_Refresh(void)
{
    Disp_ShowLastFrame();
    OLED_ShowString(3, 1, "S:");
    OLED_ShowNum   (3, 3, g_can_tx_success_count, 7);
    OLED_ShowString(3, 10, " R:");
    OLED_ShowNum   (3, 13, g_rx_count, 3);
    OLED_ShowString(4, 1, "F:");
    OLED_ShowNum   (4, 3, g_can_tx_fail_count, 3);
    OLED_ShowString(4, 6, " TEC:");
    OLED_ShowNum   (4, 11, g_can_tec, 3);
}

#endif /* TEST_DISP_ENABLE */

/* ==================== 从站模拟: 周期心跳 + 固定温湿度 ==================== */

#if (TEST_SIM_SLAVE == 1)

static void Run_SlaveSim(void)
{
    uint32_t hb_last   = 0;
    uint32_t data_last = 0;
    uint32_t disp_last = 0;
    uint32_t start_ms  = Delay_GetTick();

    for (;;) {
        uint32_t now = Delay_GetTick();

        if ((now - hb_last) >= TEST_HB_INTERVAL_MS) {
            CAN_SendHeartBeat();            /* 与从站1 完全一致的帧 */
            hb_last = now;
        }
        if ((now - data_last) >= TEST_DATA_INTERVAL_MS) {
            CAN_SendTempHumi(TEST_TEMP_INT, TEST_TEMP_DEC,
                             TEST_HUMI_INT, TEST_HUMI_DEC);
            data_last = now;
        }

        if (TEST_RUN_TIME_S > 0 &&
            (now - start_ms) >= (TEST_RUN_TIME_S * 1000UL)) {
            return;
        }

        CAN_RxPoll();                       /* 排空 RX + 读 TEC/REC */

#if (TEST_DISP_ENABLE == 1)
        if ((now - disp_last) >= TEST_DISP_REFRESH_MS) {
            Disp_Refresh();
            disp_last = now;
        }
#endif
    }
}

/* ==================== 压测: 按帧率灌帧 ==================== */

#else /* TEST_SIM_SLAVE == 0 */

static void Run_Stress(void)
{
    uint32_t next_us     = Delay_GetUs();
    uint32_t interval_us = 0;
    uint32_t disp_last   = 0;
    uint32_t start_ms    = Delay_GetTick();

    if (TEST_FRAME_RATE > 0)
        interval_us = 1000000UL / TEST_FRAME_RATE;

    for (;;) {
        CAN_TxSendOne();

        if (TEST_RUN_TIME_S > 0 &&
            (Delay_GetTick() - start_ms) >= (TEST_RUN_TIME_S * 1000UL)) {
            return;
        }

        CAN_RxPoll();                       /* 排空 RX + 读 TEC/REC */

        if (TEST_FRAME_RATE > 0) {          /* 节拍 */
            next_us += interval_us;
            while (Delay_GetUs() < next_us) { }
        }

#if (TEST_DISP_ENABLE == 1)
        if ((Delay_GetTick() - disp_last) >= TEST_DISP_REFRESH_MS) {
            Disp_Refresh();
            disp_last = Delay_GetTick();
        }
#endif
    }
}

#endif /* TEST_SIM_SLAVE */

/* ==================== 入口 ==================== */

int main(void)
{
    Delay_InitTick();

#if (TEST_DISP_ENABLE == 1)
    OLED_Init();
    Disp_Init();
#endif

    CAN_User_Init();

#if (TEST_SIM_SLAVE == 1)
    Run_SlaveSim();
#else
    Run_Stress();
#endif

    while (1) { }
}
