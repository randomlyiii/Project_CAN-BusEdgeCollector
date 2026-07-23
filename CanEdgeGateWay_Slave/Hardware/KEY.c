#include "KEY.h"
#include "delay.h"

/* ========== 消抖参数 ========== */
#define KEY_DEBOUNCE_MS     20      // 消抖延时 20ms
#define KEY_LONG_PRESS_MS   1000    // 长按判定 1000ms

/* 按键状态 */
typedef struct {
    uint8_t  last_level;            // 上次电平: 1=高, 0=低
    uint32_t press_start_tick;      // 按下起始 tick
    uint8_t  long_reported;         // 长按已上报标志
    uint8_t  short_pending;         // 短按待处理
} KeyState_t;

static KeyState_t g_key1 = {1, 0, 0, 0};
static KeyState_t g_key2 = {1, 0, 0, 0};

/* ========== 初始化 ========== */
void KEY_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(KEY1_GPIO_RCC | KEY2_GPIO_RCC, ENABLE);

    /* KEY1 PA0, KEY2 PA1 → 上拉输入 */
    gpio.GPIO_Pin   = KEY1_GPIO_PIN | KEY2_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 读初始电平 */
    g_key1.last_level = GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN);
    g_key2.last_level = GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_GPIO_PIN);
}

/* ========== 单键扫描 (非阻塞) ========== */
static uint8_t ScanOneKey(KeyState_t *key, uint8_t (*is_pressed)(void), uint32_t now,
                          uint8_t evt_short, uint8_t evt_long)
{
    uint8_t level = is_pressed() ? 0 : 1;

    /* 电平未变化 */
    if (level == key->last_level) {
        /* 仍为按下状态 → 检测长按 */
        if (level == 0) {
            if (!key->long_reported && (now - key->press_start_tick >= KEY_LONG_PRESS_MS)) {
                key->long_reported = 1;
                return evt_long;
            }
        }
        return KEY_EVT_NONE;
    }

    /* 电平变化: 从高→低 (刚按下) */
    if (level == 0) {
        key->press_start_tick = now;
        key->long_reported     = 0;
        key->short_pending     = 1;
    }
    /* 从低→高 (松开) */
    else {
        if (key->short_pending && (now - key->press_start_tick >= KEY_DEBOUNCE_MS)) {
            key->short_pending = 0;
            key->last_level = 1;
            /* 短按判定: 从按下到松开 < 长按阈值 */
            if (now - key->press_start_tick < KEY_LONG_PRESS_MS) {
                return evt_short;
            }
        }
    }

    key->last_level = level;
    return KEY_EVT_NONE;
}

/* ========== 按键扫描入口 ========== */
uint8_t KEY_Scan(void)
{
    uint32_t now = Delay_GetTick();
    uint8_t  evt;

    evt = ScanOneKey(&g_key1, KEY1_PRESS, now,
                     KEY_EVT_KEY1_SHRT, KEY_EVT_KEY1_LONG);
    if (evt != KEY_EVT_NONE) return evt;

    evt = ScanOneKey(&g_key2, KEY2_PRESS, now,
                     KEY_EVT_KEY2_SHRT, KEY_EVT_KEY2_LONG);
    if (evt != KEY_EVT_NONE) return evt;

    return KEY_EVT_NONE;
}
