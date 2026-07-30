# 从站复用架构规范 — CAN Bus Edge Collector

> 适用范围: `CanEdgeGateWay_Slave/` 从站工程  
> 最后更新: 2026-07-29 (文档同步最终阶段完成)

---

## 1. 架构概述

从站采用 **配置驱动 + 条件编译** 复用架构。同一份源码通过一行配置宏切换，编译出不同从站固件。

```
slave_config.h (SLAVE_NODE_VARIANT = SLAVE_NODE_01)
    ├─ #include "slave_node_01.h"  → DHT11 + BH1750 (温湿度+光照节点)
    └─ #include "slave_node_02.h"  → LM393 + RESERVED (光敏+预留节点)
```

### 1.1 设计目标

| 目标 | 实现方式 |
|------|---------|
| 单代码仓双变体 | `#if slave_has_xxx` 条件编译 |
| 传感器热插拔 | 传感器表 (`g_sensor_table[]`) + 适配器模式 |
| 新增传感器零代码改动 | 只需: ①写驱动 + ②写适配器 + ③改配置头文件 |
| 未用传感器零开销 | 条件编译排除 → 不编译/不链接/不占 Flash/RAM |
| 向后兼容 | 底层硬件驱动接口不变，不改已有代码 |

---

## 2. 配置层（Config Layer）

### 2.1 切换入口 (`slave_config.h`)

```c
#define SLAVE_NODE_VARIANT   SLAVE_NODE_01   // ← 改此行切换

#if   SLAVE_NODE_VARIANT == SLAVE_NODE_01
  #include "slave_node_01.h"
#elif SLAVE_NODE_VARIANT == SLAVE_NODE_02
  #include "slave_node_02.h"
#endif

// 安全默认: 未定义宏 → 0（防止静默编译）
#ifndef slave_has_dht11
  #define slave_has_dht11  0
#endif
// ... 同上 slave_has_bh1750 / slave_has_lm393 / slave_has_reserved

// 编译期约束: 最多 4 个传感器（STM32F103C8T6）
#if (slave_has_dht11 + slave_has_bh1750 + slave_has_lm393 + slave_has_reserved) > 4
  #error "Too many sensors enabled — max 4 for STM32F103C8T6"
#endif
```

### 2.2 节点配置头文件

每个节点 `.h` 文件定义：

```c
// === 节点身份 ===
#define slave_node_id           0x01        // CAN 节点地址
#define slave_node_name_str     "Node#01"   // OLED 显示名 (≤8 字符)
#define slave_node_desc_str     "T+H+Light" // 简短描述
#define slave_online_flag       1           // 上电即在线

// === 传感器使能 (1=启用, 0=排除) ===
#define slave_has_dht11         1
#define slave_has_bh1750        1
#define slave_has_lm393         0
#define slave_has_reserved      0

// === CAN 参数 ===
#define slave_can_id_offset     0x00
#define slave_heartbeat_ms      500
#define slave_interval_normal   2000
#define slave_interval_lowfreq  2000

// === 传感器采样间隔 (ms) ===
#define sensor_dht11_interval_ms    2000
#define sensor_bh1750_interval_ms   2000
```

### 2.3 传感器类型枚举 (`sensor_type.h`)

```c
#define SENSOR_TYPE_NONE        0x00
#define SENSOR_TYPE_DHT11       0x01   // == CAN_FUNC_TEMP_HUMI
#define SENSOR_TYPE_BH1750      0x02   // == CAN_FUNC_LIGHT
#define SENSOR_TYPE_LM393_DO    0x03   // LM393 数字输出
#define SENSOR_TYPE_LM393_AO    0x04   // LM393 模拟值
#define SENSOR_TYPE_RESERVED    0x05   // 预留通道（占位，无驱动）

#define SENSOR_PRIOR_NORMAL     1      // 1 级常态任务
#define SENSOR_PRIOR_LOWFREQ    2      // 2 级低频任务
```

---

## 3. 传感器管理抽象层

### 3.1 传感器数据联合 (`sensor_data_t`)

```c
typedef union {
    struct { uint8_t temp_int, temp_dec, humi_int, humi_dec; } dht11;
    struct { uint16_t lux; } bh1750;
    struct { uint8_t digital; uint16_t analog; } lm393;
    struct { uint16_t ch1; } reserved;
    uint8_t raw[8];          // 兼容 CAN 8 字节帧载荷
} sensor_data_t;
```

### 3.2 传感器描述符 (`sensor_t`)

```c
typedef struct {
    // 静态配置
    const char      name[12];           // 人类可读名
    uint8_t         type_id;            // SENSOR_TYPE_*
    uint8_t         priority;           // NORMAL / LOWFREQ
    uint16_t        interval_ms;        // 采样间隔
    uint8_t       (*init_fn)(void);     // 初始化函数, 0=OK; NULL=无驱动
    uint8_t       (*read_fn)(sensor_data_t *out); // 读取函数, NULL=无驱动
    uint8_t         enabled;            // 1=启用, 0=禁用

    // 运行时状态
    uint8_t         online;             // 0=离线, 1=在线
    uint8_t         fault_count;        // 连续失败次数
    uint8_t         alarm_active;       // 1=报警激活
    uint8_t         recover_pending;    // 1=待发 RECOVER 帧
    TickType_t      last_read_tick;     // 上次采样时刻
    sensor_data_t   last_data;          // 最近一次成功读值
} sensor_t;
```

### 3.3 传感器表（条件编译展开）

```c
static sensor_t g_sensor_table[] = {
#if slave_has_dht11
    { .name = "DHT11",  .init_fn = dht11_adapter_init,  .read_fn = dht11_adapter_read, ... },
#endif
#if slave_has_bh1750
    { .name = "BH1750", .init_fn = bh1750_adapter_init, .read_fn = bh1750_adapter_read, ... },
#endif
#if slave_has_lm393
    { .name = "LM393",  .init_fn = lm393_adapter_init,  .read_fn = lm393_adapter_read, ... },
#endif
#if slave_has_reserved
    { .name = "CH1",    .init_fn = NULL,                 .read_fn = NULL, ... },
#endif
    /* Sentinel: 至少一个条目（全禁用时的占位） */
    { .enabled = 0 }
};
#define SENSOR_COUNT  (sizeof(g_sensor_table) / sizeof(g_sensor_table[0]))
```

### 3.4 适配器模式

底层驱动 **DHT11/BH1750/LM393** 接口**保持不变**，通过薄适配器桥接到统一接口：

```c
// 示例 BH1750 适配器
#if slave_has_bh1750
static uint8_t bh1750_adapter_init(void) {
    BH1750_Init();
    return 0;
}
static uint8_t bh1750_adapter_read(sensor_data_t *out) {
    uint8_t ret;
    if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50));
    ret = BH1750_Read(&out->bh1750.lux);
    if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);
    return ret;
}
#endif
```

### 3.5 I2C 互斥锁保护

BH1750 和 OLED 共用 **PB8/PB9 (软件 I2C)**。BH1750 读取 (Sensor 任务, prio 3) 与 OLED 刷新 (Housekeep 任务, prio 0) 通过 `g_i2c_mutex` 互斥，FreeRTOS 优先级继承防止反转。

```c
// sensor_manager.h — I2C 互斥锁声明
extern SemaphoreHandle_t g_i2c_mutex;

// sensor_manager.c — 创建（sensor_manager_init 中）
g_i2c_mutex = xSemaphoreCreateMutex();

// BH1750 读取时加锁
if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50));
BH1750_Read(&out->bh1750.lux);
if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);

// OLED 刷新时加锁（Task_Housekeep 中）
if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50));
OLED_UpdateDisplay();
if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);
```

### 3.6 预留通道设计规范

| 规则 | 说明 |
|------|------|
| 结构体保留 | `sensor_data_t.reserved.ch1` (uint16) |
| 函数指针 NULL | `init_fn = NULL, read_fn = NULL` — 无驱动代码 |
| OLED 占位 | 显示 `CH1:----` |
| CAN 通讯点位 | `SENSOR_TYPE_RESERVED` → `CAN_FUNC_RESERVED (0x07)` |

扩展步骤:
1. 编写传感器驱动 (Hardware/my_sensor.h/c)
2. 编写适配器（替换 NULL → 函数指针）
3. 无需修改任何任务代码或已有传感器代码

---

## 4. 错误处理与故障容错

### 4.1 传感器故障升级

```
读取失败 1 次 → fault_count = 1
读取失败 2 次 → fault_count = 2
读取失败 3 次 → alarm_active = 1 → CAN 发送 ALARM 帧
读取成功 1 次 → fault_count = 0, alarm_active = 0, recover_pending = 1 → CAN 发送 RECOVER 帧
```

### 4.2 超时保护规范

> ⚠️ **强制规则**: 所有外设等待循环必须有超时退出（[[while-loop-timeout-required]]）

| 模块 | 超时机制 | 超时值 |
|------|---------|--------|
| CAN 发送 | `while(timeout--) + Delay_us(1)` | 2ms (2000 轮) |
| BH1750 I2C 互斥锁 | `xSemaphoreTake(timeout)` | 50ms |
| BH1750 I2C 读取 | 传感器内部超时 | 硬件超时 |
| LM393 ADC 校准 | `while(status && timeout--)` | 100000 轮 |
| OLED I2C 操作 | 由互斥锁超时保护 | 50ms |

### 4.3 CAN 发送失败 → 本地缓存

```c
if (CAN_SendFrame(...) != 0)
    LocalCache_Push(&g_local_cache, id, data);

if (LocalCache_ShouldReplay(&g_local_cache))  // CAN 恢复后补传
    LocalCache_Pop(&g_local_cache, &id, data) → CAN_SendFrame(id, data, 8)
```

缓存容量 50 条，满则丢弃最旧。

---

## 5. 从站 4 任务架构

```
Task_Sensor (prio 3, 200ms 扫描)
  └─ sensor_manager_read_all() ← 表驱动，一次调用处理所有传感器
      内部按 last_read_tick 控制各传感器实际采样间隔

Task_CAN_Slave (prio 2, 500ms)
  ├─ CAN init（调度器启动后，无中断）
  ├─ 错峰延迟 (按 node_id: S1=0ms, S2=2000ms)
  ├─ 轮询清空 RX FIFO (≤16 帧/周期)
  ├─ Heartbeat + 遍历传感器表 → CAN_SendSensorData()
  ├─ alarm_active → CAN_SendAlarm()
  └─ recover_pending → CAN_SendRecover()

Task_Key (prio 1, 20ms)
  └─ KEY1 → CAN_SendAlarm() / KEY2 → CAN_SendRecover()

Task_Housekeep (prio 0, 1s)
  └─ IWDG 喂狗 + 栈水位 + OLED_UpdateDisplay (表驱动行渲染器)
```

### 任务栈 (已验证)

| 任务 | 栈 (words) |
|------|-----------|
| Task_Sensor | 384 |
| Task_CAN_Slave | 320 |
| Task_Key | 192 |
| Task_Housekeep | 384 |

---

## 6. 空传感器保护

所有传感器均禁用时 (`slave_has_* = 0`)，`g_sensor_table[]` 仅含一个 **sentinel 条目** `{ .enabled = 0 }`，防止空数组编译错误。

`sensor_manager_get_count()` 遍历表计数 `enabled` 条目（忽略 sentinel）。

OLED 行渲染器检测 `sensor_count == 0` → 显示 `" No Sensor!    "`。

---

## 7. OLED 行渲染规范

### 7.1 16 字符硬限制

所有 `sprintf` 格式字符串均验证 ≤16 字符。超限值被 clamp 后再格式化。关键规则:

| 值 | 安全上限 | clamp 策略 |
|----|---------|-----------|
| 温度 (`temp_int`) | 99 | `if (ti > 99) ti = 99` |
| 湿度 (`humi_int`) | 99 | `if (hi > 99) hi = 99` |
| 光照 k-lux (`k`) | 99 | `if (k > 99) { k = 99; d = 9; }` |
| 心跳计数 | 99999 | `if (hb > 99999) hb = 99999` |
| 模拟值 (LM393 AO) | 9999 | `if (ao > 9999) ao = 9999` |

### 7.2 紧凑数值格式 (`OLED_CompactNum`)

```
val >= 1000000 → "1.2M" (4 chars)
val >= 10000   → "65K"  (3 chars)
val >= 1000    → "6.5K" (4 chars)
val < 1000     → "999"  (3 chars)
```

---

## 8. 配置变更清单

| 操作 | 改动 | 编译 | 重烧 |
|------|------|------|------|
| 切换从站变体 | `slave_config.h` 的 `SLAVE_NODE_VARIANT` | Rebuild All | 是 |
| 禁用单个传感器 | 对应 `slave_node_0x.h` 中 `slave_has_xxx=0` | Rebuild All | 是 |
| 调整采样间隔 | 对应 `sensor_xxx_interval_ms` | 改 .h 即可 | 是 |

> **原则**: 所有差异均通过配置宏控制，不改 C 源码。

---

## 9. 新增传感器接入流程

```
Step 1: 写驱动           Hardware/my_sensor.h/c
Step 2: 写适配器          sensor_manager.c 中新增 adapter_init/read
Step 3: 写配置宏          新增 slave_has_my_sensor → slave_node_0x.h
Step 4: 注册传感器表      g_sensor_table[] 中新增 #if slave_has_my_sensor 条目
Step 5: 更新 OLED 渲染    main.c 中补充 SENSOR_TYPE_* case
Step 6: 更新 CAN 功能码   如需要新功能码，在 CAN_User.h 中新增 CAN_FUNC_*
```

> 整个过程**无需修改任何任务代码或已存在的传感器代码**。

---

## 10. 相关文档

- `can_standard_SPI.md` — CAN 通信架构（主站双 FIFO + 从站零中断）
- `freertos_standard_SPL.md` — FreeRTOS 内核规范
- `oled_standard_SPI.md` — OLED 显示规范（I2C 引脚/时序/限制）
- `最终阶段.md` — 最终阶段完整设计文档
- `最终阶段遇到的问题及解决.md` — 调试修复记录
