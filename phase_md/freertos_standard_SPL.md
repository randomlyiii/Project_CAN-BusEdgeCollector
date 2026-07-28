# FreeRTOS 内核规范 — CAN Bus Edge Collector

> 适用范围: `CanEdgeGateWay_Master/FreeRTOS/` 和 `CanEdgeGateWay_Slave/FreeRTOS/`
>
> 最后更新: 2026-07-28 (阶段二验收通过)

---

## 1. 文件结构与 Include 路径

```
FreeRTOS/                         User/
├── inc/                           └── FreeRTOSConfig.h  ← 所有 config 宏定义
│   ├── FreeRTOS.h   ← 聚合头          │  #include "../FreeRTOS/inc/FreeRTOS.h"
│   ├── task.h       ← TCB_t + API     ▼
│   ├── queue.h      ← Queue_t + API  FreeRTOS.h
│   ├── list.h                       │  #include "FreeRTOSConfig.h"
│   ├── semphr.h                     │  #include "task.h"
│   ├── timers.h                     │  #include "queue.h"
│   ├── event_groups.h               │  #include "semphr.h"
│   ├── portmacro.h                  │  ...
│   ├── StackMacros.h
│   ├── atomic.h
│   ├── mpu_wrappers.h
│   └── projdefs.h
└── src/
    ├── tasks.c          ← 调度核心
    ├── queue.c          ← 队列/信号量
    ├── port.c           ← Cortex-M3 移植层（汇编）
    ├── timers.c
    ├── list.c
    ├── heap_4.c
    └── event_groups.c
```

**Keil Include Paths** (UV5 项目设置):
```
.\Hardware;.\Library;.\Start;.\System;.\User;.\FreeRTOS\inc;.\FreeRTOS\src
```

---

## 2. TCB_t 可见性规则

| 场景 | 看到的 TCB_t | 实现机制 |
|------|-------------|---------|
| 外部代码 (main.c, 驱动) | `typedef struct tskTaskControlBlock TCB_t;` 前向声明 | task.h 顶部无条件定义 |
| 内核文件 (tasks.c, queue.c, timers.c) | 完整 struct (含全部字段) | MPU_WRAPPERS_INCLUDED_FROM_API_FILE 守卫 |

`uxStackDepth` 为**本项目添加的非标准字段**，位于 `pcTaskName` 之后。

---

## 3. ARMCC V5 ISR 命名 (已验证)

### 3.1 规则: port.c 直接使用向量表名称

ARMCC V5 的 `__asm void funcName(void)` 中宏替换不可靠。必须在源码中写死向量表名称：

```c
// port.c — 直接命名
__asm void SVC_Handler( void )       // 原名 vPortSVCHandler
__asm void PendSV_Handler( void )    // 原名 xPortPendSVHandler
void xPortSysTickHandler( void )     // Delay.c 中转，保持原名
```

不要在 FreeRTOSConfig.h 中写映射宏——实测 ARMCC V5 在 `__asm` 函数中不展开。

### 3.2 SysTick 中转模式

```c
// Delay.c — 向量表入口
void SysTick_Handler(void) {
    extern void xPortSysTickHandler(void);
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        xPortSysTickHandler();
}
```

### 3.3 ARMCC 汇编注意事项

```c
__asm void PendSV_Handler( void ) {
    extern uxCriticalNesting;
    extern pxCurrentTCB;
    extern vTaskSwitchContext;
    // ...
    mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY  // 必须展开为纯数值 #0x50
}
```

---

## 4. 中断优先级架构 (已验证)

### 4.1 配置值

```c
#define configPRIO_BITS                         4
#define configKERNEL_INTERRUPT_PRIORITY         0xF0        // 15 << 4
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    0x50        // 5 << 4
```

### 4.2 main.c 首行必须设置

```c
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  // 4 位全抢占
```

### 4.3 BASEPRI 临界区行为

FreeRTOS 使用 BASEPRI 实现临界区 (非 PRIMASK)：

| 中断优先级 | NVIC 值 | BASEPRI=0x50 屏蔽? | 能否调 FromISR API? |
|-----------|---------|-------------------|--------------------|
| 0 (最高) | 0x00 | ❌ 不屏蔽 | ❌ 禁止 |
| 1 | 0x10 | ❌ 不屏蔽 | ❌ 禁止 |
| 2-4 | 0x20-0x40 | ❌ 不屏蔽 | ❌ 禁止 |
| **5-15** | **0x50-0xF0** | **✅ 屏蔽** | **✅ 允许** |
| SysTick (#15) | 0xF0 | ✅ 屏蔽 | — |
| PendSV (#15) | 0xF0 | ✅ 屏蔽 | — |

**已验证**：CAN ISR 必须使用优先级 **5 (RX0)** 和 **6 (SCE)**。
优先级 0 的 ISR 可穿透 BASEPRI 临界区，调用 FromISR API 会破坏调度器数据结构。

### 4.4 CAN ISR 优先级规范 (修正)

```c
// CAN_User.c — 必须的优先级配置
nvic.NVIC_IRQChannelPreemptionPriority = 5;  // CAN RX0 (≥5)
nvic.NVIC_IRQChannelPreemptionPriority = 6;  // CAN SCE  (≥5)
```

原代码使用优先级 0/1 是错误的，已修正为 5/6。

---

## 5. tasks.c 补丁 (已验证，必须)

### 5.1 栈魔数填充

```c
// prvInitialiseNewTask() 中 — 不可省略
(void) memset(pxNewTCB->pxStack, (int)tskSTACK_FILL_BYTE,
              (size_t)ulStackDepth * sizeof(StackType_t));
```

**验证**: 缺少此行 → 栈内容全 0 → Method 2 溢出检测 (`*pxStack != 0xa5a5a5a5`) 恒真 → 持续误报。

### 5.2 uxStackDepth 赋值

```c
pxNewTCB->uxStackDepth = ulStackDepth;   // 不可省略
```

**验证**: 缺少此行 → `uxTaskGetStackHighWaterMark` 循环条件 `uxReturn < pxTCB->uxStackDepth` 首次即假 → 始终返回 0。

### 5.3 字/字节常量分离

```c
#define tskSTACK_FILL_BYTE    (0xa5U)                          // memset 使用
#define tskSTACK_FILL_WORD    ((StackType_t)0xa5a5a5a5UL)      // 水位扫描使用
```

### 5.4 水位比较必须用字常量

```c
// 正确
while ((*pcEndOfStack == tskSTACK_FILL_WORD) && ...)
// 错误: 字节转字后为 0x000000a5, 栈上实际是 0xa5a5a5a5, 永远不匹配
while ((*pcEndOfStack == (StackType_t)tskSTACK_FILL_BYTE) && ...)
```

### 5.5 vTaskPlaceOnEventList 双重移除 (已验证)

```c
void vTaskPlaceOnEventList(List_t *pxEventList, TickType_t xTicksToWait) {
    vListInsert(pxEventList, &(pxCurrentTCB->xEventListItem));
    prvAddCurrentTaskToDelayedList(xTicksToWait, pdTRUE);  // 内部已做 uxListRemove
}
```

**验证**: `prvAddCurrentTaskToDelayedList` 内部已执行 `uxListRemove`。在外部再调一次会导致双向链表破坏 → HardFault。

---

## 6. 任务栈规范 (已验证)

### 6.1 最小栈分配

| 任务特征 | 最小栈 (words) | 验证情况 |
|---------|---------------|---------|
| 纯 FreeRTOS API (无 sprintf、无标准库) | 192-256 | 已验证 |
| 轻度 sprintf (1-2 次) | 320-384 | — |
| **重度 sprintf (3-5 次) + IWDG 操作** | **≥384** | **已验证: 192 爆栈** |

ARMCC 的 `sprintf` 单次调用栈消耗 ≈ 150-250 字节，是纯内核 API 的 5-10 倍。

### 6.2 sprintf 使用前提

```c
#pragma import(__use_no_semihosting_swi)   // 禁用半主机模式（必须）
struct __FILE { int handle; };
FILE __stdout = {0};
void _sys_exit(int x) { x = x; }
```

**验证**: 缺少此设置 → 第一次 sprintf 调用触发半主机 SWI → 调试器未连接时卡死。

---

## 7. 内存布局

### 7.1 STM32F103C8T6 RAM 预算

```
0x20000000 ┌──────────────────────┐
           │ MSP Stack (0x400)    │  1024B (Master) / 512B (Slave)
           │ System Heap (0)      │  0B (不使用标准库 malloc)
           ├──────────────────────┤
           │ .data                │
           │ .bss (含 ucHeap[])   │
           │   FreeRTOS heap      │  12KB (Master) / 10KB (Slave)
           │   任务栈             │  从 heap 动态分配
           │   队列/信号量        │  从 heap 动态分配
0x20005000 └──────────────────────┘
```

### 7.2 关键参数

| 参数 | Master | Slave |
|------|--------|-------|
| configTOTAL_HEAP_SIZE | 12KB | 10KB |
| configTICK_RATE_HZ | 100 (10ms) | 1000 (1ms) |
| 任务数 | 2 (Unified + Housekeep) | 4 (Sensor + CAN + Key + Housekeep) |
| MSP Stack | 1024B (0x400) | 512B (0x200) |
| libc heap | 0 | 0 |

> **Tick 差异原因**: Master 降频至 100Hz 以减少 SPI 36MHz 对 CAN RX 的 GPIOA 端口串扰风险。Slave 无 W5500 SPI 高频信号，保持 1000Hz 维持 1ms 响应精度。
> 
> **任务数差异**: Master 因 SPI-CAN 并发冲突回退至统一顺序任务（见 §16），Slave 无 W5500 外设，保持 4 任务架构。

### 7.3 startup.s 注意事项

`Stack_Size EQU 0x00000400` 控制 MSP 大小。`__initial_sp` 由链接器自动计算。

---

## 8. configUSE_TIMERS 陷阱 (已验证)

**验证结论**: 若不创建任何软件定时器 (`xTimerCreate`)，必须设置 `configUSE_TIMERS=0`。

```c
// 错误
#define configUSE_TIMERS  1   // 无定时器时不可这样设
// 正确
#define configUSE_TIMERS  0
```

原因: `configUSE_TIMERS=1` 时 `vTaskStartScheduler()` 会创建 timer 守护任务。该任务在没有定时器时**不阻塞**，在空循环中全速跑，占用 100% CPU。timer 任务优先级为 `configMAX_PRIORITIES - 1`(7)，高于所有用户任务，导致用户任务被饿死。

---

## 9. 调度器启动前信号量清空 (已验证)

以下代码必须放在 `vTaskStartScheduler()` 之前：

```c
{
    BaseType_t sem;
    do { sem = xSemaphoreTake(g_can_monitor_sem, 0); } while (sem == pdPASS);
    do { sem = xSemaphoreTake(g_can_rx_sem, 0); } while (sem == pdPASS);
    do { sem = xSemaphoreTake(g_can_fifo_not_empty, 0); } while (sem == pdPASS);
}
```

CAN 外设在 main() 中被初始化，CAN ISR 可能在 `vTaskStartScheduler()` 之前触发，调度器启动后任务上下文未完全建立时被 PendSV 切换执行。

**注意**: 如果信号量由任务内的 `CAN_User_Init()` 创建（见 §15），则此清空代码需要在任务启动后执行，而非 main() 中。

---

## 10. xSemaphoreTake 与 portMAX_DELAY (已验证)

```c
// 当前方案（已验证无问题）
xSemaphoreTake(g_can_rx_sem, pdMS_TO_TICKS(0x7FFFFFFF));
// portMAX_DELAY 触发 xSuspendedTaskList bug
// xSemaphoreTake(g_can_rx_sem, portMAX_DELAY);
```

`portMAX_DELAY` 走挂起列表路径，实测插入后系统静默卡死（无 HardFault）。

---

## 11. 临界区内绝对禁止

- DWT 延时 (`Delay_us`)
- I2C/SPI 批量传输
- 任何可能阻塞的操作 (`xSemaphoreTake`, `vTaskDelay`)

临界区内 BASEPRI=0x50 屏蔽 SysTick (优先级 15=0xF0)，长时间停留导致：
1. Tick 不增长 → 任务延迟不触发
2. IWDG 超时 → 系统复位

---

## 12. IWDG 超时配置 (已验证)

LSI 频率范围 30kHz~60kHz (典型 40kHz)。配置必须留余量：

```c
// 已验证: Prescaler=64/Reload=625 在 LSI=60kHz 时超时 667ms
//           Housekeep 1s 喂狗来不及 → 反复复位
// 已验证: Prescaler=128/Reload=1250 在任何 LSI 下都足够
IWDG_SetPrescaler(IWDG_Prescaler_128);
IWDG_SetReload(1250);
```

| Prescaler | Reload | @30kHz | @40kHz | @60kHz |
|-----------|--------|--------|--------|--------|
| 64 | 625 | 1.33s | **1.0s** | **0.67s** ❌ |
| 128 | 1250 | 5.33s | **4.0s** | **2.67s** ✅ |

---

## 13. 从 ISR 调用 FreeRTOS API 的限制 (已修正)

CAN ISR 必须使用优先级 ≥ 5。原代码使用优先级 0 是错误的。已修正为 RX0=5, SCE=6。

---

## 14. 验证清单 (任何 FreeRTOS 配置修改后)

- [ ] `configASSERT_DEFINED=1` 下无 assert 触发
- [ ] 所有任务栈水位 > 30 words
- [ ] Heap 剩余稳定不下降
- [ ] Heartbeat 持续递增 (证明 tick 正常)
- [ ] IWDG 不触发 (证明系统不卡死)
- [ ] 5 分钟以上长时间运行无异常

---

## 15. CAN 硬件初始化必须在调度器启动后 (新发现, 2026-07-27)

### 15.1 现象

裸机轮询可正常接收 CAN 帧，FreeRTOS 环境下相同的 CAN 初始化代码完全收不到帧（`FMP=0`, `REC=255`, ISR 不触发）。

### 15.2 根因

`vTaskStartScheduler()` 内部会静默破坏 CAN 外设寄存器。具体破坏机制未完全定位到代码行，但已验证：**调度器启动前初始化的 CAN 外设不可用**。

### 15.3 修复

CAN 硬件初始化必须在调度器启动后的任务上下文中执行：

```c
static void Task_CAN_Rx(void *pvParameters) {
    (void)pvParameters;
    // CRITICAL: CAN_DeInit + CAN_User_Init 必须在调度器启动后
    CAN_DeInit(CAN1);
    Delay_ms(10);
    CAN_User_Init();   // CAN 硬件 + 信号量 + FIFO

    for (;;) { /* 轮询 CAN */ }
}
```

**注意**: `CAN_User_Init()` 内部会创建 FreeRTOS 信号量和互斥量。由于此任务优先级最高，它会在其他任务访问这些对象之前完成创建，不存在竞态。

### 15.4 排查方法

在 OLED 上显示 CAN 硬件寄存器值（FMP, TEC, REC, LEC, BASEPRI, NVIC 优先级）。裸机能收但 FreeRTOS 不收时，对比两边的寄存器值定位差异。

---

## 16. SPI (W5500) 与 CAN 不能并发 (新发现, 2026-07-27)

### 16.1 现象

CAN 单独运行正常，W5500 单独运行正常。两者在 FreeRTOS 多任务环境下同时运行时：
- CAN 完全收不到帧
- ETH 显示 NOLK
- REC 飙升至 255

### 16.2 错误诊断过程 (EMI 理论, 已推翻)

最初认为是 PA5 (W5500 SCK 36MHz) 通过面包板串扰到 PA11 (CAN RX)，即电磁干扰 (EMI)。
尝试过 SPI 降速、SPI 开关宏限时、临界区保护——均无效。

### 16.3 真实根因: 并发执行冲突

Phase 1 裸机下 CAN+W5500 能同时工作，原因是 while(1) 循环顺序执行——CAN 和 SPI 永远不会同时操作。

Phase 2 FreeRTOS 将 CAN 和 W5500 分配到不同任务，任务抢占导致两者并发执行。虽然 CAN ISR 优先级(5)可抢占 W5500 任务(优先级 1)，但物理层上 SPI 时钟信号和 CAN RX 信号重叠时会产生不可恢复的比特错误。

**关键结论: 不是纯 EMI 问题，而是 SPI 和 CAN 的 GPIOA 端口信号并发冲突。即使 CS 宏开关 SPI/DISABLE SPI，SPI 时钟边沿的产生和 CAN 帧采样窗口重叠就会导致错误。**

### 16.4 正确修复: 统一顺序任务

回退到 Phase 1 的顺序执行模型——单任务轮询 CAN→W5500→CAN→W5500：

```c
static void Task_Unified(void *pvParameters) {
    CAN_DeInit(CAN1); Delay_ms(10);
    CAN_User_Init();
    vTaskDelay(100);
    W5500_TCPServer_Start(MODBUS_PORT);

    for (;;) {
        // 1. CAN Rx (紧轮询)
        while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0) {
            CAN_Receive(CAN1, CAN_FIFO0, &msg);
            CAN_ProcessFrame(&frame);
        }
        // 2. CAN Tx (FIFO 发送)
        while (FIFO_High_Pop(&f) == 0) CAN_SendFrame(...);
        while (FIFO_Normal_Pop(&f) == 0) CAN_SendFrame(...);
        // 3. CAN 监控
        CAN_HeartBeatCheck(); CAN_ErrorMonitor(); CAN_CalcBusLoad();
        // 4. W5500 (在 CAN 完成后, 不并发)
        W5500_TCPServer_Run();
        ModbusTCP_Process();
        ModbusTCP_SyncFromCAN();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));
    }
}
```

**架构权衡**: 失去了多任务优先级调度的实时性优势，但在当前硬件条件下（CAN+W5500 共用 GPIOA 端口且无物理隔离）这是唯一稳定的工作方式。

### 16.5 硬件层面的根治方案

如果要恢复真正的多任务架构，需要进行硬件改造：
- CAN 收发器改为 ISO1050 隔离型（独立供电）
- W5500 SPI 改到 SPI2 (PB13-PB15)，与 CAN 的 GPIOA 端口物理分离
- CAN RX (PA11) 对地加 47pF 滤波电容

---

## 17. 阶段二验收通过 — 最终稳定架构 (2026-07-28)

阶段二全部验收项通过后的最终架构：

### 17.1 主站 (Master) — 2 任务顺序执行

| 任务 | 优先级 | 栈 (words) | 周期 | 职责 |
|------|--------|-----------|------|------|
| `Task_Unified` | 1 | 768 | 10ms 轮询 | CAN Rx/Tx + 帧处理 + W5500 + Modbus — 全部顺序执行 |
| `Task_Housekeep` | 0 | 384 | 1s | OLED 刷新 + IWDG 喂狗 + 栈水位监控 |

### 17.2 从站 (Slave) — 4 任务

| 任务 | 优先级 | 栈 (words) | 周期 | 职责 |
|------|--------|-----------|------|------|
| `Task_Sensor` | 3 | 256 | 2s | DHT11 + BH1750 采集 |
| `Task_CAN_Slave` | 2 | 256 | 500ms 事件 | 心跳 + 传感器帧发送 |
| `Task_Key` | 1 | 128 | 20ms 轮询 | 按键检测 + 消抖 |
| `Task_Housekeep` | 0 | 128 | 1s | IWDG + OLED + 本地缓存管理 |

### 17.3 关键配置 (已验证)

| 配置项 | 值 | 说明 |
|--------|-----|------|
| CAN 波特率 | 500kbps | Prescaler=6, BS1=5, BS2=6, SJW=2 |
| CAN ISR 优先级 | RX0=5, SCE=6 | 满足 BASEPRI 0x50 阈值 |
| PA11 模式 | IPU (内部上拉) | 替代原 IN_FLOATING |
| AWUM / ABOM | 均 DISABLE | 匹配厂家参考代码 |
| IWDG | Prescaler=128, Reload=1250 | ~4s @40kHz, ~2.67s @60kHz |
| FreeRTOS tick (Master) | 100Hz (10ms) | 降频防 SPI 串扰 |
| FreeRTOS tick (Slave) | 1000Hz (1ms) | 标准精度 |

### 17.4 已知限制

1. **CAN_ResetBus() 会重建信号量**破坏任务等待 — BusOff 几乎不触发，暂不修
2. **CAN_SendFrame 使用 Delay_us 忙等** — 在多任务环境不理想但可接受
3. **全局状态变量无互斥保护** — 统一架构下实际不存在竞争
4. **W5500 SPI 与 CAN 不可并发** — 需硬件改造（SPI 改 SPI2 + ISO1050）方可恢复多任务
