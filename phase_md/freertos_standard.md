# FreeRTOS 内核规范 — CAN Bus Edge Collector

> 适用范围: `CanEdgeGateWay_Master/FreeRTOS/` 和 `CanEdgeGateWay_Slave/FreeRTOS/`
>
> 此为项目标准文档，非调试记录。调试过程记录在 [`阶段二遇到的问题及解决.md`](阶段二遇到的问题及解决.md)。
>
> 最后更新: 2026-07-24

---

## 1. 文件结构与 Include 路径

```
FreeRTOS/                         User/
├── inc/                           └── FreeRTOSConfig.h  ← 所有 config 宏定义
│   ├── FreeRTOS.h   ← 聚合头          │  #include "../FreeRTOS/inc/FreeRTOS.h"
│   ├── task.h       ← TCB_t + API     ▼
│   ├── queue.h      ← Queue_t + API  FreeRTOS.h
│   ├── list.h                       │  #include "FreeRTOSConfig.h"  ← Keil Paths → User/
│   ├── semphr.h                     │  #include "task.h"
│   ├── timers.h                     │  #include "queue.h"
│   ├── event_groups.h               │  #include "semphr.h"
│   ├── portmacro.h                  │  ...
│   ├── StackMacros.h  ← 自定义      ▼
│   ├── atomic.h         用户代码只需 #include "FreeRTOS.h"（或通过 FreeRTOSConfig.h 间接）
│   ├── mpu_wrappers.h
│   └── projdefs.h
└── src/
    ├── tasks.c          ← 内核调度核心
    ├── queue.c          ← 队列/信号量
    ├── port.c           ← Cortex-M3 移植层（汇编）
    ├── timers.c
    ├── list.c
    ├── heap_4.c
    └── event_groups.c
```

**Keil Include Paths**（UV5 项目设置）:
```
.\Hardware;.\Library;.\Start;.\System;.\User;.\FreeRTOS\inc;.\FreeRTOS\src
```

**包含规则**：`FreeRTOSConfig.h` 的最后一行必须是 `#include "../FreeRTOS/inc/FreeRTOS.h"`。所有 config 宏必须在 FreeRTOS.h 展开之前定义。

---

## 2. TCB_t 可见性规则

| 场景 | 看到的 TCB_t | 实现机制 |
|------|-------------|---------|
| 外部代码（main.c, 驱动） | `typedef struct tskTaskControlBlock TCB_t;` 前向声明 | task.h 顶部无条件定义 |
| 内核文件（tasks.c, queue.c, timers.c） | 完整 struct（含全部字段） | MPU_WRAPPERS_INCLUDED_FROM_API_FILE 守卫 |

**内核文件 include 模板**：
```c
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
#include "FreeRTOS.h"
#include "task.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE
```

> `uxStackDepth` 为**本项目添加的非标准字段**，位于 `pcTaskName` 之后。标准 FreeRTOS 不存储此字段，本项目为 `uxTaskGetStackHighWaterMark` 的边界检查而添加。

---

## 3. 队列结构体

`ucQueueTypeInternal` **为项目添加的非标准字段**，位于 `QueueDefinition` 末尾：

| 值 | 含义 |
|----|------|
| 0 | 普通队列 |
| 1 | 互斥锁 |
| 2 | 计数信号量 |
| 3 | 二值信号量 |
| 4 | 递归互斥锁 |

> 此字段的值与 `ucQueueType` 一致，但 `ucQueueType` 在某些 FreeRTOS 版本中被定义在别处。本字段作为冗余备份供内部判断用。

---

## 4. ISR Handler 命名规则（ARMCC V5）

### 4.1 规则：port.c 直接使用向量表名称

ARMCC V5 的 `__asm void funcName(void)` 中**宏替换不可靠**。必须直接在源码中写死向量表名称：

```c
// port.c — 直接命名
__asm void SVC_Handler( void )       // 原名 vPortSVCHandler
__asm void PendSV_Handler( void )    // 原名 xPortPendSVHandler
void xPortSysTickHandler( void )     // Delay.c 中转，保持原名
```

> 不要在 FreeRTOSConfig.h 中写 `#define vPortSVCHandler SVC_Handler` 等映射宏——实测 ARMCC V5 在 `__asm` 函数中不展开宏定义，弱符号死循环胜出。

### 4.2 SysTick 中转模式

```c
// Delay.c — 向量表入口
void SysTick_Handler(void) {
    extern void xPortSysTickHandler(void);
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        xPortSysTickHandler();    // 调用 port.c 的 tick 处理
}
```

**禁止**在 FreeRTOSConfig.h 中写 `#define xPortSysTickHandler SysTick_Handler`——port.c 与 Delay.c 都会导出 `SysTick_Handler` 强符号，链接冲突。

### 4.3 ARMCC 汇编注意事项

```c
// port.c — PendSV_Handler 汇编块内显式声明 extern 符号
__asm void PendSV_Handler( void ) {
    extern uxCriticalNesting;      // 汇编器不知道 C 的 extern 声明
    extern pxCurrentTCB;           // 必须在 __asm{} 块内重复声明
    extern vTaskSwitchContext;
    ...
}

// port.c — 汇编引用 configMAX_SYSCALL_INTERRUPT_PRIORITY
mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY   // 必须展开为纯数值 #0x50
// NOT: UL 后缀或表达式——ARM 汇编器报 A1586E: Bad operand types
```

---

## 5. 中断优先级架构

### 5.1 配置值

```c
// 以下三个宏在 FreeRTOSConfig.h 中固定，不可修改
#define configPRIO_BITS                         4           // 硬编码
#define configKERNEL_INTERRUPT_PRIORITY         0xF0        // 15 << 4
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    0x50        // 5 << 4
```

**必须使用纯十六进制常量**。`UL` 后缀或 `(5UL << (8 - configPRIO_BITS))` 等表达式在 port.c 内联汇编中展开后包含 `UL` → ARM 汇编器不认。

### 5.2 main.c 首行必须设置

```c
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  // 4 位全抢占，无子优先级
```

STM32 上电默认 `Group_0`（0 位抢占），会导致 **所有临界区防护失效**。FreeRTOS 的 BASEPRI 机制要求所有优先级位均为抢占位。

### 5.3 BASEPRI 临界区行为

本项目使用 **BASEPRI**（非 PRIMASK）实现临界区：

| 中断优先级 | NVIC 值 | BASEPRI=0x50 屏蔽? | 能否调 FromISR API? |
|-----------|---------|-------------------|--------------------|
| 0 (最高) | 0x00 | ❌ 不屏蔽 | ❌ **禁止** |
| 1 | 0x10 | ❌ 不屏蔽 | ❌ **禁止** |
| 2-4 | 0x20-0x40 | ❌ 不屏蔽 | ❌ **禁止** |
| **5-15** | **0x50-0xF0** | **✅ 屏蔽** | **✅ 允许** |
| **SysTick (#15)** | **0xF0** | **✅ 屏蔽** | — |
| **PendSV (#15)** | **0xF0** | **✅ 屏蔽** | — |

**关键含义**：
- 优先级 0-4 的 ISR（如 CAN_RX0=0, CAN_SCE=1）**可穿透临界区**，但**不可调用 FreeRTOS API**
- SysTick（优先级 15）**在临界区内被屏蔽** → Tick 不增长 → 任务延迟不触发
- PendSV（优先级 15）**在临界区内被屏蔽** → 无法上下文切换

---

## 6. 栈溢出检测

```c
// StackMacros.h — 标准实现（已修复，2026-07-24）
#if configCHECK_FOR_STACK_OVERFLOW == 1
    #define taskCHECK_FOR_STACK_OVERFLOW()                              \
    {                                                                   \
        if (pxCurrentTCB->pxTopOfStack <= pxCurrentTCB->pxStack)      \
            vApplicationStackOverflowHook(                             \
                (TaskHandle_t)pxCurrentTCB, pxCurrentTCB->pcTaskName); \
    }
#elif configCHECK_FOR_STACK_OVERFLOW == 2
    #define taskCHECK_FOR_STACK_OVERFLOW()                              \
    {                                                                   \
        if (*(pxCurrentTCB->pxStack) != 0xa5a5a5a5UL                 \
            || pxCurrentTCB->pxTopOfStack <= pxCurrentTCB->pxStack)  \
            vApplicationStackOverflowHook(                             \
                (TaskHandle_t)pxCurrentTCB, pxCurrentTCB->pcTaskName); \
    }
#endif
```

**规则**：
- `tskSTACK_FILL_BYTE = 0xa5U`（字节，给 memset 用）
- `tskSTACK_FILL_WORD = 0xa5a5a5a5UL`（字，给水位扫描比较用）
- 两者不可混用

---

## 7. tasks.c 补丁说明

本项目 tasks.c 相比标准 FreeRTOS V10.3.1 有以下修改：

### 7.1 栈魔数填充（不可省略）

```c
// prvInitialiseNewTask() 中 — 无条件执行
(void) memset(pxNewTCB->pxStack, (int)tskSTACK_FILL_BYTE,
              (size_t)ulStackDepth * sizeof(StackType_t));
```

缺少此行 → 栈内容为随机值 → Method 2 溢出检测恒真 → 持续误报。

### 7.2 uxStackDepth 赋值

```c
pxNewTCB->uxStackDepth = ulStackDepth;   // 不可省略
```

缺少此行 → `uxTaskGetStackHighWaterMark` 循环条件 `uxReturn < pxTCB->uxStackDepth` 首次即假 → **始终返回 0**。

### 7.3 字/字节常量分离

```c
#define tskSTACK_FILL_BYTE    (0xa5U)                          // memset 使用
#define tskSTACK_FILL_WORD    ((StackType_t)0xa5a5a5a5UL)      // 水位扫描使用
```

### 7.4 vTaskPlaceOnEventList / vTaskPlaceOnUnorderedEventList

**已修复为标准实现**（不可再添加 `uxListRemove`）：

```c
void vTaskPlaceOnEventList(List_t *pxEventList, TickType_t xTicksToWait) {
    vListInsert(pxEventList, &(pxCurrentTCB->xEventListItem));
    prvAddCurrentTaskToDelayedList(xTicksToWait, pdTRUE);   // 内部已做 uxListRemove
}
```

> `prvAddCurrentTaskToDelayedList` 内部已执行 `uxListRemove`。**在外面再调一次会导致双重移除**，操作野指针破坏双向链表 → HardFault。

---

## 8. 任务栈规范

### 8.1 最小栈分配

| 任务特征 | 最小栈 (words) | 示例 |
|---------|---------------|------|
| 纯 FreeRTOS API（无 sprintf、无标准库） | 192-256 | CAN_Rx(320), CAN_Mon(320) |
| 轻度 sprintf（1-2 次） | 320-384 | — |
| **重度 sprintf（3-5 次）+ IWDG 操作** | **≥384** | **Housekeep(384)** |

ARMCC 的 `sprintf` 单次调用栈消耗 ≈ 150-250 字节，是纯内核 API 的 5-10 倍。

### 8.2 sprintf 使用前提

```c
#pragma import(__use_no_semihosting_swi)   // 禁用半主机模式（必须）
struct __FILE { int handle; };
FILE __stdout = {0};
void _sys_exit(int x) { x = x; }
```

缺少此设置 → 第一次 `sprintf` 调用触发半主机 SWI → 调试器未连接时卡死。

---

## 9. 内存布局

### 9.1 STM32F103C8T6 RAM 预算

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

### 9.2 关键参数

| 参数 | Master | Slave |
|------|--------|-------|
| configTOTAL_HEAP_SIZE | 12KB | 10KB |
| MSP Stack | 1024B (0x400) | 512B (0x200) |
| libc heap | 0 | 0 |
| configMINIMAL_STACK_SIZE | 128 words | 128 words |

### 9.3 修改 startup.s 的风险

`startup_stm32f10x_md.s` 中 `Stack_Size EQU 0x00000400` 控制 MSP 大小：

- **偏小**（如 512B 跑 6 任务）：MSP 溢出静默踩 `ucHeap[]`，**无 HardFault**，表现为任务栈水位正常但系统随机崩溃
- **偏大**（如 2048B）：链接阶段报 `L6406E: No space`，不会留到运行时
- **`__initial_sp`** 由链接器自动计算为 `Stack_Mem + Stack_Size`，不要手动改

验证：检查 `.map` 文件中 `__initial_sp` ≤ `0x20005000`（STM32F103C8T6 RAM 尾端）。

---

## 10. BASEPRI 临界区使用规则

### 10.1 什么时候用

```
✅ 需要临界区:                     ❌ 不需要临界区:
  两个任务共享同一外设             一个任务独占整个外设
  操作全局结构体（非原子写）        操作任务私有局部变量
  ISR 与任务共享 ring buffer       仅调线程安全的外设库
```

**本项目判断**：
- W5500：**独占 SPI1**（OLED 是软件 I2C，CAN 是 bxCAN）→ ❌ 不需要临界区
- FIFO 操作：多任务共享 → ✅ 用 `xSemaphoreTake` 互斥锁，非临界区
- Modbus 寄存器：单写单读不同时 → ❌ 不需要临界区

### 10.2 正确互斥选择

| 场景 | 保护机制 |
|------|---------|
| 多任务共享外设 | **互斥锁**（`xSemaphoreCreateMutex`） |
| ISR 与任务共享数据 | **信号量 + BASEPRI 保护** |
| 极短（<1μs）读-改-写 | **BASEPRI 临界区** |
| **耗时外设操作（SPI 批量传输等）** | **互斥锁，绝不用临界区** |

### 10.3 临界区内绝对禁止

- DWT 延时（`Delay_us`）
- I2C/SPI 批量传输
- 任何可能阻塞的操作（`xSemaphoreTake`, `vTaskDelay`）

临界区内 BASEPRI=0x50 屏蔽 SysTick（优先级 15=0xF0），长时间停留导致：
1. Tick 不增长 → 任务延迟不触发
2. PendSV 被屏蔽 → 无法上下文切换
3. 低优先级任务（如 Housekeep）被饿死
4. IWDG 超时 → 系统复位

---

## 11. xSemaphoreGiveFromISR 与任务上下文的隔离

**绝对规则**：
- ISR 上下文 → `xSemaphoreGiveFromISR`, `xQueueSendFromISR`
- 任务上下文 → `xSemaphoreGive`, `xQueueSend`

`FromISR` 函数操作调度器事件链表（`xPendingReadyList`、就绪优先级位图），与 SysTick 中的 `xTaskIncrementTick` 存在竞态。在任务上下文中调用 `FromISR` 破坏链表 → HardFault 或静默挂死。

**检查**：`CAN_ProcessFrame` 运行在 `Task_CAN_Rx`（任务上下文），必须用 `xSemaphoreGive` 而非 `xSemaphoreGiveFromISR`。

---

## 12. SPI 多任务竞争

**SPI 外设应当只有一个"所有者"任务**。其他任务需要状态时，通过缓存读取：

```c
// W5500.c — SPI 操作仅在 W5500 任务中
uint8_t W5500_IsLinkUpCached(void) { return g_phy_linked; }  // 无 SPI，读缓存

// ETH_StateStr — 用缓存版
if (!W5500_IsLinkUpCached()) return "NOLK";  // 不调 W5500_LinkUp()
```

---

## 13. 调度器启动前信号量清空

以下代码必须放在 `vTaskStartScheduler()` 之前：

```c
{
    BaseType_t sem;
    do { sem = xSemaphoreTake(g_can_monitor_sem, 0); } while (sem == pdPASS);
    do { sem = xSemaphoreTake(g_can_rx_sem, 0); } while (sem == pdPASS);
    do { sem = xSemaphoreTake(g_can_fifo_not_empty, 0); } while (sem == pdPASS);
}
```

CAN 外设在 main() 中被初始化，CAN ISR 可能在 `vTaskStartScheduler()` 之前触发。若 ISR 已置位信号量，调度器启动后等待该信号量的任务会立即就绪，可能 **在 Idle 任务上下文完全建立前** 被 PendSV 切换执行。

---

## 14. xSemaphoreTake 与 portMAX_DELAY

**当前标准**：用 `pdMS_TO_TICKS(0x7FFFFFFF)`（≈24 天）代替 `portMAX_DELAY`：

```c
xSemaphoreTake(g_can_rx_sem, pdMS_TO_TICKS(0x7FFFFFFF));   // ✅ 当前方案
// xSemaphoreTake(g_can_rx_sem, portMAX_DELAY);             // ❌ 触发 xSuspendedTaskList bug
```

`portMAX_DELAY` 走挂起列表路径（`vListInsertEnd(&xSuspendedTaskList, ...)`），实测插入后系统静默卡死（无 HardFault）。有限超时走延迟列表路径完全正常。两者在 24 天级别上等价。

> 此标记为已知未解问题。若后续恢复 `portMAX_DELAY` 需先验证 `xSuspendedTaskList` 路径。

---

## 15. 验证清单（任何 FreeRTOS 配置修改后）

- [ ] `configASSERT_DEFINED=1` 下无 assert 触发
- [ ] 所有任务栈水位 > 30 words
- [ ] Heap 剩余稳定不下降
- [ ] Heartbeat 持续递增（证明 tick 正常）
- [ ] IWDG 不触发（证明系统不卡死）
- [ ] 5 分钟以上长时间运行无异常

---

## 16. 常见编译/链接错误

| 错误 | 根因 | 修复 |
|------|------|------|
| `struct has no field "ulNotifiedValue"` | task.h 顶部简化版 TCB 缺少完整字段 | 改为前向声明 |
| `struct has no field "ucQueueTypeInternal"` | QueueDefinition 缺少类型字段 | queue.h 末尾追加 |
| `A1586E: Bad operand types` | `configMAX_SYSCALL_INTERRUPT_PRIORITY` 含 `UL` 后缀 | 改为纯十六进制 `0x50` |
| `A1516E: Bad symbol` | `__asm` 块缺少 `extern` 声明 | 汇编块内显式 `extern` |
| `L6200E: Symbol multiply defined` | 宏映射与显式定义冲突 | 勿映射 SysTick |
| `L6406E: No space` | `configTOTAL_HEAP_SIZE` > 可用 RAM | 减小堆或查看 .map |

---

## 17. 参考文件

| 文件 | 用途 |
|------|------|
| `User/FreeRTOSConfig.h` | 所有 config 宏定义 |
| `FreeRTOS/inc/FreeRTOS.h` | 聚合 include |
| `FreeRTOS/inc/task.h` | TCB_t + 任务 API |
| `FreeRTOS/inc/queue.h` | QueueDefinition + 队列 API |
| `FreeRTOS/inc/portmacro.h` | BASEPRI 内联汇编 |
| `FreeRTOS/src/port.c` | SVC/PendSV/SysTick + 临界区 |
| `FreeRTOS/src/tasks.c` | 调度核心（含 3 处补丁 + 2 处修复） |
| `FreeRTOS/src/heap_4.c` | 堆管理 |
| `Start/startup_stm32f10x_md.s` | 向量表 + MSP 栈定义 |
| `System/Delay.c` | SysTick_Handler 中转 |
| [阶段二遇到的问题及解决](阶段二遇到的问题及解决.md) | 全部调试记录 |
