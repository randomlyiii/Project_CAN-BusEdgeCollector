# 阶段二-1：主站 FreeRTOS 独立调试（W5500/CAN 屏蔽）

> 状态: ✅ 已完成 — 2026-07-24
> 父阶段: [阶段二](阶段二.md)（FreeRTOS实时内核 + 双FIFO缓冲 + 工业级可靠性增强）
> 关联文档: [FreeRTOS 内核规范](freertos_standard.md) · [阶段二编译修复](phase2-freeertos-build.md)

---

## 子阶段目标

将主站 FreeRTOS 内核独立出来，屏蔽 W5500 以太网和 CAN 总线模块，在 OLED 上验证调度器运行、任务栈水位、动态堆剩余等 FreeRTOS 核心指标，确保内核配置完全正确后再恢复 W5500/CAN。

**为什么需要这个子阶段**：
1. W5500/CAN 硬件依赖复杂，问题混杂时难以定位是 FreeRTOS 配置错误还是外设驱动 bug
2. FreeRTOSConfig.h 存在多个隐患（废弃 API、中断校验关闭、非标准宏），需要先修正
3. 阶段二原计划直接 6 任务上线，缺乏 "Hello World" 级渐进验证环节

---

## 一、修改文件清单

| 文件 | 操作 | 关键变更 |
|------|------|---------|
| `CanEdgeGateWay_Master/User/FreeRTOSConfig.h` | 修正 | 11 处修改（见下表） |
| `CanEdgeGateWay_Master/User/main.c` | 屏蔽 + 新增 | W5500/CAN `#if 0` 屏蔽，3 个调试任务 |

核心原则：**只注释，不删除**。所有原始代码用 `#if 0 ... #endif` 包裹，恢复只需改 `#if 1`。

---

## 二、FreeRTOSConfig.h 修正清单

### 2.1 修改对照表

| # | 配置项 | 改前 | 改后 | 原因 |
|---|--------|------|------|------|
| 1 | `configPRIO_BITS` | `__NVIC_PRIO_BITS` | `4` | 避免依赖 CMSIS 头文件包含顺序（`FreeRTOS.h` 在 `stm32f10x.h` 之前引入时该宏未定义） |
| 2 | `configKERNEL_INTERRUPT_PRIORITY` | `(15 << 4)` | `(15UL << (8 - configPRIO_BITS))` | 语义等价（均为 0xF0），但公式形式表达了 "优先级 15 左移后写入 NVIC 8-bit 寄存器" 的意图 |
| 3 | `configMAX_SYSCALL_INTERRUPT_PRIORITY` | `(5 << 4)` | `(5UL << (8 - configPRIO_BITS))` | 同上，结果均为 0x50 |
| 4 | `configLIBRARY_LOWEST_INTERRUPT_PRIORITY` | `0x0F` | `15` | 十进制更直观，值不变 |
| 5 | `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | `0x05` | `5` | 同上 |
| 6 | `configASSERT` | `taskDISABLE_INTERRUPTS()` | `portDISABLE_INTERRUPTS()` | FreeRTOS V10.x 已废弃 `taskDISABLE_INTERRUPTS`，推荐用 `port` 前缀版本 |
| 7 | `configASSERT_DEFINED` | `0` | `1` | **关键修复**：开启 `xPortStartScheduler()` 中的 NVIC 优先级位数自检 + 运行时 `portASSERT_IF_INTERRUPT_PRIORITY_INVALID()` 校验。之前关闭导致所有中断优先级错误静默通过 |
| 8 | `configSUPPORT_DYNAMIC_ALLOCATION` | 未定义 | `1` | 显式声明，不依赖内核默认值（默认也是 1） |
| 9 | `configSUPPORT_STATIC_ALLOCATION` | 未定义 | `0` | 显式声明 |
| 10 | `configKERNEL_YIELD_PRIORITY` | 存在 | **删除** | 非标准宏，FreeRTOS 内核不读取此宏（内核用 `configKERNEL_INTERRUPT_PRIORITY`） |
| 11 | `configMAX_PRIORITIES_MASK` | 存在 | **删除** | 启用了 `configUSE_PORT_OPTIMISED_TASK_SELECTION = 1`（portmacro.h 默认），走 CLZ 指令路径，不使用此掩码 |
| 12 | `FREERTOS_KERNEL_PATH` / `FREERTOS_KERNEL_INC_PATH` | 存在 | **删除** | 死代码，整个工程中无引用 |
| 13 | 中断优先级速查注释 | 无 | **新增** | 顶部大注释，记录 NVIC 分组/优先级映射/BASEPRI 阈值 |

### 2.2 中断优先级验证（未改变，仅文档化确认）

| 宏 | 值 | 含义 | 结论 |
|---|---|---|---|
| `configKERNEL_INTERRUPT_PRIORITY` | 0xF0 | PendSV(#14) / SysTick(#15) 设为最低优先级 15 | ✅ |
| `configMAX_SYSCALL_INTERRUPT_PRIORITY` | 0x50 | 临界区 BASEPRI 阈值，屏蔽优先级 ≥ 0x50 的中断 | ✅ |
| `NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)` | main.c 首行 | 4 位全抢占，无子优先级（BASEPRI 机制的前提条件） | ✅ |
| STM32F103 NVIC 优先级位数 | 4 | 有效优先级 0-15，寄存器值 = 优先级 << 4 | ✅ |

**结论**：中断优先级配置从数学上是正确的。`configASSERT_DEFINED = 1` 会在运行时通过硬件自检验证。

---

## 三、main.c 变更详情

### 3.1 屏蔽部分（#if 0 包裹）

```c
/* 头文件 */
#if 0
#include "CAN_User.h"
#include "W5500.h"
#include "ModbusTCP.h"
#include "fifo.h"
#endif

/* 硬件初始化 */
#if 0
    CAN_User_Init();
#endif

#if 0
    ret = W5500_Init();
    // ... 完整 W5500 错误检查 ...
    ret = W5500_ConfigNetwork();
    ModbusTCP_Init();
    g_eth_was_connected = 0;
#endif

/* 6 个原始任务函数 */
#if 0
static void Task_CAN_Rx(...)     { ... }
static void Task_CAN_Monitor(...){ ... }
static void Task_CAN_Tx(...)     { ... }
static void Task_Protocol(...)   { ... }
static void Task_W5500(...)      { ... }
static void Task_Housekeep(...)  { ... }
#endif
```

### 3.2 新增 3 个调试任务

| 任务函数 | 优先级 | 栈深度 | 周期 | OLED 显示内容 |
|---------|--------|--------|------|--------------|
| `Task_DbgHeartbeat` | 4 | 200 | 200ms | 不直接刷新 OLED，仅证明调度器有任务在跑 |
| `Task_DbgOLED` | 3 | 320 | 500ms | 运行时间(tick→s)、堆剩余(KB)、各任务栈高水位 |
| `Task_DbgHousekeep` | 0 | 256 | 1s | IWDG 独立看门狗喂狗 |

### 3.3 OLED 调试显示布局（4×16 字符）

```
Line 1: "M:RTOS  12345s"    // 系统运行秒数
Line 2: "Heap:8192/12288"   // 堆剩余 / 总堆大小
Line 3: "R  0 M  0 T  0"    // 原始 CAN_Rx/Monitor/Tx 栈水位 (句柄=NULL 时显示0)
Line 4: "P  0 W  0 H256"    // 原始 Protocol/W5500 栈水位 + DbgHousekeep 栈水位
```

> 注：Line 3/4 中原始任务句柄为 NULL（任务未创建），`OLED_DebugDisplay()` 通过三元运算符安全返回 0。DbgHousekeep 通过 `hTask_Housekeep` 句柄可读到真实栈水位。

### 3.4 同时修复的非 FreeRTOS 问题

| 位置 | 改前 | 改后 | 原因 |
|------|------|------|------|
| `vApplicationStackOverflowHook` | `taskDISABLE_INTERRUPTS()` | `portDISABLE_INTERRUPTS()` | 废弃 API |
| `vApplicationMallocFailedHook` | `taskDISABLE_INTERRUPTS()` | `portDISABLE_INTERRUPTS()` | 同上 |

---

## 四、编译/烧录/验证步骤

### 4.1 编译

在 Keil MDK-ARM 中打开 `CanEdgeGateWay_Master/ceg_master.uvprojx`，编译。预期：

```
Build target 'ceg_master'
0 Error(s), 0 Warning(s)
```

若 `configASSERT_DEFINED = 1` 触发新增 error，检查：
- `stm32f10x.h` 中 `__NVIC_PRIO_BITS` 是否确为 4（若不存在则 `configPRIO_BITS = 4` 已兜底）
- `portmacro.h` 中 `vPortValidateInterruptPriority` 是否与 ARMCC 汇编语法兼容

### 4.2 烧录

ST-Link 连接 Blue Pill 板（仅接 SWDIO/SWCLK/GND，不接 3.3V），BOOT0=0，下载后复位。

### 4.3 OLED 验证清单

| 序号 | 观察项 | 预期现象 | 异常指示 |
|------|--------|---------|---------|
| 1 | 上电 | OLED 短暂显示 `M: Init...` → `CAN: OFF` → `ETH: OFF` → `M: Starting...` | 若卡在 `M: Init...` → OLED I2C 故障 |
| 2 | 调度器启动 | OLED 显示 `M:RTOS    0s` 然后秒数递增 | 若显示 `SCHED FAILED!` → `configTOTAL_HEAP_SIZE` 不足或启动检查失败 |
| 3 | 秒数累加 | `M:RTOS` 行每 1s 更新，秒数单调递增 | 若不动 → SysTick 未正常触发 (`Delay.c` 的 `SysTick_Handler` 未调用 `xPortSysTickHandler`) |
| 4 | 堆剩余 | `Heap:XXXX/12288`，初始值约 9500-10500 | 若 < 1000 → 任务栈总和过大，堆即将耗尽 |
| 5 | 栈水位 | Line 4 最后一个数字为 `Task_DbgHousekeep` 栈剩余（word），初始约 180-220 | 若 < 30 → 栈不足，增大 `STACK_DBG_HK` |
| 6 | 长时间运行 | 5 分钟后秒数正常累加，堆剩余稳定，IWDG 不触发 | 若秒数跳跃/停止 → 栈溢出或内存踩踏 |
| 7 | `configASSERT_DEFINED = 1` | 启动阶段无 assert 触发 → 正常运行 | 若 assert → NVIC 优先级位数与 `configPRIO_BITS=4` 不匹配 |

### 4.4 调试断点验证（可选）

在 Keil Debug 中设置断点：
- `xPortSysTickHandler` → 每 1ms 触发一次（证明 SysTick 正常）
- `Task_DbgOLED` 内 `OLED_DebugDisplay()` → 每 500ms 触发
- `Task_DbgHousekeep` 内 `IWDG_ReloadCounter()` → 每 1s 触发

---

## 五、恢复 W5500/CAN 的步骤

将以下 `#if 0` 改为 `#if 1`：

| 位置 | #if 位置 | 恢复内容 |
|------|---------|---------|
| main.c 第18行 | `#if 0 /* W5500/CAN 调试屏蔽: 头文件 */` | 恢复 `#include "CAN_User.h"` 等 4 行 |
| main.c 第60行 | `#if 0 /* ---- 原始 OLED ---- */` | 恢复 `CAN_StateStr/ETH_StateStr/OLED_UpdateDisplay` |
| main.c 第170行 | `#if 0 /* 原始任务函数 */` | 恢复 6 个任务函数 |
| main.c 第426行 | `#if 0 /* ==== CAN 初始化 ==== */` | 恢复 `CAN_User_Init()` |
| main.c 第433行 | `#if 0 /* ==== W5500 初始化 ==== */` | 恢复 `W5500_Init/ConfigNetwork/ModbusTCP_Init` |

然后将调试任务的 `xTaskCreate` 调用替换回原始 6 个 `xTaskCreate`，删除 3 个 `Task_Dbg*` 函数定义。

---

## 六、FreeRTOSConfig.h 深度排查记录

本节是此前整个 FreeRTOSConfig.h 排查的完整输出，供 DeepSeek 等参考。

### 6.1 已验证正确的项

| 检查项 | 值 | 判断 |
|--------|-----|------|
| `configCPU_CLOCK_HZ` | 72000000 | ✅ 72MHz |
| `configTICK_RATE_HZ` | 1000 | ✅ 1ms tick |
| `configMAX_PRIORITIES` | 8 | ✅ 0-7, 数值越大优先级越高 |
| `configMINIMAL_STACK_SIZE` | 128 | ✅ Idle 任务栈 128 words |
| `configTOTAL_HEAP_SIZE` | 12KB (Master) | ✅ heap_4.c |
| `configUSE_PREEMPTION` | 1 | ✅ 抢占式 |
| `configUSE_PORT_OPTIMISED_TASK_SELECTION` | 1 (默认) | ✅ CLZ 指令, 要求 configMAX_PRIORITIES ≤ 32 |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | ✅ 方法 2: 栈顶魔数检测 |
| `configUSE_TIMERS` | 1 | ✅ 软件定时器任务优先级 = configMAX_PRIORITIES-1 |
| `configUSE_MUTEXES / RECURSIVE_MUTEXES / COUNTING_SEMAPHORES` | 1 | ✅ 全部启用 |
| `configTASK_NOTIFICATION_ARRAY_ENTRIES` | 3 | ✅ 每任务 3 个通知槽 |
| `NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)` | main.c 首行 | ✅ |

### 6.2 需要后续关注的项

| 关注项 | 当前状态 | 建议 |
|--------|---------|------|
| 启动文件 MSP 栈 | `Stack_Size EQU 0x200` (512B) | 6 任务 + 多层 ISR 嵌套时偏小，建议后续改为 0x400 (1024B) |
| 启动文件 Heap | `Heap_Size EQU 0` | ✅ 正确，FreeRTOS 用 `pvPortMalloc` 而非标准库 `malloc` |
| Delay.c SysTick_Handler | 自行调用 `xPortSysTickHandler()` + 调度器状态检查 | ✅ 正确，调度器启动前不调用内核 tick 处理 |
| 复用 DWT 做微秒延时 | `Delay_DWT_Init()` → `DWT->CYCCNT` | ✅ 正确，不与 SysTick 冲突 |
| `configTIMER_TASK_STACK_DEPTH` | 128 | 可考虑缩减至 80-100（定时器回调简单时） |

### 6.3 STM32F103C8T6 + FreeRTOS 易踩坑速查

| # | 坑点 | 本项目状态 |
|---|------|----------|
| 1 | **SysTick 冲突**：标准库 Delay 与 FreeRTOS 抢 SysTick | ✅ 已规避（微秒用 DWT，SysTick 由 port.c 配置） |
| 2 | **中断分组未设 Group_4**：默认 Group_0 导致 BASEPRI 失效 | ✅ main.c 首行已设 |
| 3 | **ISR 优先级过高调 FromISR API**：`configASSERT_DEFINED=0` 时不报错 | ✅ 已改为 1（开发阶段） |
| 4 | **启动文件 MSP 栈偏小**：ISR 嵌套 + FreeRTOS 上下文切换帧 | ⚠️ 当前 512B，建议后续改 1024B |
| 5 | **`taskENTER_CRITICAL` 包裹外设 I/O**：长时间关中断导致 SysTick 丢失 | ⚠️ 原始 Task_W5500 存在此问题（恢复 W5500 后需用互斥锁替代） |
| 6 | **动态内存碎片化**：反复 `pvPortMalloc/pvPortFree` 不同大小块 | ✅ 本项目启动时一次性创建所有任务，运行时不分配/释放 |
| 7 | **任务函数 return**：任务函数返回到 `prvTaskExitError` 触发 assert | ✅ 本项目所有任务均为 `for(;;)` 无限循环 |
| 8 | **heap_4.c 堆数组在 .bss**：`ucHeap[configTOTAL_HEAP_SIZE]` 占用 RAM | ✅ 12KB 堆 + 其他静态数据 < 20KB SRAM |
| 9 | **`configASSERT` 用废弃 API**：`taskDISABLE_INTERRUPTS` 在 V10.x 废弃 | ✅ 已改为 `portDISABLE_INTERRUPTS` |
| 10 | **`configMAX_SYSCALL_INTERRUPT_PRIORITY = 0`**：port.c 编译报错 | ✅ 本项设为 0x50 ≠ 0 |

---

## 七、与阶段二主线的衔接

```
阶段二-1 (本次)              阶段二-2 (后续)
─────────────────────────────────────────────────
FreeRTOSConfig.h 修正        W5500 初始化恢复
3 个调试任务验证调度器        CAN 初始化恢复
OLED 显示 RTOS 运行指标       6 个原始任务恢复
configASSERT_DEFINED = 1     双 FIFO + 信号量创建
                              升降级/限流/错误处置上线
```

子阶段完成标志：**主站 FreeRTOS 在无 W5500/CAN 条件下编译 0 Error、OLED 验证 7 项通过、5 分钟以上无 IWDG 复位**。

---

## 八、参考

- `CanEdgeGateWay_Master/User/FreeRTOSConfig.h` — 修正后的完整配置
- `CanEdgeGateWay_Master/User/main.c` — 调试版（W5500/CAN `#if 0` 屏蔽）
- `CanEdgeGateWay_Master/System/Delay.c` — DWT 微秒延时 + SysTick_Handler
- `CanEdgeGateWay_Master/FreeRTOS/src/port.c` — Cortex-M3 BASEPRI 临界区移植
- `CanEdgeGateWay_Master/Start/startup_stm32f10x_md.s` — 向量表 + MSP 栈定义
- [FreeRTOS 内核规范](freertos_standard.md) — 本项目的 FreeRTOS 非标准约定
- [阶段二](阶段二.md) — 父阶段完整设计文档
