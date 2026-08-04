# CAN 通信规范 — CAN Bus Edge Collector

> 适用范围: 主站 + 从站 CAN 通信层  
> 最后更新: 2026-08-04 (第十次修改: FMP0 ISR + 紧凑 ring64 + 独立 drain 任务; 负载折算 108→122)

---

## 1. CAN 物理层

| 项目 | 参数 |
|------|------|
| 波特率 | **500kbps** |
| 采样点 | **75%** — SJW=2tq, BS1=8tq, BS2=3tq, Prescaler=6 (36MHz/6/12tq=500kbps) |
| 收发器 | TJA1050 / ISO1050 |
| 总线拓扑 | **并联**（3 节点: 1 主站 + 2 从站） |
| 终端电阻 | 总线两端各 120Ω（CAN_H 与 CAN_L 之间） |

> **采样点说明**: 原始阶段二 50% (BS1=5, BS2=6) 在 2 节点下频繁 CRC 错。75% 降低误码率。

---

## 2. CAN 帧协议

### 2.1 帧格式（8 字节标准帧）

```
[Byte0=优先级][Byte1=源节点ID][Byte2=功能码][Byte3-6=载荷][Byte7=校验和]
```

- 校验和 = Byte0~Byte6 异或
- 全部为标准帧 (StdId, 11-bit ID)

### 2.2 帧 ID 分配（3 级优先级）

| 优先级 | ID 范围 | 用途 |
|--------|---------|------|
| **0 级 (紧急)** | 0x100 ~ 0x1FF | 报警/故障/BusOff 恢复 |
| **1 级 (常态)** | 0x200 ~ 0x2FF | 心跳/温湿度 |
| **2 级 (低频)** | 0x300 ~ 0x3FF | 光照/LM393/预留通道 |

### 2.3 功能码定义

| 功能码 | 宏名 | 数据来源 | 说明 |
|--------|------|---------|------|
| 0x01 | `CAN_FUNC_HEARTBEAT` | — | 心跳帧 |
| 0x02 | `CAN_FUNC_TEMP_HUMI` | DHT11 | 温湿度 |
| 0x03 | `CAN_FUNC_ALARM` | — | 报警 |
| 0x04 | `CAN_FUNC_RECOVER` | — | 故障恢复 |
| 0x05 | `CAN_FUNC_LIGHT` | BH1750 / LM393 AO | 光照（双用途） |
| 0x06 | `CAN_FUNC_LM393_DO` | LM393 DO | LM393 数字输出（最终阶段新增） |
| 0x07 | `CAN_FUNC_RESERVED` | 预留通道 | 预留通道（最终阶段新增） |
| 0xF0 | `CAN_FUNC_BUSOFF_RECOVERY` | — | Bus-Off 恢复通知 |

### 2.4 载荷格式（Byte3-6）

| 传感器类型 | Byte3 | Byte4 | Byte5 | Byte6 |
|-----------|-------|-------|-------|-------|
| DHT11 (0x02) | temp_int | temp_dec | humi_int | humi_dec |
| BH1750 (0x05) | lux_hi | lux_lo | 0 | 0 |
| LM393_AO (0x05) | analog_hi | analog_lo | digital | 0 |
| LM393_DO (0x06) | digital | 0 | 0 | 0 |
| RESERVED (0x07) | ch1_hi | ch1_lo | 0 | 0 |
| Heartbeat (0x01) | node_id | 0 | 0 | 0 |
| ALARM (0x03) | node_id | 0x01 | 0 | 0 |
| RECOVER (0x04) | node_id | 0 | 0 | 0 |

---

## 3. 主站 CAN 架构

### 3.1 任务架构（三任务解耦）

```
Task_CAN_Drain (prio 3, 10ms 固定周期):      ← CAN 帧处理独立任务
  1. drain ring_high (FMP0 ISR 填充, 64 深紧凑帧) → CAN_ProcessFrame
  2. drain ring_norm (FIFO1 fallback, 4 深)
  (g_can_ready 门闸: CAN/FIFO 初始化完成前只清 ring 不处理)

Task_Unified (prio 1, 10ms 固定周期):
  1. CAN 监控: HeartBeatCheck / ErrorMonitor / CalcBusLoad / CheckEscalation / CheckDeescalation
  2. W5500 + ModbusTCP (顺序执行, 不与 CAN 并发)
  3. ModbusTCP_SyncFromCAN (FIFO → 历史缓存)
  4. 按键扫描 (PA0 翻页) + IWDG 喂狗

Task_Housekeep (prio 0, 1s): OLED 刷新 + 栈水位
```

> **为何拆分 drain 任务**: W5500/Modbus 的 SocketCmd/SendData 可阻塞 100~700ms
> (实测 T:700)。若 CAN drain 留在 Task_Unified, 阻塞期 ring(64 深, ~14ms 容量)
> 必然溢出丢帧 (实测 F:OV)。Task_CAN_Drain (prio 3) 抢占 Task_Unified, 每 10ms
> 强制 drain → ring 占用恒 ≤ 50 帧, 5000fps 实测零丢帧。
>
> **关键约束**: Task_CAN_Drain 只读内存 ring (不碰 CAN 硬件, 不碰 SPI), 不产生
> SPI 边沿 → 物理上与顺序架构等价。SPI (W5500) 与 CAN 共用 GPIOA
> (PA5=SCK, PA11=CAN_RX), 并发时 SPI 时钟干扰 CAN RX 采样 — 该约束由
> Task_Unified 内部顺序执行满足。
>
> **CAN Tx**: 主站只接收, 不主动发帧 (无 ACK 时硬件重传推 TEC→255 BusOff)。
> 从站心跳由 CAN 控制器硬件自动应答, 无需主站参与。

### 3.2 CAN 接收架构（FMP0 中断 + 紧凑环形缓冲 + 独立 drain 任务）

主接收路径: 硬件 FIFO0 → FMP0 中断 ISR (`USB_LP_CAN1_RX0_IRQHandler`, prio 8)
→ ring_high (64 深紧凑帧) → Task_CAN_Drain → CAN_ProcessFrame。

| FIFO | 接收方式 | 缓冲区 | 说明 |
|------|---------|--------|------|
| **FIFO0** | **FMP0 ISR** → ring_high | ring_high (64 深, RingFrame_t 12B) | 主路径, 全通滤波所有帧 |
| **FIFO1** | ISR → ring_norm (FMP1 禁用) | ring_norm (4 深) | fallback, 正常无帧 |

**紧凑帧 RingFrame_t (12B, 原 CanRxMsg 20B)** — RAM 约束下 64 深必须紧凑:

```c
typedef struct {
    uint16_t StdId;   /* 协议只用标准帧 StdId≤0x3FF */
    uint8_t  DLC;
    uint8_t  Data[8];
} RingFrame_t;   /* sizeof = 12 (对齐 2) */
/* 去 ExtId/IDE/RTR/FMI — 协议无信息量。64 深 CanRxMsg 会 L6406E RAM 溢出。
 * ring_high 64 + ring_norm 4 = 824B (map 实测), 全工程 RAM 余 896B。 */
```

**FMP0 ISR (只搬运不解析, 无锁 SPSC push)**:

```c
void USB_LP_CAN1_RX0_IRQHandler(void) {
    /* ⚠️ F103 md 启动文件向量名是 USB_LP_CAN1_RX0, 不是 CAN1_RX0! */
    while (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CanRxMsg msg; RingFrame_t rf;
        CAN_Receive(CAN1, CAN_FIFO0, &msg);
        rf.StdId = (uint16_t)msg.StdId; rf.DLC = msg.DLC;
        memcpy(rf.Data, msg.Data, 8);
        RxRingHigh_push(&g_rx_ring_high, &rf);   /* 锁无关 SPSC, 无 RTOS API */
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}
```

**Task_CAN_Drain drain (任务只读软件 ring, 不碰硬件 FIFO)**:

```c
RingFrame_t rf;
while (RxRingHigh_pop(&g_rx_ring_high, &rf) == 0) {
    g_isr_rx_frame.StdId = rf.StdId; g_isr_rx_frame.DLC = rf.DLC;
    memcpy(g_isr_rx_frame.Data, rf.Data, 8);
    CAN_ProcessFrame(&g_isr_rx_frame);
}
```

> **向量名陷阱**: F103 启动文件把 IRQ20 映射到 `USB_LP_CAN1_RX0_IRQHandler` (weak)。
> 若代码定义 `CAN1_RX0_IRQHandler`, linker 当未用符号删除, 向量指向 weak 死循环
> → FMP0 一开首帧即挂死 (启动屏卡死)。RX1/SCE 名字正常, 只有 RX0 是 USB 混合名。
> 验收必须查 map 确认 136B 真实代码进向量。

### 3.3 CAN ISR 优先级

| ISR | 优先级 | 说明 |
|-----|--------|------|
| USB_LP_CAN1_RX0_IRQHandler (FIFO0) | **8** | FMP0 中断, 只 CAN_Receive→ring_push, 无 RTOS API |
| CAN1_RX1_IRQHandler (FIFO1) | **8** | FMP1 禁用, 正常不触发 (ring_norm fallback) |
| CAN1_SCE_IRQHandler | **6** | ERR/BOF 中断禁用, 不执行 |

> **注意**: 优先级 8 ≥ configMAX_SYSCALL (0x50), 会被 FreeRTOS 临界区 (BASEPRI) 屏蔽,
> 因此 ISR 内不调用任何 FreeRTOS API (无 xSemaphoreGiveFromISR、无 portYIELD_FROM_ISR)。

### 3.4 CAN 硬件滤波器

当前使用**全通滤波器**（所有帧走 FIFO0），简化中断优先级管理：

```c
/* Filter 0: 全通 (IdMask=0x00000000), 所有帧 → FIFO0 */
filter.CAN_FilterNumber           = 0;
filter.CAN_FilterMode             = CAN_FilterMode_IdMask;
filter.CAN_FilterScale            = CAN_FilterScale_32bit;
filter.CAN_FilterIdHigh           = 0x0000;
filter.CAN_FilterIdLow            = 0x0000;
filter.CAN_FilterMaskIdHigh       = 0x0000;   /* 全 0 = 全部接受 */
filter.CAN_FilterMaskIdLow        = 0x0000;
filter.CAN_FilterFIFOAssignment   = CAN_FIFO0;
```

> 之前使用双滤波器（FIFO0=紧急 0x100-0x1FF, FIFO1=常态 0x200-0x3FF）配合双 ISR。
> 现用全通滤波器 + FMP0 ISR (prio 8) → ring_high。FIFO1 中断保留为 fallback
> （CAN1_RX1_IRQHandler, FMP1 禁用, 只做 ring_push）。

### 3.5 CAN_ProcessFrame（Task 上下文调用）— 精简版

```c
CAN_ProcessFrame:
  1. DLC 校验 (!=8 则丢弃)
  2. Checksum 校验 (XOR byte0~6 != byte7 则丢弃)
  3. g_can_rx_int_count++
  4. 查找/注册从站节点
  4. 收到任何帧 → online=1, stale=0, offline=0, expire_start_tick=0
     → timestamp=now, g_boff_consec=0（清零连续 BusOff 计数）
  5. 按 func 分支:
     - HEARTBEAT → heartbeat_count++
     - TEMP_HUMI → 温湿度数据
     - ALARM → fault_flag=1
     - RECOVER → fault_flag=0
     - LIGHT/LM393_DO/RESERVED → 传感器数据
  6. 推入 FIFO (按优先级分流, 主站 receive-only 无自接收回环)
  
  注: 所有 frame 都在步骤 4 统一设 online/stale/offline, FIFO 数据由 ModbusTCP_SyncFromCAN 消费 -> 历史缓存 -> 断网恢复后批量上传.
      各 case 只处理具体数据字段, 不再单独设 flag.
```

### 3.6 主站 CAN 错误处理

| 机制 | 说明 |
|------|------|
| HeartBeatCheck | 每节点独立: >3s 无帧→stale=1; >30s 持续 stale→offline=1。BusOff/Passive 时跳过 (error_level>=2) |
| ErrorMonitor | ABOM=DISABLE, INRQ 协议恢复: 200ms 间隔, 连续 5 次 BusOff 后拉长到 3s。g_boff_consec 边沿检测防重复累加 |
| CalcBusLoad | 100ms 窗口滑动计算 → 负载 >70% 限速 / >90% 紧急模式 |
| CAN_SendFrame | 1ms 超时轮询 → 失败返回 1 |

> **负载折算 (CAN_FRAME_BITS=122)**: 8 字节标准帧实际 ~122 位 (0x55 低填充实测,
> 500k/4100fps 反推)。`负载% = 帧率 × 122 / 500000 × 100%`。
> 70% 阈值 = 2869fps, 90% 阈值 = 3689fps, 物理上限 500000/122 ≈ 4100fps = 100%。
> 原 108 位 (无填充最小值) 低估真实占用 — 5000fps 压测时显示 88.56% 实为总线已满。

#### HeartBeatCheck（三态标记, 每节点独立判定）

```c
void CAN_HeartBeatCheck(void) {
    /* 每节点独立判定. 去掉 error_level 门禁 — 避免总线错误波及全部节点.
     * 仅通过 checksum 校验的合法帧才能刷新节点时间戳. */
    for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == 0) continue;
        if (!g_slave_nodes[i].online) continue;

        uint32_t delta = now - g_slave_nodes[i].last_heartbeat_tick;
        if (delta > STALE_TIMEOUT_MS) {
            g_slave_nodes[i].stale = 1;
            if (g_slave_nodes[i].expire_start_tick == 0)
                g_slave_nodes[i].expire_start_tick = now;
            if (now - g_slave_nodes[i].expire_start_tick > OFFLINE_CONFIRM_MS)
                g_slave_nodes[i].offline = 1;
        } else {
            g_slave_nodes[i].stale = 0;
            g_slave_nodes[i].offline = 0;
            g_slave_nodes[i].expire_start_tick = 0;
        }
    }
}
```

> **状态变迁**: 收到合法帧(checksum通过)→全部清除 | >3s→EXP | >30s→OFF | ALARM 帧→ALM | 节点独立判定

#### ErrorMonitor（ABOM=DISABLE, INRQ 协议恢复）

```c
void CAN_ErrorMonitor(void) {
    // 读 ESR → 更新 error_level
    if (error_level == 3) {
        if (!boff_counted) {          // 边沿检测: 每次 BusOff 仅累加一次
            g_boff_consec++;
            boff_counted = 1;
        }
        uint32_t cd = (g_boff_consec > 5) ? 3000 : 200;
        if (now - last_recover >= cd) { CAN_ResetBus(); ... }
    } else {
        boff_counted = 0;
    }
    // CAN_ProcessFrame 收到帧时清零 g_boff_consec
}
```

#### CAN_ResetBus（INRQ 退出初始化模式）

不再使用 `CAN_DeInit + CAN_Hardware_Init`（软件全复位违反 CAN 协议）。

```c
void CAN_ResetBus(void) {
    CAN1->MCR |= CAN_MCR_INRQ;           // 请求初始化模式
    while (!(CAN1->MSR & CAN_MSR_INAK));  // 等 INAK 确认
    CAN1->MCR &= ~(uint32_t)CAN_MCR_INRQ; // 退出初始化模式
    while (CAN1->MSR & CAN_MSR_INAK);     // 等 INAK 清除
    while (CAN1->ESR & BOFF_BIT);         // 等待 128 隐性位检测完成
    // 时序寄存器保持, 无需重配
}
```

### 3.7 从站节点管理 (g_slave_nodes)

```c
typedef struct {
    uint8_t  node_id;
    uint8_t  online;             // 1=收到过帧(CAN_ProcessFrame 置, 永远不清0)
    uint8_t  stale;              // >3s 无帧=1, 短期过期标记
    uint8_t  offline;            // >30s 持续 EXP=1, 长期确认离线
    uint32_t last_heartbeat_tick;
    uint32_t expire_start_tick;  // 进入 EXP 瞬间的时间戳
    uint16_t heartbeat_count;
    uint8_t  fault_flag;         // ALARM 帧置位, RECOVER 清0
    // 无 blacklist / hb_lost / 防抖计数器 — 极简设计
    uint8_t  temp_int, temp_dec;
    uint8_t  humi_int, humi_dec;
    uint16_t light_lux;
    uint16_t lm393_analog;
    uint8_t  lm393_digital;
    uint16_t reserved_ch1;
} SlaveNode_t;
```

---

## 4. 从站 CAN 架构（零中断 · 纯轮询 · NART 隔离）

### 4.1 架构决策

从站经过反复调试后确定采用**零 CAN 中断**架构。原因详见 `engineReuse_standard.md`。

### 4.2 NART=ENABLE 权衡说明

从站 `CAN_NART = ENABLE`，这是经过单节点掉线→全网连坐故障后确认的设计决策。

#### 权衡点（工程留档）

**潜在代价**: 单次心跳丢失不可重试
- 极端干扰窗口内某一次心跳发送失败，需要等待 500ms 下一轮才能补发。
- 主站心跳超时阈值 1500ms（3 倍心跳周期），预留充足裕量，单次丢帧不会触发离线告警。

**与 ISO11898 标准的冲突说明**
- ISO11898 默认允许自动重传；但工业多节点、存在节点热插拔场景，NART=ENABLE 是公认的容错优化手段。
- 标准追求"尽力送达"；咱们的系统追求"**网络隔离，单一节点故障不拖累全网**"，优先级不同。

**ABOM 保留 ENABLE 的意义**
- 从站保留 ABOM=ENABLE 作为极端持续干扰下的兜底防护。
- 主站 ABOM=DISABLE（由 ErrorMonitor INRQ 协议恢复替代），避免 2 节点下震荡。

#### 故障链对比

| 状态 | NART=DISABLE（之前） | NART=ENABLE（现在） |
|------|-------------------|-------------------|
| 从站掉线→总线扰动 | 其他从站 TX 失败→硬件微秒级自动重传→TEC 32次冲到255→BusOff→ABOM 恢复→再失败→震荡循环 | 其他从站 TX 失败→TEC+8→放弃→500ms 后重试→总线已稳定→成功 |
| 从站2能否保住 | ❌ 几秒后掉线 | ✅ 持续在线 |
| 单帧丢失 | 硬件重传直到成功 | 需等 500ms 应用层重试 |

### 4.3 从站 CAN 初始化

```c
// ✅ 正确: 在 Task_CAN_Slave 首行调用（调度器启动后）
static void Task_CAN_Slave(void *pvParameters) {
    CAN_DeInit(CAN1);        // 复位 CAN 外设（清除 vTaskStartScheduler 的寄存器破坏）
    Delay_ms(10);            // 等待稳定
    CAN_User_Init();         // GPIO + CAN 外设 + 滤波器（无中断！无 NVIC！无信号量！）

    // 启动错峰: 按 node_id 延迟避免多从站同时上电冲突
    vTaskDelay(pdMS_TO_TICKS((slave_node_id - 1) * 2000));

    for (;;) { /* 收发循环 */ }
}
```

### 4.3 CAN_User_Init（从站无中断版）

```c
void CAN_User_Init(void) {
    CAN_GPIO_Init();    // PA11=IPU (RX), PA12=AF_PP (TX)
    CAN_Init(CAN1, &can);  // ↑ NART=ENABLE (见 4.2 权衡说明)
    CAN_FilterInit(&filter);   // 全通滤波器
    LocalCache_Init(&g_local_cache);
    // ⚠️ 无 CAN_ITConfig / 无 NVIC_Init / 无信号量创建
}
```

### 4.4 CAN 收发循环（纯轮询）

```c
for (;;) {
    // 1. 轮询接收: 从 FIFO0 清空帧（≤16 帧/周期，防饿死低优先级任务）
    uint8_t drain = 0;
    while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0 && drain < 16) {
        CAN_Receive(CAN1, CAN_FIFO0, &rx_msg);
        drain++;
    }

    // 2. 发送: 心跳 + 传感器数据
    CAN_SendHeartBeat();
    for (i = 0; i < sensor_count; i++)
        CAN_SendSensorData(s->type_id, &s->last_data);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
}
```

### 4.5 CAN 发送超时

```c
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len) {
    uint32_t timeout = 2000;  // 2ms 超时
    uint8_t mailbox = CAN_Transmit(CAN1, &tx_msg);
    while (timeout--) {
        if (CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Ok)
            return 0;  // 成功
        Delay_us(1);
    }
    return 1;  // 失败 → 写入本地缓存 LocalCache
}
```

---

## 5. SPI 超时防护（W5500）

> ⚠️ **关键约束**: 所有 `while (flag == RESET)` 循环必须有超时！

### 5.1 SPI 发送超时

```c
static uint8_t SPI_SendByte(uint8_t dat) {
    uint32_t to = 200;          // ≈17µs @72MHz，远超正常 SPI 传输 (~28ns)
    SPI_I2S_SendData(W5500_SPI, dat);
    while (--to) {
        if (SPI_I2S_GetFlagStatus(W5500_SPI, SPI_I2S_FLAG_TXE) != RESET)
            return 0;           // OK
    }
    g_chip_ok = 0;              // SPI 异常 → 标记 W5500 离线
    return 1;                   // 超时
}
```

### 5.2 超时后的行为

- `g_chip_ok = 0` → 后续 `W5500_TCPServer_Run()` 立即返回，不执行 SPI 操作
- `ETH_StateStr()` 读到 `g_chip_ok=0` → OLED 显示 `"ETH:FAIL"`
- 超时由 CAN ALARM 帧在 SPI 走线上的电磁干扰引起（瞬态，不影响后续通信）
- 当前无自动恢复逻辑（`g_chip_ok` 保持 0），W5500 通信中断后需重启

---

## 6. 已解决的问题汇总

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| 1 | 主站 CAN_ResetBus 死循环 | error_level 未清零 | `g_can_error.error_level = 0` 提前清 |
| 2 | ALARM 帧自接收回环 | STM32F1 CAN 自接收（收→推FIFO→发→自接收→循环） | 去 CAN_ProcessFrame 中的 FIFO push；V3.0 因主站 receive-only 安全恢复 FIFO_Push，不复现回环 |
| 3 | KEY1 冻结 4s→复位 | CAN1_RX0_IRQHandler 干扰调度 | FIFO0 改 Task 轮询 |
| 4 | SPI 死循环 | `while(TXE)` 无超时 | 加超时 + `g_chip_ok=0` |
| 5 | 2 节点下全部掉线 | 终端阻抗(40Ω→60Ω)改变导致信号完整性差; ABOM 2.8ms 自愈形成 BusOff 震荡 | ABOM=DISABLE + ErrorMonitor INRQ 协议恢复; 采样点 50%→75%; 收到帧时清零 boff_consec |
| 6 | BusOff 期间心跳假离线 | error_level>=2 时 HeartBeatCheck 仍判 stale/offline | 增 `if (error_level >= 2) return;` 保护退出 |
| 7 | OLED 假 ON（从未收帧也显示 ON） | `OLED_NodeStatusStr` 未检查 `online` | 增 `!online→"--"` 6 态显示 |
| 8 | hb_lost/blacklist 复杂耦合 | 超时/黑名单/防抖多重状态联动 | 精简为 stale/offline 两段标记 + CAN_ProcessFrame 统一置位 |
| 9 | CAN_ResetBus 暴力软件复位 | `CAN_DeInit + Init` 违反 CAN 协议, 时序寄存器全丢 | 改 INRQ 进入/退出初始化模式, 硬件走 128 隐性位恢复 |
| 10 | g_boff_consec 10ms 周期重复累加 | ErrorMonitor 10ms 轮询, 一次 BusOff 等冷却期间被加到 20+ | 边沿检测 boff_counted, 仅 error_level 跳变时累加一次 |
| 11 | CAN_ProcessFrame 缺少 checksum 校验 | 干扰帧 DLC=8 但 data 被损坏 → 污染节点时间戳 | 加 XOR 异或校验, byte7不一致直接丢弃 |
| 12 | HeartBeatCheck error_level 门禁牵连全节点 | S2断线后 error_level≥2 → 整函数跳过 → S1也判离线 | 去掉 error_level≥2 gate, 每节点独立判定 |
| 13 | 告警帧无收发限流 | 从站连续上报 ALARM → FIFO堆积 → 正常帧处理延迟 | 从站 1→2s 降频, 主站 200ms/节点限频, 丢弃计数 |
| 14 | 主站 RX0 ISR 向量名错误 → 首帧卡死 | F103 md 启动文件 RX0 向量是 `USB_LP_CAN1_RX0_IRQHandler`, 代码定义 `CAN1_RX0_IRQHandler` 被 linker 当未用符号删除 → 向量指向 weak 死循环 | 改名 `USB_LP_CAN1_RX0_IRQHandler`, 查 map 确认 136B 代码进向量 |
| 15 | 高压下 W5500 阻塞饿死 CAN ring | W5500 SocketCmd/SendData 超时 100ms~1s 阻塞 Task_Unified, ring(64深)溢出丢帧 (F:OV) | 独立 Task_CAN_Drain (prio 3) 每 10ms 抢占 drain, ring 占用恒 ≤50 帧 |
| 16 | 原 FIFO0 轮询 300fps 上限 | 硬件 FIFO 3 深 × 10ms 轮询 = 300fps, 总线 90% 时丢 97% | FMP0 ISR + ring64 紧凑帧 + drain 任务 → 5000fps 零丢帧 (总线物理上限 ~4100fps) |

---

## 7. 相关文档

- `engineReuse_standard.md` — 从站复用架构 + 传感器抽象层
- `freertos_standard_SPL.md` — FreeRTOS 内核规范（中断优先级/栈/IWDG）
- `oled_standard_SPI.md` — OLED 显示规范
- `最终阶段.md` — 最终阶段完整设计文档
- `最终阶段遇到的问题及解决.md` — 调试修复记录
