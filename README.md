# CAN Bus Edge Collector
# 项目设计-工业数据采集终端
# 通用工业级抗干扰数据采集终端设计
# 自定义CAN应用层通信协议
# 硬件资料参考 "./硬件厂家资料"
# 如果看不到大概是私有化仓库部署

## 文件分层
|--本文件(README.md)
|--phase_md(各阶段设计文档/验收截图/问题记录)
|  |--can_standard_SPI.md(CAN通信协议规范)
|  |--engineReuse_standard.md(从站传感器架构设计)
|  |--最终阶段.md(最终阶段完整设计)
|  |--最终阶段遇到的问题及解决.md(调试修复全记录)
|--CanEdgeGateWay_Master(主站)
|--CanEdgeGateWay_Slave (从站)
|--排线设计
|  |--README.md(引脚排线总览)
|  |--阶段一排线设计.md
|  |--阶段二排线设计.md
|  |--阶段三排线设计.md

---

## 硬件规格

| 项目 | 参数 |
|------|------|
| MCU | STM32F103C8T6 (72MHz, 64KB Flash, 20KB RAM) |
| CAN 收发器 | TJA1050（模块内置 120Ω 终端） |
| 以太网 | W5500（SPI 接口，硬件 TCP/IP） |
| OLED | 0.96" SSD1306，128×64，I2C |
| 从站传感器 | DHT11（温湿度）、BH1750（光照）、LM393（开关量） |
| 电源 | DC 12V ±10%（主从站同源供电） |

**波特率**: 500kbps（固定，不可降）。采样点 75%（BS1=8tq, BS2=3tq, SJW=2tq, Prescaler=6, 详见 phase_md）。

---

## Modbus TCP 寄存器操作

上位机通过 ModbusPoll（功能码 0x03 读 / 0x06 写）操作寄存器，端口 502。

### 常用监控寄存器

| 地址 | 名称 | 说明 |
|------|------|------|
| 0x0000 | 从站1 温度 | 实际值 × 10，如 253 = 25.3°C |
| 0x0001 | 从站1 湿度 | 实际值 × 10，如 621 = 62.1% |
| 0x0002 | 从站1 心跳计数 | 累计值，溢出回绕 |
| 0x0004 | 从站1 在线标志 | 1=在线 |
| 0x0005~0x000B | 从站2 寄存器 | 同上 + LM393_AO/DO |
| 0x0010 | CAN 总线负载 | 0~10000，如 1900=19.00% |
| 0x0013 | CAN 错误等级 | 0=OK, 1=WRN, 3=BOF |

### 常用控制寄存器

用 ModbusPoll 的 "Write Single Register" (0x06) 写入。

| 地址 | 写入值 | 作用 |
|------|--------|------|
| 0x0020 | 0x55 | 复位 CAN 控制器（BusOff 时手动恢复）|
| 0x002C | 毫秒值 | OLED 自动回主页超时，写 0=永不回主页 |

---

## 架构概览

### 主站

三任务解耦架构。CAN 帧处理独立成高优先级任务，避免 W5500 阻塞（实测 100~700ms）饿死 CAN ring：

```
Task_CAN_Drain (prio 3, 10ms):   ← CAN RX 主路径
  drain ring_high (FMP0 ISR 填充, 64 深紧凑帧 12B) → CAN_ProcessFrame
  (g_can_ready 门闸: 初始化完成前只清 ring 不处理)

Task_Unified (prio 1, 10ms):
  1. CAN 监控: ErrorMonitor → HeartBeatCheck → CalcBusLoad
  2. W5500:   TCP Server Run → Modbus Process → SyncFromCAN
  3. 按键翻页 + IWDG 喂狗

Task_Housekeep (prio 0, 1s): OLED 刷新 + 栈水位监控
```

接收链: 硬件 FIFO0 → FMP0 中断 ISR (`USB_LP_CAN1_RX0_IRQHandler`) → 紧凑帧 ring
(12B×64, 锁无关 SPSC) → Task_CAN_Drain → CAN_ProcessFrame (DLC + checksum 校验)。
任务只读内存 ring，不碰硬件 FIFO/SPI → 不违反顺序约束。

关键约束：
- SPI（W5500）与 CAN 共用 GPIOA，不可并发（详见 `phase_md` 问题三）
- Task_CAN_Drain 只读内存 ring，不产生 SPI 边沿，物理上与顺序架构等价
- 主站 **receive-only**，不发帧（无 ACK 时硬件重传推 TEC→BusOff）
- NART=ENABLE（从站），防单节点故障拖累全网

### CAN 错误恢复（全链路闭环）

| 角色 | ABOM | BusOff 恢复 | 恢复时间 |
|------|------|------------|---------|
| 主站 | DISABLE | 软件 INRQ 协议复位 + 冷却退避 | 200ms / 3s（连续 5 次后）|
| 从站 | ENABLE | 硬件自动 128×11bit 隐性位检测 | 2.8ms |

### 心跳检测

两段标记，每节点独立判定。仅通过 checksum 校验的合法帧才能刷新时间戳。

```
收到合法帧 ─→ ON       (delta ≤ 3s)
3s 无合法帧 ─→ EXP     (stale，短期过期)
30s EXP ─→ OFF     (offline，确认离线)
```

**节点隔离**：单节点断线不影响其他节点在线状态。无全局 error_level 门禁。

---

## 功能实现状态

| 功能 | 所属 | 状态 | 说明 |
|------|------|------|------|
| CAN 0/1/2 级优先级分组 | 双方 | ✅ 完成 | 0x100/0x200/0x300 ID 分段 |
| 升降级机制 | 主站 | 🟡 框架完成 | 主站 receive-only 期间不生效 |
| 双 FIFO 缓冲（16+64 深）| 主站 | ✅ 完成 | 紧急/常态分级 + 溢出保护 + 历史缓存消费 |
| 总线负载管控 | 主站 | ✅ 完成 | 100ms 窗口，三档节流（70%/90%），折算位 122 |
| CAN RX 高吞吐 | 主站 | ✅ 完成 | FMP0 ISR + ring64 紧凑帧 + 独立 drain 任务，压测 5000fps 注入（超总线物理上限 ~4100fps，总线饱和）接收零丢帧（旧轮询 300fps 上限）|
| 心跳检测 + BusOff 保护 | 主站 | ✅ 完成 | stale/offline 两段标记，error_level≥2 跳过 |
| CAN 错误状态机 | 双方 | ✅ 完成 | 主站 INRQ 协议恢复 + 从站 ABOM 自愈 |
| OLED 自动回主页超时可调 | 主站 | ✅ 完成 | Modbus 0x002C 动态设置或 #define 编译改 |
| Modbus TCP 转换 | 主站 | ✅ 完成 | 实时+历史双缓存，断网分级 |
| 从站传感器抽象层 | 从站 | ✅ 完成 | sensor_manager 表驱动 + 故障累计 + 手动按键告警 + 本地缓存补传 |
| 多 Modbus 客户端 | 主站 | ❌ 未来需求 | W5500 单 socket |
| 固件远程升级 | — | ❌ 未来需求 | 当前仅 ST-Link SWD 烧录 |
| 硬件隔离方案 | — | ❌ 未来需求 | 当前 TJA1050，ISO1050 量产待布板 |

---

## 关键设计决策（10 条）

详细根因分析见 `phase_md/最终阶段遇到的问题及解决.md`。

1. **从站 CAN 不需要中断** — 纯轮询满足低频采集，ISR 高频触发会劫持 FreeRTOS 调度
2. **所有 while 循环必须加超时** — 外设状态位可能因电气噪声永远不变
3. **I2C 共享总线必须互斥** — 不同优先级的两个任务操作同一软件 I2C = 数据竞争
4. **CAN 初始化必须在调度器启动后** — `vTaskStartScheduler()` 破坏 CAN 寄存器
5. **心跳检测无 error_level 门禁** — 单节点断线的总线错误不牵连其他节点。每节点 stale/offline 独立判定
6. **CAN BusOff 恢复用 INRQ 协议** — `DeInit+Init` 暴力复位违反 CAN 协议，可能引发总线风暴
7. **OLED sprintf 必须 ≤16 字符** — `g_oled_line[17]` 无溢出检查，超长踩内存
8. **SPI 与 CAN 不可并发** — GPIOA 端口共用，SPI 时钟耦合至 CAN RX 引脚
9. **ALARM 帧不计 tx_budget** — 紧急帧必须无条件发出，不受数据流限速约束
10. **NART=ENABLE（从站）** — 硬件自动重传在总线故障时推 TEC 至 BusOff，单帧丢失影响远小于全网震荡
11. **checksum 校验必须与 DLC 校验配合** — DLC 保证长度合规，checksum 保证数据完整。缺任一都可能被干扰帧污染节点时间戳
12. **告警帧收发均限流** — 从站发送 1 次/秒（连续告警降 2s），主站接收 200ms/节点。防止单从站故障引起的总线告警风暴
13. **中断或轮询，二选一** — 不可混合使用。ISR 和 Task 同时读同一硬件 FIFO 必然产生竞态，导致帧丢失
14. **RX0 ISR 向量名是 `USB_LP_CAN1_RX0_IRQHandler`** — F103 md 启动文件如此映射。写成 `CAN1_RX0_IRQHandler` 会被 linker 当未用符号删除，向量指向 weak 死循环 → 首帧卡死
15. **CAN 帧处理必须独立任务** — W5500 SocketCmd/SendData 阻塞 100~700ms，同任务内 ring drain 会被饿死丢帧；独立 prio 3 任务每 10ms 抢占 drain
16. **负载折算用真实帧长 122 位** — 原 108 位（无填充最小值）低估真实总线占用，4100fps 满速在老 108 位折算下显示 88.56%，实为总线已满；改 122 位后 100%

---

## 安全注意事项

- **严禁带电插拔** 传感器/CAN 线缆。本设计为 **非 SIL 等级** 工业数据采集前端，不得用于人身安全相关系统。
- 工作温度 -20~+70°C，污染等级 PD2，防护等级 IP20（须安装于电控柜内）。
