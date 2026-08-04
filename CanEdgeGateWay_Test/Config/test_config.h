/**
 * CAN TX Stress / Slave Sim — 配置头
 *
 * 注意: CAN 位时序采用从站1 原版参数 (CAN_BS1_5tq/CAN_BS2_6tq/CAN_SJW_2tq,
 * 宏定义即寄存器编码), 不可用裸数字覆盖 — 曾误传裸值导致 428kbps, 主站无法解帧.
 */

#ifndef __TEST_CONFIG_H
#define __TEST_CONFIG_H

#include "CAN_User.h"          /* 协议常量: ID 基址 / 功能码 / 字节索引 */

/* ==================== 工作模式 ==================== */
#define TEST_SIM_SLAVE          0       /* 1=模拟从站(心跳+固定温湿度); 0=压测(帧率灌帧) */

/* ==================== 发送参数 ==================== */
/* 主站负载折算 (CAN_FRAME_BITS=122, 见主站 CAN_User.c):
 *   负载% = 帧率 × 122 / 500000 × 100%
 *   帧率  = 负载% × 500000 / 122
 * 物理可达 (0x55 低填充帧实测): 500000/122 ≈ 4100 fps = 负载 100%
 * 常用目标帧率:
 *   50% = 2049 fps    70%(节流低阈值) = 2869 fps
 *   90%(节流高阈值) = 3689 fps   100% = 4100 fps
 * 注意: 请求 ≥4100 fps 时总线打满, 主站负载钳位 100%, 再高无意义;
 *       测试站阻塞式发送(等全帧传完), 实际交付受总线物理上限约束. */
#define TEST_FRAME_RATE         3900    /* 压测模式: 每秒发送帧数; 0 = 最大突发 */
#define TEST_RUN_TIME_S         0       /* 运行秒数; 0 = 无限 */

/* 模拟节点号: 同时驱动 CAN 帧 src 字节 (= 从站 slave_node_id) */
#define TEST_NODE_ID_BASE       1
#define TEST_SIM_NODES          1       /* 压测模式模拟节点数 (1~8) */

#define TEST_FRAME_FUNC         CAN_FUNC_TEMP_HUMI   /* 压测功能码 */

/* Payload 模式: 0=固定字节  1=32位递增计数  2=模拟温湿度  3=功能码轮转
 * 压 90% 用 0: 0x55 位填充最少, 帧最短 */
#define TEST_PAYLOAD_MODE       0

/* 固定字节模式 (TEST_PAYLOAD_MODE==0) 的载荷字节; switch 恒编译, 不能条件定义 */
#define TEST_FIXED_PAYLOAD_B0   0x55

/* ==================== 从站模拟 (TEST_SIM_SLAVE==1) ==================== */
#define TEST_HB_INTERVAL_MS     1000    /* 心跳间隔 (主站 stale 3s, 需 <3000) */
#define TEST_DATA_INTERVAL_MS   1000    /* 温湿度发送间隔 */
#define TEST_TEMP_INT           26      /* 温度整数位 */
#define TEST_TEMP_DEC           5       /* 温度小数位 */
#define TEST_HUMI_INT           45      /* 湿度整数位 */
#define TEST_HUMI_DEC           2       /* 湿度小数位 */

/* ==================== 显示 ==================== */
#define TEST_DISP_ENABLE        1       /* OLED 统计显示 (PB8/PB9 软件I2C) */
#define TEST_DISP_REFRESH_MS    200     /* 刷新周期 */

/* ==================== 从站节点号别名 (供 CAN_User.c 使用) ==================== */
#define slave_node_id           TEST_NODE_ID_BASE

#endif /* __TEST_CONFIG_H */
