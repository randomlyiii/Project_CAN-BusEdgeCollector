#ifndef __W5500_H
#define __W5500_H

#include "stm32f10x.h"
#include <stdint.h>

/* ===================== 引脚定义 ===================== */
#define W5500_SPI                  SPI1
#define W5500_SCS_PORT             GPIOA
#define W5500_SCS_PIN              GPIO_Pin_4
#define W5500_RST_PORT             GPIOC
#define W5500_RST_PIN              GPIO_Pin_15

/* ===================== BSB (Block Select Bits) ===================== */
#define BSB_COMMON                0x00   /* 通用寄存器       */
#define BSB_SOCK0_REG             0x01   /* Socket0 寄存器   */
#define BSB_SOCK0_TX              0x08   /* Socket0 TX Buffer */
#define BSB_SOCK0_RX              0x10   /* Socket0 RX Buffer */

/* ===================== 通用寄存器 (BSB=0x00, offset) ===================== */
#define REG_MR                     0x0000
#define REG_GAR                    0x0001
#define REG_SUBR                   0x0005
#define REG_SHAR                   0x0009
#define REG_SIPR                   0x000F
#define REG_IR                     0x0015
#define REG_IMR                    0x0016
#define REG_SIR                    0x0017
#define REG_PHYCFGR                0x002E
#define REG_VERSIONR               0x0039

/* ===================== Socket0 寄存器 (BSB=0x01, offset) ===================== */
#define OFF_SN_MR                  0x0000
#define OFF_SN_CR                  0x0001
#define OFF_SN_IR                  0x0002
#define OFF_SN_SR                  0x0003
#define OFF_SN_PORT                0x0004
#define OFF_SN_DIPR                0x000C
#define OFF_SN_DPORT               0x0010
#define OFF_SN_TX_FSR              0x0020
#define OFF_SN_TX_RD               0x0022
#define OFF_SN_TX_WR               0x0024
#define OFF_SN_RX_RSR              0x0026
#define OFF_SN_RX_RD               0x0028
#define OFF_SN_KPALVTR             0x002F

/* ===================== 常量 ===================== */
#define MODBUS_PORT                502
#define W5500_RX_BUF_SIZE          1024

/* Socket 模式 */
#define Sn_MR_TCP                  0x21
#define Sn_MR_CLOSE                0x00

/* Socket 命令 */
#define Sn_CR_OPEN                 0x01
#define Sn_CR_LISTEN               0x02
#define Sn_CR_CONNECT              0x04
#define Sn_CR_DISCON               0x08
#define Sn_CR_CLOSE                0x10
#define Sn_CR_SEND                 0x20
#define Sn_CR_RECV                 0x40

/* Socket 状态 */
#define SOCK_CLOSED                0x00
#define SOCK_INIT                  0x13
#define SOCK_LISTEN                0x14
#define SOCK_ESTABLISHED           0x17
#define SOCK_CLOSE_WAIT            0x1C
#define SOCK_UDP                   0x22

/* 超时 */
#define SPI_TIMEOUT_MS             10
#define SOCK_CMD_TIMEOUT_MS        100
#define TCP_KEEPALIVE_ENABLE       0x01
#define PHY_LINK_MASK              0x01
#define W5500_REINIT_INTERVAL_MS   500   /* SPI 异常后重初始化节流 */

/* ===================== 环形接收缓冲区 ===================== */
typedef struct {
    uint8_t  buf[W5500_RX_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} RingBuf_t;

/* ===================== 错误码 ===================== */
#define W5500_OK                  0
#define W5500_ERR_SPI             -1
#define W5500_ERR_TIMEOUT         -2
#define W5500_ERR_LINK            -3
#define W5500_ERR_PARAM           -4
#define W5500_ERR_NOCONN          -5

/* ===================== API ===================== */
int8_t   W5500_Init(void);
uint8_t  W5500_IsOnline(void);
int8_t   W5500_Recovery(void);   /* SPI 异常后全量重初始化: 快探+节流+Init+ConfigNetwork */

int8_t   W5500_ConfigNetwork(void);
int8_t   W5500_SetNetParam(uint8_t *mac, uint8_t *ip, uint8_t *sub, uint8_t *gw);

int8_t   W5500_TCPServer_Start(uint16_t port);
void     W5500_TCPServer_Run(void);
uint8_t  W5500_IsConnected(void);
uint8_t  W5500_LinkUp(void);

/* 缓存版 — 不触 SPI，由 W5500_TCPServer_Run() 在内部更新 g_phy_linked。
 * 供其他任务（如 Housekeep OLED）读取，避免 SPI 总线竞争。 */
uint8_t  W5500_IsLinkUpCached(void);

int8_t   W5500_SendData(uint8_t *buf, uint16_t len);

/* 底层 SPI — bsb 控制块选择, off 块内偏移 */
uint8_t  W5500_ReadByte(uint8_t bsb, uint16_t off);
void     W5500_WriteByte(uint8_t bsb, uint16_t off, uint8_t val);
void     W5500_ReadBuf(uint8_t bsb, uint16_t off, uint8_t *buf, uint16_t len);
void     W5500_WriteBuf(uint8_t bsb, uint16_t off, uint8_t *buf, uint16_t len);

/* 环形缓冲区 */
void     RingBuf_Init(RingBuf_t *r);
uint16_t RingBuf_Available(RingBuf_t *r);
int8_t   RingBuf_Push(RingBuf_t *r, uint8_t *data, uint16_t len);
int16_t  RingBuf_Pop(RingBuf_t *r, uint8_t *out, uint16_t maxlen);
void     RingBuf_Clear(RingBuf_t *r);

extern RingBuf_t        g_w5500_rx_ring;
extern volatile uint8_t g_w5500_online;
extern uint8_t          g_w5500_version;

#endif
