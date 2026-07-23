#ifndef __W5500_H
#define __W5500_H

#include "stm32f10x.h"
#include <stdint.h>

/* ======================== 引脚定义 ======================== */
#define W5500_SPI                  SPI1
#define W5500_SPI_RCC              RCC_APB2Periph_SPI1

#define W5500_SCS_PORT             GPIOA
#define W5500_SCS_PIN              GPIO_Pin_4
#define W5500_SCS_RCC              RCC_APB2Periph_GPIOA

#define W5500_RST_PORT             GPIOA
#define W5500_RST_PIN              GPIO_Pin_3
#define W5500_RST_RCC              RCC_APB2Periph_GPIOA

/* SPI 引脚 (PA5 SCK, PA6 MISO, PA7 MOSI) */
#define W5500_SPI_GPIO_PORT        GPIOA
#define W5500_SPI_GPIO_PINS        (GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)
#define W5500_SPI_GPIO_RCC         RCC_APB2Periph_GPIOA

/* ======================== 常用寄存器地址 ======================== */
/* 通用寄存器 (Common Register Block = 0x00) */
#define REG_MR                     0x0000   // Mode Register
#define REG_GAR                    0x0001   // Gateway Address (4 bytes)
#define REG_SUBR                   0x0005   // Subnet Mask (4 bytes)
#define REG_SHAR                   0x0009   // Source MAC (6 bytes)
#define REG_SIPR                   0x000F   // Source IP (4 bytes)
#define REG_IR                     0x0015   // Interrupt Register
#define REG_IMR                    0x0016   // Interrupt Mask
#define REG_SIR                    0x0017   // Socket Interrupt Register
#define REG_PTYPETIMR              0x001C   // PPPoE... 暂不用
#define REG_VERSIONR               0x001F   // Version Register

/* Socket 寄存器 (Socket 0 基址, 每 Socket 偏移 0x0100) */
#define REG_SN_MR(n)               (0x0000 + (n) * 0x0100)  // Socket n Mode
#define REG_SN_CR(n)               (0x0001 + (n) * 0x0100)  // Socket n Command
#define REG_SN_IR(n)               (0x0002 + (n) * 0x0100)  // Socket n Interrupt
#define REG_SN_SR(n)               (0x0003 + (n) * 0x0100)  // Socket n Status
#define REG_SN_PORT(n)             (0x0004 + (n) * 0x0100)  // Socket n Source Port (2B)
#define REG_SN_DIPR(n)             (0x000C + (n) * 0x0100)  // Socket n Dest IP (4B)
#define REG_SN_DPORT(n)            (0x0010 + (n) * 0x0100)  // Socket n Dest Port (2B)
#define REG_SN_TX_FSR(n)           (0x0020 + (n) * 0x0100)  // TX Free Size
#define REG_SN_TX_RD(n)            (0x0022 + (n) * 0x0100)  // TX Read Pointer
#define REG_SN_TX_WR(n)            (0x0024 + (n) * 0x0100)  // TX Write Pointer
#define REG_SN_RX_RSR(n)           (0x0026 + (n) * 0x0100)  // RX Received Size
#define REG_SN_RX_RD(n)            (0x0028 + (n) * 0x0100)  // RX Read Pointer

/* Socket TX/RX Buffer 基址 */
#define SOCKET_TX_BUF(n)           (0x0000 + (n) * 0x4000)  // Socket n TX buffer
#define SOCKET_RX_BUF(n)           (0x0000 + (n) * 0x4000)  // Socket n RX buffer
#define TX_BUF_BASE                0x8000                    // TX Buffer 基地址
#define RX_BUF_BASE                0xC000                    // RX Buffer 基地址

/* ======================== 常量 ======================== */
#define SOCKET_COUNT               1          // 阶段一只用 Socket 0
#define MODBUS_PORT                502        // Modbus TCP 端口
#define W5500_TCP_BUF_SIZE         512        // TCP 收发缓冲区大小

/* Socket 模式 */
#define Sn_MR_TCP                  0x21       // TCP 模式
#define Sn_MR_UDP                  0x02       // UDP 模式
#define Sn_MR_CLOSE                0x00       // 关闭

/* Socket 命令 */
#define Sn_CR_OPEN                 0x01       // 打开
#define Sn_CR_LISTEN               0x02       // 监听
#define Sn_CR_CONNECT              0x04       // 连接
#define Sn_CR_DISCON               0x08       // 断开
#define Sn_CR_CLOSE                0x10       // 关闭
#define Sn_CR_SEND                 0x20       // 发送
#define Sn_CR_RECV                 0x40       // 接收

/* Socket 状态 */
#define SOCK_CLOSED                0x00
#define SOCK_INIT                  0x13
#define SOCK_LISTEN                0x14
#define SOCK_ESTABLISHED           0x17
#define SOCK_CLOSE_WAIT            0x1C
#define SOCK_UDP                   0x22

/* ======================== 函数声明 ======================== */
void W5500_Init(void);                       // 硬件初始化(SPI + W5500复位)
void W5500_ConfigNetwork(void);              // 配置网络参数
void W5500_TCPServer_Start(uint16_t port);   // 开启 TCP Server
void W5500_TCPServer_Run(void);              // TCP Server 状态机(主循环调用)
uint16_t W5500_ReceiveData(uint8_t *buf);    // 接收 TCP 数据
void W5500_SendData(uint8_t *buf, uint16_t len); // 发送 TCP 数据
uint8_t W5500_IsConnected(void);             // TCP 连接状态

/* 内部接收缓冲区访问 (ModbusTCP 使用) */
uint8_t* W5500_GetRxBuf(void);               // 获取接收缓冲区指针
uint16_t W5500_GetRxLen(void);               // 获取接收数据长度
void     W5500_ClrRxLen(void);               // 清除接收长度标志

/* SPI 读写 */
uint8_t  W5500_ReadByte(uint16_t addr);      // 读 1 字节
void     W5500_WriteByte(uint16_t addr, uint8_t val); // 写 1 字节
void     W5500_ReadBuf(uint16_t addr, uint8_t *buf, uint16_t len);  // 读多字节
void     W5500_WriteBuf(uint16_t addr, uint8_t *buf, uint16_t len); // 写多字节

#endif
