#include "W5500.h"
#include "delay.h"
#include <string.h>

/* ======================== 局部变量 ======================== */
static uint8_t g_socket_status = SOCK_CLOSED;
static uint8_t g_socket_mem[W5500_TCP_BUF_SIZE];   // TCP 收/发共享缓冲区
static uint16_t g_tcp_rx_len = 0;                   // 待处理数据长度

/* ======================== SPI 操作 ======================== */

/* SPI CS 控制 */
#define W5500_CS_LOW()   GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS_PIN)
#define W5500_CS_HIGH()  GPIO_SetBits(W5500_SCS_PORT, W5500_SCS_PIN)

/* SPI 单字节读写 */
static uint8_t SPI_ReadWriteByte(uint8_t tx)
{
    while (SPI_I2S_GetFlagStatus(W5500_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(W5500_SPI, tx);
    while (SPI_I2S_GetFlagStatus(W5500_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(W5500_SPI);
}

/* ======================== W5500 寄存器读写 ======================== */

/**
  * @brief  SPI 帧头构造 (Variable Length Data Mode)
  *   Byte0: [Offset[15:13] | Block[4:3] | RW | OM[1:0]]
  *           OM = 11 (可变长度模式)
  *           RW = 0 (写) / 1 (读)
  *   Byte1: Offset[12:5]  (中间 8 位)
  *   Byte2: Offset[4:0] | Length[7:5]
  *   Byte3: Length[4:0] (长度，对于单字节读写为 0x01)
  */
static void W5500_SPI_SendHeader(uint16_t addr, uint8_t rw, uint16_t len)
{
    uint8_t block = (addr >> 6) & 0x03;    // Block: bits[7:6] of addr → block 0-3
    uint16_t offset = addr & 0x01FF;        // Offset: bits[8:0] of addr (within block)

    /* Byte0: BSB[4:0] | RW | OM[1:0] */
    uint8_t ctrl = (block << 3) | (rw << 2) | 0x03;  // OM = 11 (VDM)
    SPI_ReadWriteByte(ctrl);

    /* Byte1: Offset[15:8] 高 8 位 */
    SPI_ReadWriteByte((uint8_t)(offset >> 8));

    /* Byte2: Offset[7:0] 低 8 位 */
    SPI_ReadWriteByte((uint8_t)(offset & 0xFF));

    /* Byte3: 数据长度 (len-1 for VDM) */
    SPI_ReadWriteByte((uint8_t)(len - 1));
}

/**
  * @brief  读 1 字节寄存器
  */
uint8_t W5500_ReadByte(uint16_t addr)
{
    uint8_t val;

    W5500_CS_LOW();
    W5500_SPI_SendHeader(addr, 1, 1);   // RW=1, len=1
    val = SPI_ReadWriteByte(0xFF);
    W5500_CS_HIGH();

    return val;
}

/**
  * @brief  写 1 字节寄存器
  */
void W5500_WriteByte(uint16_t addr, uint8_t val)
{
    W5500_CS_LOW();
    W5500_SPI_SendHeader(addr, 0, 1);   // RW=0, len=1
    SPI_ReadWriteByte(val);
    W5500_CS_HIGH();
}

/**
  * @brief  读多字节寄存器
  */
void W5500_ReadBuf(uint16_t addr, uint8_t *buf, uint16_t len)
{
    if (len == 0) return;

    W5500_CS_LOW();
    W5500_SPI_SendHeader(addr, 1, len);
    for (uint16_t i = 0; i < len; i++)
        buf[i] = SPI_ReadWriteByte(0xFF);
    W5500_CS_HIGH();
}

/**
  * @brief  写多字节寄存器
  */
void W5500_WriteBuf(uint16_t addr, uint8_t *buf, uint16_t len)
{
    if (len == 0) return;

    W5500_CS_LOW();
    W5500_SPI_SendHeader(addr, 0, len);
    for (uint16_t i = 0; i < len; i++)
        SPI_ReadWriteByte(buf[i]);
    W5500_CS_HIGH();
}

/* ======================== 网络配置默认值 ======================== */

/* MAC 地址 (可自定义, 需唯一) */
static const uint8_t g_mac_addr[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
/* IP: 192.168.1.100 */
static const uint8_t g_ip_addr[4] = {192, 168, 1, 100};
/* 子网掩码: 255.255.255.0 */
static const uint8_t g_subnet[4] = {255, 255, 255, 0};
/* 网关: 192.168.1.1 */
static const uint8_t g_gw_addr[4] = {192, 168, 1, 1};

/* ======================== 硬件初始化 ======================== */

/**
  * @brief  SPI1 初始化
  */
static void W5500_SPI_Init(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef   spi;

    RCC_APB2PeriphClockCmd(W5500_SPI_GPIO_RCC | W5500_SPI_RCC, ENABLE);

    /* PA5 SCK, PA7 MOSI → 复用推挽 */
    gpio.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA6 MISO → 浮空输入 */
    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* SPI1 配置: 主机模式, 2 线全双工, 8MHz (72MHz / 9 = 8MHz) */
    SPI_I2S_DeInit(SPI1);
    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = SPI_Mode_Master;
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_Low;
    spi.SPI_CPHA              = SPI_CPHA_1Edge;
    spi.SPI_NSS               = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_9;  // 72/9 = 8MHz
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}

/**
  * @brief  W5500 复位引脚与 CS 引脚初始化
  */
static void W5500_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(W5500_RST_RCC | W5500_SCS_RCC, ENABLE);

    /* PA3 RST */
    gpio.GPIO_Pin   = W5500_RST_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W5500_RST_PORT, &gpio);

    /* PA4 SCS */
    gpio.GPIO_Pin   = W5500_SCS_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W5500_SCS_PORT, &gpio);

    /* 初始状态: 复位高, CS 高 */
    GPIO_SetBits(W5500_RST_PORT, W5500_RST_PIN);
    GPIO_SetBits(W5500_SCS_PORT, W5500_SCS_PIN);
}

/* ======================== W5500 初始化 ======================== */

void W5500_Init(void)
{
    uint8_t ver;

    W5500_GPIO_Init();
    W5500_SPI_Init();

    /* 硬件复位: RST 拉低 10us 后释放 */
    GPIO_ResetBits(W5500_RST_PORT, W5500_RST_PIN);
    Delay_us(10);
    GPIO_SetBits(W5500_RST_PORT, W5500_RST_PIN);
    Delay_ms(50);   // 等待 W5500 内部初始化完成

    /* 检查版本寄存器 (0x4 表示 W5500) */
    ver = W5500_ReadByte(REG_VERSIONR);
    (void)ver;  // 实际调试时可验证 ver == 0x04

    g_socket_status = SOCK_CLOSED;
    g_tcp_rx_len = 0;
}

void W5500_ConfigNetwork(void)
{
    /* 配置 MAC 地址 */
    W5500_WriteBuf(REG_SHAR, (uint8_t*)g_mac_addr, 6);

    /* 配置 IP 地址 */
    W5500_WriteBuf(REG_SIPR, (uint8_t*)g_ip_addr, 4);

    /* 配置子网掩码 */
    W5500_WriteBuf(REG_SUBR, (uint8_t*)g_subnet, 4);

    /* 配置网关 */
    W5500_WriteBuf(REG_GAR, (uint8_t*)g_gw_addr, 4);
}

/* ======================== TCP Server ======================== */

void W5500_TCPServer_Start(uint16_t port)
{
    /* 关闭 Socket 0 (确保初始状态) */
    W5500_WriteByte(REG_SN_CR(0), Sn_CR_CLOSE);
    Delay_ms(10);

    /* 设置 Socket 模式为 TCP */
    W5500_WriteByte(REG_SN_MR(0), Sn_MR_TCP);

    /* 设置端口 (大端) */
    W5500_WriteByte(REG_SN_PORT(0), (uint8_t)(port >> 8));
    W5500_WriteByte(REG_SN_PORT(0) + 1, (uint8_t)(port & 0xFF));

    /* 打开 Socket */
    W5500_WriteByte(REG_SN_CR(0), Sn_CR_OPEN);
    Delay_ms(5);

    /* 监听 */
    W5500_WriteByte(REG_SN_CR(0), Sn_CR_LISTEN);
    Delay_ms(5);

    g_socket_status = SOCK_LISTEN;
}

void W5500_TCPServer_Run(void)
{
    uint8_t sr;

    sr = W5500_ReadByte(REG_SN_SR(0));
    g_socket_status = sr;

    switch (sr) {
    case SOCK_ESTABLISHED: {
        /* 检查是否有数据到达 */
        uint16_t rx_size;
        uint8_t size_h, size_l;

        size_h  = W5500_ReadByte(REG_SN_RX_RSR(0));
        size_l  = W5500_ReadByte(REG_SN_RX_RSR(0) + 1);
        rx_size = ((uint16_t)size_h << 8) | size_l;

        if (rx_size > 0) {
            if (rx_size > W5500_TCP_BUF_SIZE)
                rx_size = W5500_TCP_BUF_SIZE;
            W5500_ReceiveData(g_socket_mem);
            g_tcp_rx_len = rx_size;
        }
        break;
    }

    case SOCK_CLOSE_WAIT:
        /* 客户端断开, 发送断开命令后重新监听 */
        W5500_WriteByte(REG_SN_CR(0), Sn_CR_DISCON);
        Delay_ms(1);
        W5500_WriteByte(REG_SN_CR(0), Sn_CR_CLOSE);
        Delay_ms(5);
        W5500_TCPServer_Start(MODBUS_PORT);
        break;

    case SOCK_CLOSED:
        /* Socket 关闭, 重新开启 */
        Delay_ms(10);
        W5500_TCPServer_Start(MODBUS_PORT);
        break;

    default:
        break;
    }
}

uint16_t W5500_ReceiveData(uint8_t *buf)
{
    uint16_t rx_size;
    uint16_t rx_rd;
    uint8_t rx_rd_h, rx_rd_l;

    /* 获取接收数据大小 */
    uint8_t h = W5500_ReadByte(REG_SN_RX_RSR(0));
    uint8_t l = W5500_ReadByte(REG_SN_RX_RSR(0) + 1);
    rx_size = ((uint16_t)h << 8) | l;

    if (rx_size == 0) return 0;
    if (rx_size > W5500_TCP_BUF_SIZE)
        rx_size = W5500_TCP_BUF_SIZE;

    /* 获取 RX 读指针 */
    rx_rd_h = W5500_ReadByte(REG_SN_RX_RD(0));
    rx_rd_l = W5500_ReadByte(REG_SN_RX_RD(0) + 1);
    rx_rd   = ((uint16_t)rx_rd_h << 8) | rx_rd_l;

    /* 从 RX buffer 读取数据 */
    W5500_ReadBuf(RX_BUF_BASE + rx_rd, buf, rx_size);

    /* 更新 RX 读指针 */
    rx_rd += rx_size;
    W5500_WriteByte(REG_SN_RX_RD(0), (uint8_t)(rx_rd >> 8));
    W5500_WriteByte(REG_SN_RX_RD(0) + 1, (uint8_t)(rx_rd & 0xFF));

    /* 发送 RECV 命令 */
    W5500_WriteByte(REG_SN_CR(0), Sn_CR_RECV);

    return rx_size;
}

void W5500_SendData(uint8_t *buf, uint16_t len)
{
    uint16_t tx_wr;
    uint16_t free_size;
    uint8_t free_h, free_l;
    uint8_t tx_wr_h, tx_wr_l;

    if (len == 0) return;
    if (g_socket_status != SOCK_ESTABLISHED) return;

    /* 等待 TX 缓冲区有空闲 */
    uint32_t timeout = 1000;  // ~10ms
    do {
        free_h = W5500_ReadByte(REG_SN_TX_FSR(0));
        free_l = W5500_ReadByte(REG_SN_TX_FSR(0) + 1);
        free_size = ((uint16_t)free_h << 8) | free_l;
        if (free_size >= len) break;
        Delay_us(10);
    } while (timeout--);

    if (free_size < len) return;  // 超时

    /* 获取 TX 写指针 */
    tx_wr_h = W5500_ReadByte(REG_SN_TX_WR(0));
    tx_wr_l = W5500_ReadByte(REG_SN_TX_WR(0) + 1);
    tx_wr   = ((uint16_t)tx_wr_h << 8) | tx_wr_l;

    /* 写入 TX buffer */
    W5500_WriteBuf(TX_BUF_BASE + tx_wr, buf, len);

    /* 更新 TX 写指针 */
    tx_wr += len;
    W5500_WriteByte(REG_SN_TX_WR(0), (uint8_t)(tx_wr >> 8));
    W5500_WriteByte(REG_SN_TX_WR(0) + 1, (uint8_t)(tx_wr & 0xFF));

    /* 发送 SEND 命令 */
    W5500_WriteByte(REG_SN_CR(0), Sn_CR_SEND);
}

uint8_t W5500_IsConnected(void)
{
    return (g_socket_status == SOCK_ESTABLISHED) ? 1 : 0;
}

/**
  * @brief  获取接收缓冲区指针
  */
uint8_t* W5500_GetRxBuf(void)
{
    return g_socket_mem;
}

uint16_t W5500_GetRxLen(void)
{
    return g_tcp_rx_len;
}

void W5500_ClrRxLen(void)
{
    g_tcp_rx_len = 0;
}
