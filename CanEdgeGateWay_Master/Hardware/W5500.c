/**
 * W5500 以太网驱动 (基于 Project_template 验证的 SPI 帧格式)
 *
 * 引脚: RST=PC15, SCS=PA4, SCK=PA5, MISO=PA6(AF_PP), MOSI=PA7
 * SPI:  Mode0, 36MHz, FDM1读/VDM写, BSB仅在控制字节
 *
 * NOTE: SPI 与 CAN 不可并发。由上层统一任务顺序调度,
 *       确保 SPI 操作期间 CAN 帧已处理完毕, 反之亦然。
 */
#include "W5500.h"
#include "delay.h"
#include "config.h"
#include <string.h>

/* ===================== 全局状态 ===================== */
RingBuf_t        g_w5500_rx_ring;
volatile uint8_t g_w5500_online  = 0;
uint8_t          g_w5500_version = 0;

static uint8_t  g_socket_status   = SOCK_CLOSED;
static uint8_t  g_chip_ok         = 0;
static uint32_t g_close_wait_tick = 0;
static uint32_t g_closed_tick     = 0;
static uint8_t  g_phy_linked      = 0;
static uint32_t g_reinit_throttle_tick = 0;   /* SPI 异常重初始化节流 */
static uint32_t g_cfg_verify_tick        = 0;   /* 网络参数周期校验 */

/* ===================== SPI 控制字节 (模板验证) ===================== */
#define T_VDM       0x00
#define T_FDM1      0x01
#define T_FDM2      0x02
#define T_RWB_RD    0x00
#define T_RWB_WR    0x04
#define T_COMMON    0x00
#define T_S0_REG    0x08
#define T_S0_TXBUF  0x10
#define T_S0_RXBUF  0x18

/* ===================== CS 宏 ===================== */
#define W5500_CS_LOW()   GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS_PIN)
#define W5500_CS_HIGH()  GPIO_SetBits(W5500_SCS_PORT, W5500_SCS_PIN)

/* ===================== SPI 底层 ===================== */
static uint8_t SPI_SendByte(uint8_t dat)
{
    uint32_t to = 200;          /* ≈17µs @72MHz */
    SPI_I2S_SendData(W5500_SPI, dat);
    while (--to) {
        if (SPI_I2S_GetFlagStatus(W5500_SPI, SPI_I2S_FLAG_TXE) != RESET)
            return 0;           /* OK */
    }
    g_chip_ok = 0;              /* SPI 异常 — 标记 W5500 离线 */
    return 1;                   /* 超时 */
}
static void SPI_SendShort(uint16_t dat) { SPI_SendByte(dat>>8); SPI_SendByte(dat&0xFF); }

/* ===================== 寄存器读写 ===================== */

/* 通用寄存器读1字节: [AddrH][AddrL][Ctrl=FDM1|RD][Len=0][Data] */
static uint8_t R_Common(uint16_t reg)
{
    uint8_t val;
    W5500_CS_LOW();
    SPI_SendShort(reg);
    SPI_SendByte(T_FDM1 | T_RWB_RD | T_COMMON);
    SPI_I2S_ReceiveData(W5500_SPI);
    SPI_SendByte(0x00);
    val = (uint8_t)SPI_I2S_ReceiveData(W5500_SPI);
    W5500_CS_HIGH();
    return val;
}
/* 通用寄存器写1字节 */
static void W_Common(uint16_t reg, uint8_t dat)
{
    W5500_CS_LOW();
    SPI_SendShort(reg);
    SPI_SendByte(T_FDM1 | T_RWB_WR | T_COMMON);
    SPI_SendByte(dat);
    W5500_CS_HIGH();
}
/* 通用寄存器写N字节 (VDM) */
static void W_CommonBuf(uint16_t reg, uint8_t *buf, uint16_t len)
{
    W5500_CS_LOW();
    SPI_SendShort(reg);
    SPI_SendByte(T_VDM | T_RWB_WR | T_COMMON);
    for (uint16_t i = 0; i < len; i++) SPI_SendByte(buf[i]);
    W5500_CS_HIGH();
}
/* Socket寄存器读1字节 */
static uint8_t R_Sock(uint8_t sock, uint16_t reg)
{
    uint8_t val, bsb = (uint8_t)(sock * 0x20 + T_S0_REG);
    W5500_CS_LOW();
    SPI_SendShort(reg);
    SPI_SendByte(T_FDM1 | T_RWB_RD | bsb);
    SPI_I2S_ReceiveData(W5500_SPI);
    SPI_SendByte(0x00);
    val = (uint8_t)SPI_I2S_ReceiveData(W5500_SPI);
    W5500_CS_HIGH();
    return val;
}
/* Socket寄存器写1字节 */
static void W_Sock(uint8_t sock, uint16_t reg, uint8_t dat)
{
    uint8_t bsb = (uint8_t)(sock * 0x20 + T_S0_REG);
    W5500_CS_LOW(); SPI_SendShort(reg);
    SPI_SendByte(T_FDM1 | T_RWB_WR | bsb); SPI_SendByte(dat);
    W5500_CS_HIGH();
}
/* Socket寄存器读2字节 */
static uint16_t R_Sock2(uint8_t sock, uint16_t reg)
{
    uint16_t val; uint8_t bsb = (uint8_t)(sock * 0x20 + T_S0_REG);
    W5500_CS_LOW(); SPI_SendShort(reg);
    SPI_SendByte(T_FDM2 | T_RWB_RD | bsb);
    SPI_I2S_ReceiveData(W5500_SPI); SPI_SendByte(0x00);
    val  = (uint16_t)SPI_I2S_ReceiveData(W5500_SPI) << 8;
    SPI_SendByte(0x00); val |= SPI_I2S_ReceiveData(W5500_SPI);
    W5500_CS_HIGH(); return val;
}
/* Socket寄存器写2字节 */
static void W_Sock2(uint8_t sock, uint16_t reg, uint16_t dat)
{
    uint8_t bsb = (uint8_t)(sock * 0x20 + T_S0_REG);
    W5500_CS_LOW(); SPI_SendShort(reg);
    SPI_SendByte(T_FDM2 | T_RWB_WR | bsb); SPI_SendShort(dat);
    W5500_CS_HIGH();
}
/* Socket Buffer读 (VDM) */
static void R_SockBuf(uint8_t sock, uint8_t bsb_code, uint16_t off, uint8_t *buf, uint16_t len)
{
    uint8_t bsb = (uint8_t)(sock * 0x20 + bsb_code);
    W5500_CS_LOW(); SPI_SendShort(off);
    SPI_SendByte(T_VDM | T_RWB_RD | bsb);
    SPI_I2S_ReceiveData(W5500_SPI);
    for (uint16_t i = 0; i < len; i++) { SPI_SendByte(0x00); buf[i] = (uint8_t)SPI_I2S_ReceiveData(W5500_SPI); }
    W5500_CS_HIGH();
}
/* Socket Buffer写 (VDM) */
static void W_SockBuf(uint8_t sock, uint8_t bsb_code, uint16_t off, uint8_t *buf, uint16_t len)
{
    uint8_t bsb = (uint8_t)(sock * 0x20 + bsb_code);
    W5500_CS_LOW(); SPI_SendShort(off);
    SPI_SendByte(T_VDM | T_RWB_WR | bsb);
    for (uint16_t i = 0; i < len; i++) SPI_SendByte(buf[i]);
    W5500_CS_HIGH();
}

/* ===================== 公共API (BSB映射) ===================== */
uint8_t W5500_ReadByte(uint8_t bsb, uint16_t off)
{
    if (bsb == BSB_COMMON)    return R_Common(off);
    if (bsb == BSB_SOCK0_REG) return R_Sock(0, off);
    return R_Sock(bsb - 1, off);
}
void W5500_WriteByte(uint8_t bsb, uint16_t off, uint8_t val)
{
    if (bsb == BSB_COMMON)    { W_Common(off, val); return; }
    if (bsb == BSB_SOCK0_REG) { W_Sock(0, off, val); return; }
    W_Sock(bsb - 1, off, val);
}
void W5500_ReadBuf(uint8_t bsb, uint16_t off, uint8_t *buf, uint16_t len)
{
    if (!len) return;
    if (bsb == BSB_SOCK0_RX) { R_SockBuf(0, T_S0_RXBUF, off, buf, len); return; }
    for (uint16_t i = 0; i < len; i++) buf[i] = W5500_ReadByte(bsb, off + i);
}
void W5500_WriteBuf(uint8_t bsb, uint16_t off, uint8_t *buf, uint16_t len)
{
    if (!len) return;
    if (bsb == BSB_SOCK0_TX) { W_SockBuf(0, T_S0_TXBUF, off, buf, len); return; }
    if (bsb == BSB_COMMON)   { W_CommonBuf(off, buf, len); return; }
    for (uint16_t i = 0; i < len; i++) W5500_WriteByte(bsb, off + i, buf[i]);
}

/* ===================== GPIO + SPI 初始化 ===================== */
static void W5500_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin = W5500_RST_PIN; gpio.GPIO_Speed = GPIO_Speed_10MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(W5500_RST_PORT, &gpio);
    GPIO_ResetBits(W5500_RST_PORT, W5500_RST_PIN);
}

static void W5500_SPI_Init(void)
{
    GPIO_InitTypeDef gpio; SPI_InitTypeDef spi;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz; gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);

    gpio.GPIO_Pin = W5500_SCS_PIN; gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(W5500_SCS_PORT, &gpio);
    GPIO_SetBits(W5500_SCS_PORT, W5500_SCS_PIN);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master; spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low; spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft; spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    /* 36MHz — 降速至 Prescaler_8/16 会导致 W5500 ver:0x00 无法初始化 */
    spi.SPI_FirstBit = SPI_FirstBit_MSB; spi.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &spi); SPI_Cmd(SPI1, ENABLE);
}

/* ===================== 初始化 (仅硬件, 网络参数由 ConfigNetwork 负责) ===================== */
int8_t W5500_Init(void)
{
    uint8_t ver;
    W5500_GPIO_Init(); W5500_SPI_Init();

    GPIO_ResetBits(W5500_RST_PORT, W5500_RST_PIN); Delay_ms(50);
    GPIO_SetBits(W5500_RST_PORT, W5500_RST_PIN);   Delay_ms(200);

    RingBuf_Init(&g_w5500_rx_ring);
    g_socket_status = SOCK_CLOSED; g_chip_ok = g_w5500_online = g_phy_linked = 0;
    g_w5500_version = 0;

    W_Common(REG_MR, 0x80); Delay_ms(10);     /* 软件复位 */
    ver = R_Common(REG_VERSIONR);              /* 读版本号 */
    g_w5500_version = ver;
    if (ver != 0x04) return W5500_ERR_SPI;

    g_chip_ok = 1;

    /* Socket0 buf=2KB, 重传2000/200ms, 次数8, 关中断 */
    W_Sock(0, 0x001E, 0x02); W_Sock(0, 0x001F, 0x02);
    { uint8_t r[2]={0x07,0xD0}; W_CommonBuf(0x0019, r, 2); }
    W_Common(0x001B, 8);
    W_Common(REG_IMR, 0x00); W_Common(REG_IR, 0xFF);
    return W5500_OK;
}

uint8_t W5500_IsOnline(void) { return g_chip_ok; }

/* ===================== 自动恢复 =====================
 * 背景: SPI_SendByte 超时 → g_chip_ok=0 → 旧代码永久死, 需断电重启.
 *       碰触 W5500 模块导致 SPI/电源引脚瞬时接触不良即触发.
 *
 * 快探前置: 芯片真死时 R_Common 超时仅 ~2µs, 不烧 W5500_Init 的
 *       50+200+10ms 固定延时; 芯片复活后下一次节流窗口即满血恢复.
 * 节流: 死芯片期间每 500ms 才尝试一次, 不占 Task_Unified (CAN drain
 *       任务 prio 3 照常抢占, 负载度不受影响). */
int8_t W5500_Recovery(void)
{
    if (Delay_GetTick() - g_reinit_throttle_tick < W5500_REINIT_INTERVAL_MS) return W5500_ERR_SPI;
    g_reinit_throttle_tick = Delay_GetTick();
    if (R_Common(REG_VERSIONR) != 0x04) return W5500_ERR_SPI;   /* SPI/电源仍断 */
    if (W5500_Init() != W5500_OK)                    return W5500_ERR_SPI;
    if (W5500_ConfigNetwork() != W5500_OK)           return W5500_ERR_SPI;
    if (W5500_TCPServer_Start(MODBUS_PORT) != W5500_OK) return W5500_ERR_TIMEOUT;
    return W5500_OK;
}

/* ===================== 网络配置 ===================== */
static uint8_t g_mac[6] = W5500_CFG_MAC;
static uint8_t g_ip[4]  = W5500_CFG_IP;
static uint8_t g_sub[4] = W5500_CFG_SUB;
static uint8_t g_gw[4]  = W5500_CFG_GW;

int8_t W5500_ConfigNetwork(void)
{
    if (!g_chip_ok) return W5500_ERR_SPI;

    W_CommonBuf(REG_GAR,  g_gw,  4);
    W_CommonBuf(REG_SUBR, g_sub, 4);
    W_CommonBuf(REG_SHAR, g_mac, 6);
    W_CommonBuf(REG_SIPR, g_ip,  4);

    /* 验证: MAC首字节非0 */
    if (R_Common(REG_SHAR) == 0 && R_Common(REG_SHAR+1) == 0)
        return W5500_ERR_SPI;
    return W5500_OK;
}

int8_t W5500_SetNetParam(uint8_t *mac, uint8_t *ip, uint8_t *sub, uint8_t *gw)
{
    if (mac) { memcpy(g_mac, mac, 6); W_CommonBuf(REG_SHAR, mac, 6); }
    if (ip)  { memcpy(g_ip,  ip,  4); W_CommonBuf(REG_SIPR, ip,  4); }
    if (sub) { memcpy(g_sub, sub, 4); W_CommonBuf(REG_SUBR, sub, 4); }
    if (gw)  { memcpy(g_gw,  gw,  4); W_CommonBuf(REG_GAR,  gw,  4); }
    return W5500_OK;
}

uint8_t W5500_LinkUp(void) { return (R_Common(REG_PHYCFGR) & 0x01) ? 1 : 0; }

/* 缓存版 — 由 W5500_TCPServer_Run 每 5ms 更新 g_phy_linked，不触 SPI */
uint8_t W5500_IsLinkUpCached(void) { return g_phy_linked; }

/* ===================== Socket ===================== */
static int8_t SocketCmd(uint8_t sock, uint8_t cmd, uint8_t expect_sr)
{
    uint32_t to = (uint32_t)SOCK_CMD_TIMEOUT_MS * 1000;
    W_Sock(sock, OFF_SN_CR, cmd);
    while (--to) {
        uint8_t sr = R_Sock(sock, OFF_SN_SR);
        if (sr == expect_sr) { g_socket_status = sr; return W5500_OK; }
        if (sr == SOCK_CLOSED && expect_sr != SOCK_CLOSED) break;
        Delay_us(100);
    }
    g_socket_status = R_Sock(sock, OFF_SN_SR);
    return W5500_ERR_TIMEOUT;
}

/* ===================== TCP Server ===================== */
int8_t W5500_TCPServer_Start(uint16_t port)
{
    if (!g_chip_ok) return W5500_ERR_SPI;
    W_Sock2(0, OFF_SN_PORT, port);
    W_Sock(0, OFF_SN_MR, 0x01);
    if (SocketCmd(0, Sn_CR_OPEN, SOCK_INIT))   return W5500_ERR_TIMEOUT;
    if (SocketCmd(0, Sn_CR_LISTEN, SOCK_LISTEN)) return W5500_ERR_TIMEOUT;
    return W5500_OK;
}

void W5500_TCPServer_Run(void)
{
    if (!g_chip_ok) { W5500_Recovery(); return; }

    /* 周期校验: 芯片被外部复位(电源抖动)后 SPI 仍活但 IP 配置清零,
     * g_chip_ok 保持 1 走不进 Recovery → 每 1s 校验 SIPR 前两字节.
     * (R_Common 超时自身也会清 g_chip_ok, 故下方统一复查) */
    if (Delay_GetTick() - g_cfg_verify_tick > 1000) {
        g_cfg_verify_tick = Delay_GetTick();
        if (R_Common(REG_SIPR) == 0 && R_Common(REG_SIPR + 1) == 0)
            g_chip_ok = 0;              /* 配置被清 → 下次循环走 Recovery */
        if (!g_chip_ok) return;
    }

    uint8_t sr = R_Sock(0, OFF_SN_SR);
    if (!g_chip_ok) return;             /* SPI 异常 — 放弃本轮操作 */
    g_socket_status = sr; g_phy_linked = W5500_LinkUp();

    switch (sr) {
    case SOCK_ESTABLISHED: {
        g_w5500_online = 1; g_close_wait_tick = g_closed_tick = 0;
        uint16_t rx_sz = R_Sock2(0, OFF_SN_RX_RSR);
        if (!g_chip_ok) break;          /* SPI 异常 — 停止读取 */
        if (rx_sz > 0) {
            uint16_t rd = R_Sock2(0, OFF_SN_RX_RD);
            uint16_t room = W5500_RX_BUF_SIZE - g_w5500_rx_ring.count;
            if (rx_sz > room) rx_sz = room; if (rx_sz > 512) rx_sz = 512;
            if (rx_sz > 0) {
                uint8_t tmp[512];
                R_SockBuf(0, T_S0_RXBUF, rd & 0x7FF, tmp, rx_sz);
                rd += rx_sz; W_Sock2(0, OFF_SN_RX_RD, rd);
                SocketCmd(0, Sn_CR_RECV, SOCK_ESTABLISHED);
                RingBuf_Push(&g_w5500_rx_ring, tmp, rx_sz);
            }
        }
        break;
    }
    case SOCK_CLOSE_WAIT:
        if (Delay_GetTick() - g_close_wait_tick > 200) {
            g_close_wait_tick = Delay_GetTick();
            SocketCmd(0, Sn_CR_DISCON, SOCK_CLOSED);
            SocketCmd(0, Sn_CR_CLOSE, SOCK_CLOSED);
        }
        break;
    case SOCK_CLOSED:
        g_w5500_online = 0;
        if (Delay_GetTick() - g_closed_tick > 500)
            { g_closed_tick = Delay_GetTick(); W5500_TCPServer_Start(MODBUS_PORT); }
        break;
    }
}

uint8_t W5500_IsConnected(void)
    { return (g_socket_status == SOCK_ESTABLISHED && g_w5500_online) ? 1 : 0; }

/* ===================== 发送数据 ===================== */
int8_t W5500_SendData(uint8_t *buf, uint16_t len)
{
    if (!len) return W5500_OK;
    if (!buf || len > 1460) return W5500_ERR_PARAM;
    if (!g_w5500_online || g_socket_status != SOCK_ESTABLISHED) return W5500_ERR_NOCONN;

    uint32_t to = 10000; uint16_t free_sz;
    do { free_sz = R_Sock2(0, OFF_SN_TX_FSR); if (free_sz >= len) break; Delay_us(100); } while (--to);
    if (free_sz < len) return W5500_ERR_TIMEOUT;

    uint16_t wr = R_Sock2(0, OFF_SN_TX_WR);
    W_SockBuf(0, T_S0_TXBUF, wr & 0x7FF, buf, len);
    wr += len; W_Sock2(0, OFF_SN_TX_WR, wr);
    if (SocketCmd(0, Sn_CR_SEND, SOCK_ESTABLISHED)) return W5500_ERR_TIMEOUT;
    return W5500_OK;
}

/* ===================== 环形缓冲 ===================== */
void RingBuf_Init(RingBuf_t *r)   { r->head=r->tail=r->count=0; }
uint16_t RingBuf_Available(RingBuf_t *r) { return r->count; }
int8_t RingBuf_Push(RingBuf_t *r, uint8_t *d, uint16_t n) {
    if (r->count+n > W5500_RX_BUF_SIZE) return W5500_ERR_PARAM;
    for (uint16_t i=0;i<n;i++){r->buf[r->head]=d[i];r->head=(r->head+1)%W5500_RX_BUF_SIZE;}
    r->count+=n; return W5500_OK;
}
int16_t RingBuf_Pop(RingBuf_t *r, uint8_t *o, uint16_t m) {
    if (!r->count||!m) return 0;
    uint16_t n=(r->count<m)?r->count:m;
    for (uint16_t i=0;i<n;i++){o[i]=r->buf[r->tail];r->tail=(r->tail+1)%W5500_RX_BUF_SIZE;}
    r->count-=n; return (int16_t)n;
}
void RingBuf_Clear(RingBuf_t *r) { r->head=r->tail=r->count=0; }
