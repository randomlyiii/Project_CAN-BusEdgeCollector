#include "ModbusTCP.h"
#include "W5500.h"
#include "CAN_User.h"
#include <string.h>

/* ===================== 全局状态 ===================== */
volatile uint16_t g_modbus_rx_errs  = 0;
volatile uint8_t  g_modbus_offline  = 0;

static uint16_t g_regs[MODBUS_REG_COUNT];
static uint8_t  g_resp_buf[260];
static uint8_t  g_work_buf[512];     /* 工作缓冲区(分包拼接) */

void ModbusTCP_Init(void)
{
    memset(g_regs, 0, sizeof(g_regs));
    g_modbus_rx_errs = 0;
    g_modbus_offline = 1;
}

/* ===================== 从CAN同步寄存器 ===================== */
void ModbusTCP_SyncFromCAN(void)
{
    if (g_slave_nodes[0].node_id != 0) {
        g_regs[REG_SLAVE1_TEMP]   = (uint16_t)g_slave_nodes[0].temp_int * 10
                                    + g_slave_nodes[0].temp_dec;
        g_regs[REG_SLAVE1_HUMI]   = (uint16_t)g_slave_nodes[0].humi_int * 10
                                    + g_slave_nodes[0].humi_dec;
        g_regs[REG_SLAVE1_HB_CNT] = g_slave_nodes[0].heartbeat_count;
        g_regs[REG_SLAVE1_FAULT]  = g_slave_nodes[0].fault_flag;
        g_regs[REG_SLAVE1_ONLINE] = g_slave_nodes[0].online;
    }
    if (g_slave_nodes[1].node_id != 0) {
        g_regs[REG_SLAVE2_TEMP]   = (uint16_t)g_slave_nodes[1].temp_int * 10
                                    + g_slave_nodes[1].temp_dec;
        g_regs[REG_SLAVE2_HUMI]   = (uint16_t)g_slave_nodes[1].humi_int * 10
                                    + g_slave_nodes[1].humi_dec;
        g_regs[REG_SLAVE2_HB_CNT] = g_slave_nodes[1].heartbeat_count;
        g_regs[REG_SLAVE2_FAULT]  = g_slave_nodes[1].fault_flag;
        g_regs[REG_SLAVE2_ONLINE] = g_slave_nodes[1].online;
    }
    g_regs[REG_CAN_BUS_LOAD]  = g_can_error.bus_load;
    g_regs[REG_CAN_TEC]       = g_can_error.tec;
    g_regs[REG_CAN_REC]       = g_can_error.rec;
    g_regs[REG_CAN_ERR_LEVEL] = g_can_error.error_level;
    g_regs[REG_NET_ERR_CNT]   = g_modbus_rx_errs;

    g_regs[REG_ONLINE_NODE_MASK] = 0;
    for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].online)
            g_regs[REG_ONLINE_NODE_MASK] |= (1 << (g_slave_nodes[i].node_id - 1));
    }
}

/* ===================== 异常响应 ===================== */
static uint16_t BuildException(uint8_t fc, uint8_t excode, uint8_t *buf)
{
    buf[MBAP_UID_IDX]     = 0xFF;
    buf[PDU_FC_IDX]       = fc | 0x80;
    buf[PDU_FC_IDX + 1]   = excode;
    return MBAP_HEADER_LEN + 2;
}

/* ===================== 0x03 读寄存器 ===================== */
static uint16_t HandleReadRegs(uint8_t *req, uint16_t req_len, uint8_t *resp)
{
    (void)req_len;
    uint16_t start = ((uint16_t)req[PDU_ADDR_IDX] << 8) | req[PDU_ADDR_IDX + 1];
    uint16_t cnt   = ((uint16_t)req[PDU_DATA_IDX] << 8) | req[PDU_DATA_IDX + 1];

    if (start + cnt > MODBUS_REG_COUNT || cnt == 0 || cnt > 125)
        return BuildException(FC_READ_HOLDING_REGS, EX_ILLEGAL_ADDRESS, resp);

    resp[MBAP_UID_IDX]     = req[MBAP_UID_IDX];
    resp[PDU_FC_IDX]       = FC_READ_HOLDING_REGS;
    resp[PDU_FC_IDX + 1]   = (uint8_t)(cnt * 2);

    for (uint16_t i = 0; i < cnt; i++) {
        uint16_t v = g_regs[start + i];
        resp[RESP_DATA_IDX + i * 2]     = (uint8_t)(v >> 8);
        resp[RESP_DATA_IDX + i * 2 + 1] = (uint8_t)(v & 0xFF);
    }
    return MBAP_HEADER_LEN + 2 + cnt * 2;
}

/* ===================== 0x06 写单寄存器 ===================== */
static uint16_t HandleWriteReg(uint8_t *req, uint16_t req_len, uint8_t *resp)
{
    (void)req_len;
    uint16_t addr = ((uint16_t)req[PDU_ADDR_IDX] << 8) | req[PDU_ADDR_IDX + 1];
    uint16_t val  = ((uint16_t)req[PDU_DATA_IDX] << 8) | req[PDU_DATA_IDX + 1];

    if (addr >= MODBUS_REG_COUNT)
        return BuildException(FC_WRITE_SINGLE_REG, EX_ILLEGAL_ADDRESS, resp);

    if (addr == REG_CTRL_RESET && val == 0x55) {
        CAN_ResetBus();
    } else if (addr == REG_CTRL_UNBLACKLIST && val > 0 && val <= MAX_SLAVE_NODES) {
        g_slave_nodes[val - 1].blacklist = 0;
        g_slave_nodes[val - 1].online = 1;
    } else if (addr >= 0x0020) {
        g_regs[addr] = val;
    }
    /* 数据区(0x0000~0x001F)只读, 静默丢弃 */

    memcpy(resp, req, req_len);
    return req_len;
}

/* ===================== 主处理流程 ===================== */
void ModbusTCP_Process(void)
{
    uint16_t avail;
    int16_t  n;
    uint8_t  fc;
    uint16_t tid, resp_len;
    uint16_t mbap_len;
    uint8_t  *p;

    avail = RingBuf_Available(&g_w5500_rx_ring);
    if (avail == 0) return;

    /* 从环形缓冲区读数据 */
    n = RingBuf_Pop(&g_w5500_rx_ring, g_work_buf, sizeof(g_work_buf));
    if (n < MBAP_HEADER_LEN + 2) {
        g_modbus_rx_errs++;
        return;
    }

    /* 校验协议ID */
    if (g_work_buf[MBAP_PID_IDX] != 0x00 || g_work_buf[MBAP_PID_IDX + 1] != 0x00) {
        g_modbus_rx_errs++;
        return;
    }

    /* MBAP长度 */
    mbap_len = ((uint16_t)g_work_buf[MBAP_LEN_IDX] << 8)
             | g_work_buf[MBAP_LEN_IDX + 1];
    if (mbap_len + MBAP_HEADER_LEN - 1 > (uint16_t)n) {
        /* 帧不完整, 放回 */
        RingBuf_Push(&g_w5500_rx_ring, g_work_buf, (uint16_t)n);
        return;
    }

    tid = ((uint16_t)g_work_buf[MBAP_TID_IDX] << 8)
        | g_work_buf[MBAP_TID_IDX + 1];
    fc  = g_work_buf[PDU_FC_IDX];

    switch (fc) {
    case FC_READ_HOLDING_REGS:
        resp_len = HandleReadRegs(g_work_buf, (uint16_t)n, g_resp_buf);
        break;
    case FC_WRITE_SINGLE_REG:
        resp_len = HandleWriteReg(g_work_buf, (uint16_t)n, g_resp_buf);
        break;
    default:
        resp_len = BuildException(fc, EX_ILLEGAL_FUNCTION, g_resp_buf);
        g_modbus_rx_errs++;
        break;
    }

    /* 填MBAP头 */
    g_resp_buf[MBAP_TID_IDX]     = (uint8_t)(tid >> 8);
    g_resp_buf[MBAP_TID_IDX + 1] = (uint8_t)(tid & 0xFF);
    g_resp_buf[MBAP_PID_IDX]     = 0x00;
    g_resp_buf[MBAP_PID_IDX + 1] = 0x00;
    g_resp_buf[MBAP_LEN_IDX]     = 0x00;
    g_resp_buf[MBAP_LEN_IDX + 1] = (uint8_t)(resp_len - MBAP_HEADER_LEN + 1);

    /* 发送前检查TCP连接状态 */
    if (W5500_IsConnected()) {
        W5500_SendData(g_resp_buf, resp_len);
        g_modbus_offline = 0;
    } else {
        RingBuf_Clear(&g_w5500_rx_ring);
        g_modbus_offline = 1;
    }
}
