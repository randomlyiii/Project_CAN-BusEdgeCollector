#include "ModbusTCP.h"
#include "W5500.h"
#include "CAN_User.h"
#include "delay.h"
#include <string.h>

/* ======================== 寄存器表 ======================== */
static uint16_t g_regs[MODBUS_REG_COUNT];

/* ======================== 全局变量 ======================== */
static uint8_t g_resp_buf[256];   // 响应缓冲区

/* ======================== 初始化 ======================== */
void ModbusTCP_Init(void)
{
    memset(g_regs, 0, sizeof(g_regs));
}

/**
  * @brief  更新寄存器值
  * @param  addr  寄存器地址(字偏移)
  * @param  value 值
  */
void ModbusTCP_UpdateReg(uint16_t addr, uint16_t value)
{
    if (addr < MODBUS_REG_COUNT)
        g_regs[addr] = value;
}

/**
  * @brief  更新从站数据到寄存器表
  *         由 main 循环中调用，将 CAN 接收的数据同步到 Modbus 寄存器
  */
void ModbusTCP_SyncFromCAN(void)
{
    /* 从站1 数据 */
    if (g_slave_nodes[0].node_id != 0) {
        uint16_t temp_x10 = (uint16_t)g_slave_nodes[0].temp_int * 10 + g_slave_nodes[0].temp_dec;
        uint16_t humi_x10 = (uint16_t)g_slave_nodes[0].humi_int * 10 + g_slave_nodes[0].humi_dec;

        g_regs[REG_SLAVE1_TEMP]    = temp_x10;
        g_regs[REG_SLAVE1_HUMI]    = humi_x10;
        g_regs[REG_SLAVE1_HB_CNT]  = g_slave_nodes[0].heartbeat_count;
        g_regs[REG_SLAVE1_FAULT]   = g_slave_nodes[0].fault_flag;
        g_regs[REG_SLAVE1_ONLINE]  = g_slave_nodes[0].online;
    }

    /* 从站2 数据 */
    if (g_slave_nodes[1].node_id != 0) {
        uint16_t temp_x10 = (uint16_t)g_slave_nodes[1].temp_int * 10 + g_slave_nodes[1].temp_dec;
        uint16_t humi_x10 = (uint16_t)g_slave_nodes[1].humi_int * 10 + g_slave_nodes[1].humi_dec;

        g_regs[REG_SLAVE2_TEMP]    = temp_x10;
        g_regs[REG_SLAVE2_HUMI]    = humi_x10;
        g_regs[REG_SLAVE2_HB_CNT]  = g_slave_nodes[1].heartbeat_count;
        g_regs[REG_SLAVE2_FAULT]   = g_slave_nodes[1].fault_flag;
        g_regs[REG_SLAVE2_ONLINE]  = g_slave_nodes[1].online;
    }

    /* CAN 状态 */
    g_regs[REG_CAN_BUS_LOAD]     = g_can_error.bus_load;
    g_regs[REG_CAN_TEC]          = g_can_error.tec;
    g_regs[REG_CAN_REC]          = g_can_error.rec;
    g_regs[REG_CAN_ERR_LEVEL]    = g_can_error.error_level;

    /* 在线掩码 */
    g_regs[REG_ONLINE_NODE_MASK] = 0;
    for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].online)
            g_regs[REG_ONLINE_NODE_MASK] |= (1 << (g_slave_nodes[i].node_id - 1));
    }
}

/* ======================== Modbus TCP 处理 ======================== */

/**
  * @brief  构造异常响应
  */
static uint16_t BuildException(uint8_t fc, uint8_t excode, uint8_t *buf)
{
    /* MBAP 头先空着, 最后填充长度 */
    buf[MBAP_UID_IDX] = 0xFF;          // 单元标识
    buf[PDU_FC_IDX]   = fc | 0x80;     // 异常功能码
    buf[PDU_FC_IDX + 1] = excode;
    return MBAP_HEADER_LEN + 2;        // MBAP + FC + 异常码
}

/**
  * @brief  处理 0x03 读保持寄存器
  * @retval 响应长度
  */
static uint16_t HandleReadHoldingRegs(uint8_t *req, uint16_t req_len, uint8_t *resp)
{
    uint16_t start_addr, reg_count;
    uint16_t i, resp_len;

    (void)req_len;

    /* 解析请求: 起始地址 (2B) + 寄存器数量 (2B) */
    start_addr = ((uint16_t)req[PDU_ADDR_IDX] << 8) | req[PDU_ADDR_IDX + 1];
    reg_count  = ((uint16_t)req[PDU_DATA_IDX] << 8) | req[PDU_DATA_IDX + 1];

    /* 合法性检查 */
    if (start_addr + reg_count > MODBUS_REG_COUNT || reg_count == 0 || reg_count > 125)
        return BuildException(FC_READ_HOLDING_REGS, EX_ILLEGAL_ADDRESS, resp);

    /* 构建响应: MBAP头 + FC + 字节数 + 数据 */
    resp[MBAP_UID_IDX] = req[MBAP_UID_IDX];
    resp[PDU_FC_IDX]   = FC_READ_HOLDING_REGS;
    resp[PDU_FC_IDX + 1] = (uint8_t)(reg_count * 2);   // 字节数

    for (i = 0; i < reg_count; i++) {
        uint16_t val = g_regs[start_addr + i];
        resp[PDU_DATA_IDX + i * 2]     = (uint8_t)(val >> 8);
        resp[PDU_DATA_IDX + i * 2 + 1] = (uint8_t)(val & 0xFF);
    }

    resp_len = MBAP_HEADER_LEN + 2 + reg_count * 2;  // MBAP + FC + 字节数 + 数据
    return resp_len;
}

/**
  * @brief  处理 0x06 写单个寄存器
  * @retval 响应长度
  */
static uint16_t HandleWriteSingleReg(uint8_t *req, uint16_t req_len, uint8_t *resp)
{
    uint16_t reg_addr, reg_val;

    (void)req_len;

    reg_addr = ((uint16_t)req[PDU_ADDR_IDX] << 8) | req[PDU_ADDR_IDX + 1];
    reg_val  = ((uint16_t)req[PDU_DATA_IDX] << 8) | req[PDU_DATA_IDX + 1];

    if (reg_addr >= MODBUS_REG_COUNT)
        return BuildException(FC_WRITE_SINGLE_REG, EX_ILLEGAL_ADDRESS, resp);

    /* 控制类寄存器处理 */
    if (reg_addr == REG_CTRL_RESET && reg_val == 0x55) {
        CAN_ResetBus();
    } else if (reg_addr == REG_CTRL_UNBLACKLIST && reg_val > 0 && reg_val <= MAX_SLAVE_NODES) {
        g_slave_nodes[reg_val - 1].blacklist = 0;
        g_slave_nodes[reg_val - 1].online = 1;
    } else if (reg_addr < 0x0020) {
        /* 数据区寄存器只读, 丢弃写入 */
    } else {
        g_regs[reg_addr] = reg_val;   // 控制区可写
    }

    /* 回显请求数据作为响应 */
    memcpy(resp, req, req_len);
    return req_len;
}

/**
  * @brief  检查并处理 Modbus 请求
  *         由 main 循环调用，检查 W5500 接收缓冲区
  */
void ModbusTCP_Process(void)
{
    uint8_t  *rx_buf;
    uint16_t  rx_len;
    uint16_t  resp_len;
    uint8_t   fc;
    uint16_t  tid;

    rx_len = W5500_GetRxLen();
    if (rx_len == 0) return;

    rx_buf = W5500_GetRxBuf();

    /* 最小长度: MBAP(7) + PDU(2: FC+addr) = 9 */
    if (rx_len < 9) {
        W5500_ClrRxLen();
        return;
    }

    /* 检查协议标识 (必须为 0x0000) */
    if (rx_buf[MBAP_PID_IDX] != 0x00 || rx_buf[MBAP_PID_IDX + 1] != 0x00) {
        W5500_ClrRxLen();
        return;
    }

    /* 复制事务标识 */
    tid = ((uint16_t)rx_buf[MBAP_TID_IDX] << 8) | rx_buf[MBAP_TID_IDX + 1];

    /* 解析功能码 */
    fc = rx_buf[PDU_FC_IDX];

    switch (fc) {
    case FC_READ_HOLDING_REGS:
        resp_len = HandleReadHoldingRegs(rx_buf, rx_len, g_resp_buf);
        break;

    case FC_WRITE_SINGLE_REG:
        resp_len = HandleWriteSingleReg(rx_buf, rx_len, g_resp_buf);
        break;

    default:
        resp_len = BuildException(fc, EX_ILLEGAL_FUNCTION, g_resp_buf);
        break;
    }

    /* 填充 MBAP 头: 事务标识 + 协议标识 + 长度 */
    g_resp_buf[MBAP_TID_IDX] = (uint8_t)(tid >> 8);
    g_resp_buf[MBAP_TID_IDX + 1] = (uint8_t)(tid & 0xFF);
    g_resp_buf[MBAP_PID_IDX] = 0x00;
    g_resp_buf[MBAP_PID_IDX + 1] = 0x00;
    /* 后续长度 = 单元标识(1) + PDU 长度(去掉 MBAP 头之后的部分) */
    g_resp_buf[MBAP_LEN_IDX] = 0x00;
    g_resp_buf[MBAP_LEN_IDX + 1] = (uint8_t)(resp_len - MBAP_HEADER_LEN + 1); // +1 for UID

    /* 通过 W5500 发送响应 */
    W5500_SendData(g_resp_buf, resp_len);

    W5500_ClrRxLen();
}
