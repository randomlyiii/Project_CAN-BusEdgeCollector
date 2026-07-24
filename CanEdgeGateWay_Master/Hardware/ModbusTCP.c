/**
 * Modbus TCP Server — Phase 2 with dual-zone cache + disconnect strategies
 */

#include "ModbusTCP.h"
#include "W5500.h"
#include "CAN_User.h"
#include "fifo.h"
#include "delay.h"
#include <string.h>

/* ---- Register array ---- */
static uint16_t g_regs[MODBUS_REG_COUNT];
static uint8_t  g_resp_buf[260];
static uint8_t  g_work_buf[512];

/* ---- Dual-zone cache ---- */
static ModbusCache g_cache;

/* ---- Global state ---- */
volatile uint16_t g_modbus_rx_errs  = 0;
volatile uint8_t  g_modbus_offline  = 1;
static uint32_t   g_sys_uptime      = 0;
static uint32_t   g_last_uptime_tick = 0;

/* ---- Forward declarations ---- */
static uint16_t BuildException(uint8_t fc, uint8_t excode, uint8_t *buf);
static uint16_t HandleReadRegs(uint8_t *req, uint16_t req_len, uint8_t *resp);
static uint16_t HandleWriteReg(uint8_t *req, uint16_t req_len, uint8_t *resp);

/* ==================== Init ==================== */

void ModbusTCP_Init(void)
{
    memset(g_regs, 0, sizeof(g_regs));
    memset(&g_cache, 0, sizeof(g_cache));
    g_modbus_rx_errs = 0;
    g_modbus_offline = 1;
    g_cache.was_disconnected = 0;
}

/* ==================== Sync from CAN (called at 50ms period) ==================== */

void ModbusTCP_SyncFromCAN(void)
{
    uint8_t i;

    /* Slave 1 */
    if (g_slave_nodes[0].node_id != 0) {
        g_regs[REG_SLAVE1_TEMP]   = (uint16_t)g_slave_nodes[0].temp_int * 10
                                    + g_slave_nodes[0].temp_dec;
        g_regs[REG_SLAVE1_HUMI]   = (uint16_t)g_slave_nodes[0].humi_int * 10
                                    + g_slave_nodes[0].humi_dec;
        g_regs[REG_SLAVE1_HB_CNT] = g_slave_nodes[0].heartbeat_count;
        g_regs[REG_SLAVE1_FAULT]  = g_slave_nodes[0].fault_flag;
        g_regs[REG_SLAVE1_ONLINE] = g_slave_nodes[0].online;
    }
    /* Slave 2 */
    if (g_slave_nodes[1].node_id != 0) {
        g_regs[REG_SLAVE2_TEMP]   = (uint16_t)g_slave_nodes[1].temp_int * 10
                                    + g_slave_nodes[1].temp_dec;
        g_regs[REG_SLAVE2_HUMI]   = (uint16_t)g_slave_nodes[1].humi_int * 10
                                    + g_slave_nodes[1].humi_dec;
        g_regs[REG_SLAVE2_HB_CNT] = g_slave_nodes[1].heartbeat_count;
        g_regs[REG_SLAVE2_FAULT]  = g_slave_nodes[1].fault_flag;
        g_regs[REG_SLAVE2_ONLINE] = g_slave_nodes[1].online;
    }

    /* CAN diagnostics */
    g_regs[REG_CAN_BUS_LOAD]  = g_can_error.bus_load;
    g_regs[REG_CAN_TEC]       = g_can_error.tec;
    g_regs[REG_CAN_REC]       = g_can_error.rec;
    g_regs[REG_CAN_ERR_LEVEL] = g_can_error.error_level;
    g_regs[REG_NET_ERR_CNT]   = g_modbus_rx_errs;

    /* Extended diagnostics (Phase 2) */
    g_regs[REG_FIFO_HIGH_COUNT]    = FIFO_High_Count();
    g_regs[REG_FIFO_NORMAL_COUNT]  = FIFO_Normal_Count();
    g_regs[REG_FIFO_HIGH_OVERFLOW] = g_fifo_high_overflow_cnt;
    g_regs[REG_FIFO_NORMAL_OVERFLOW] = g_fifo_normal_overflow_cnt;
    g_regs[REG_BUSOFF_RECOVERY]    = g_bus_off_recovery_cnt;
    g_regs[REG_HIST_CACHE_COUNT]   = g_cache.hist_count;
    g_regs[REG_THROTTLE_LEVEL]     = g_system_throttle_level;

    /* Uptime */
    {
        uint32_t now = Delay_GetTick();
        if (now - g_last_uptime_tick >= 1000) {
            g_sys_uptime++;
            g_last_uptime_tick = now;
        }
        g_regs[REG_SYS_UPTIME_HIGH] = (uint16_t)(g_sys_uptime >> 16);
        g_regs[REG_SYS_UPTIME_LOW]  = (uint16_t)(g_sys_uptime & 0xFFFF);
    }

    /* Online node mask */
    g_regs[REG_ONLINE_NODE_MASK] = 0;
    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].online)
            g_regs[REG_ONLINE_NODE_MASK] |= (1 << (g_slave_nodes[i].node_id - 1));
    }
}

/* ==================== Cache management ==================== */

void ModbusTCP_OnDisconnect(void)
{
    if (!g_cache.was_disconnected) {
        g_cache.disconnect_start_tick = Delay_GetTick();
        g_cache.was_disconnected = 1;
    }
}

void ModbusTCP_OnReconnect(void)
{
    if (g_cache.was_disconnected) {
        uint32_t offline_duration = Delay_GetTick() - g_cache.disconnect_start_tick;
        g_cache.was_disconnected = 0;
        g_cache.batch_sent       = 0;

        if (offline_duration >= LONG_DISCONNECT_MS) {
            /* Long disconnect: purge level-2 (low-freq) entries from history */
            uint8_t i, write = 0;
            for (i = 0; i < g_cache.hist_count; i++) {
                uint8_t idx = (g_cache.hist_tail + i) % HIST_CACHE_MAX;
                if (g_cache.history[idx].priority != 2) {
                    if (write != i) {
                        g_cache.history[(g_cache.hist_tail + write) % HIST_CACHE_MAX]
                            = g_cache.history[idx];
                    }
                    write++;
                }
            }
            g_cache.hist_count = write;
            g_cache.hist_head  = (g_cache.hist_tail + write) % HIST_CACHE_MAX;
        }
        /* Short disconnect: keep everything, batch upload will handle */
    }
}

uint8_t ModbusTCP_HasPendingBatch(void)
{
    return (g_cache.hist_count > g_cache.batch_sent) ? 1 : 0;
}

uint16_t ModbusTCP_BatchUpload(void)
{
    /* Note: batch upload is triggered from vTask_Protocol.
       The actual upload happens as part of the protocol layer.
       This returns the count of pending entries to upload. */
    return g_cache.hist_count - g_cache.batch_sent;
}

/* ==================== Modbus Exception ==================== */

static uint16_t BuildException(uint8_t fc, uint8_t excode, uint8_t *buf)
{
    buf[MBAP_UID_IDX]     = 0xFF;
    buf[PDU_FC_IDX]       = fc | 0x80;
    buf[PDU_FC_IDX + 1]   = excode;
    return MBAP_HEADER_LEN + 2;
}

/* ==================== 0x03 Read Holding Registers ==================== */

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

/* ==================== 0x06 Write Single Register ==================== */

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
        CAN_ManualDeescalate(val);
    } else if (addr >= 0x0020) {
        g_regs[addr] = val;
    }
    /* Data zone (0x0000–0x001F) is read-only */

    memcpy(resp, req, req_len);
    return req_len;
}

/* ==================== Main Modbus TCP processing ==================== */

void ModbusTCP_Process(void)
{
    uint16_t avail;
    int16_t  n;
    uint8_t  fc;
    uint16_t tid, resp_len;
    uint16_t mbap_len;

    avail = RingBuf_Available(&g_w5500_rx_ring);
    if (avail == 0) return;

    n = RingBuf_Pop(&g_w5500_rx_ring, g_work_buf, sizeof(g_work_buf));
    if (n < MBAP_HEADER_LEN + 2) {
        g_modbus_rx_errs++;
        return;
    }

    /* Validate protocol ID */
    if (g_work_buf[MBAP_PID_IDX] != 0x00 || g_work_buf[MBAP_PID_IDX + 1] != 0x00) {
        g_modbus_rx_errs++;
        return;
    }

    /* Validate MBAP length */
    mbap_len = ((uint16_t)g_work_buf[MBAP_LEN_IDX] << 8)
             | g_work_buf[MBAP_LEN_IDX + 1];
    if (mbap_len + MBAP_HEADER_LEN - 1 > (uint16_t)n) {
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

    /* Fill MBAP header */
    g_resp_buf[MBAP_TID_IDX]     = (uint8_t)(tid >> 8);
    g_resp_buf[MBAP_TID_IDX + 1] = (uint8_t)(tid & 0xFF);
    g_resp_buf[MBAP_PID_IDX]     = 0x00;
    g_resp_buf[MBAP_PID_IDX + 1] = 0x00;
    g_resp_buf[MBAP_LEN_IDX]     = 0x00;
    g_resp_buf[MBAP_LEN_IDX + 1] = (uint8_t)(resp_len - MBAP_HEADER_LEN + 1);

    if (W5500_IsConnected()) {
        W5500_SendData(g_resp_buf, resp_len);
        g_modbus_offline = 0;
    } else {
        RingBuf_Clear(&g_w5500_rx_ring);
        g_modbus_offline = 1;
    }
}
