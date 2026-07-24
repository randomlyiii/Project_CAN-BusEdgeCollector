/**
 * @file    sample_config.h
 * @brief   W5500 网络配置模板
 *
 * 使用方法:
 *   1. 复制此文件为 config.h
 *   2. 根据实际网络环境修改 config.h 中的 IP/MAC
 *   3. config.h 已被 .gitignore 忽略, 不会提交到仓库
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* ===================== W5500 网络配置 ===================== */

/*
 * MAC 地址 — 必须全球唯一
 *   私有试验网段建议 02:xx:xx:xx:xx:xx (Locally Administered)
 */
#define W5500_CFG_MAC  {0x02, 0x00, 0x00, 0x00, 0x00, 0x01}

/*
 * IP 地址 — 与 PC 在同一网段即可
 *   例: PC 是 192.168.1.100, 这里填 192.168.1.200
 */
#define W5500_CFG_IP   {192, 168, 1, 200}

/*
 * 子网掩码 — 通常为 255.255.255.0
 */
#define W5500_CFG_SUB  {255, 255, 255, 0}

/*
 * 默认网关 — 一般填路由器地址
 */
#define W5500_CFG_GW   {192, 168, 1, 1}

#endif /* __CONFIG_H */
