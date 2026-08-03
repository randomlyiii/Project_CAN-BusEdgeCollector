1.主站 OLED 切页时间 = Modbus 寄存器 REG_OLED_AUTO_RETURN_MS（0x002C），内存里没直接记文件，但代码在：

  改默认值（编译期）：CanEdgeGateWay_Master\Hardware\ModbusTCP.c:36
  g_regs[REG_OLED_AUTO_RETURN_MS] = 10000;   /* OLED 自动回主页超时 (ms) */
  改 10000 即改默认。
