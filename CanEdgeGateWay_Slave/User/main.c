#include "stm32f10x.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

int main(void)
{
  // 1. 外设初始化（顺序不可乱）
  Delay_ms(1000);     // 上电延时
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(1, 1, "SYS InitAllRight");
  Delay_ms(1000);

  // 

}
