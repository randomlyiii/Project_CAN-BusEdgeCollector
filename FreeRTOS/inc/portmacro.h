/*
 * FreeRTOS V10.4.6 - ARM Cortex-M3 port for STM32F103
 */

#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Type definitions */
#define portCHAR          char
#define portFLOAT         float
#define portDOUBLE        double
#define portLONG          long
#define portSHORT         short
#define portSTACK_TYPE    uint32_t
#define portBASE_TYPE     long
#define portMAX_DELAY     ((TickType_t)0xFFFFFFFFUL)

typedef uint32_t          StackType_t;
typedef long              BaseType_t;
typedef unsigned long     UBaseType_t;
typedef uint32_t          TickType_t;

#define portSTACK_GROWTH            (-1)
#define portTICK_PERIOD_MS          ((TickType_t)1000 / configTICK_RATE_HZ)
#define portBYTE_ALIGNMENT          8
#define portDONT_DISCARD

/* NVIC register access */
#define portNVIC_INT_CTRL_REG       (*((volatile uint32_t *)0xE000ED04UL))
#define portNVIC_PENDSVSET_BIT      (1UL << 28UL)
#define portNVIC_SYSTICK_CTRL_REG   (*((volatile uint32_t *)0xE000E010UL))
#define portNVIC_SYSTICK_COUNT_REG  (*((volatile uint32_t *)0xE000E018UL))

/* Critical section */
#define portDISABLE_INTERRUPTS()    __disable_irq()
#define portENABLE_INTERRUPTS()     __enable_irq()

extern void vPortEnterCritical(void);
extern void vPortExitCritical(void);

#define portENTER_CRITICAL()        vPortEnterCritical()
#define portEXIT_CRITICAL()         vPortExitCritical()

#define portSET_INTERRUPT_MASK_FROM_ISR()     __get_PRIMASK()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)  __set_PRIMASK(x)

/* Yield / context switch */
extern void vPortYield(void);
#define portYIELD()                 vPortYield()
#define portYIELD_FROM_ISR(x)       portEND_SWITCHING_ISR(x)
#define portEND_SWITCHING_ISR(x)    { if ((x) != pdFALSE) { portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT; } }
#define portYIELD_WITHIN_API()      portYIELD()

/* Scheduler start / suspend */
extern void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime);
#define portSUPPRESS_TICKS_AND_SLEEP(x)  vPortSuppressTicksAndSleep(x)

/* Kernel interrupt priority */
#define portKERNEL_INTERRUPT_PRIORITY           (configKERNEL_INTERRUPT_PRIORITY)
#define portMAX_SYSCALL_INTERRUPT_PRIORITY      (configMAX_SYSCALL_INTERRUPT_PRIORITY)

/* Architecture specifics */
#define portNOP()       __NOP()
#define portINLINE      __inline
#define portFORCE_INLINE __forceinline static

/* Task function macros */
#define portTASK_FUNCTION_PROTO(vFunction, pvParameters)    void vFunction(void *pvParameters)
#define portTASK_FUNCTION(vFunction, pvParameters)          void vFunction(void *pvParameters)

/* Scheduler entry point */
BaseType_t xPortStartScheduler(void);
void vPortEndScheduler(void);

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
