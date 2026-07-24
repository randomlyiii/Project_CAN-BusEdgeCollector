/*
 * FreeRTOS V10.4.6
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 * ARM Cortex-M3 port for STM32F103 - Keil MDK-ARM compiler
 */

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* STM32 CMSIS includes. */
#include "stm32f10x.h"

/*-----------------------------------------------------------*/

/* Constants required to manipulate the NVIC. */
#define portNVIC_SYSTICK_CTRL_REG           ( * ( ( volatile uint32_t * ) 0xE000E010 ) )
#define portNVIC_SYSTICK_LOAD_REG           ( * ( ( volatile uint32_t * ) 0xE000E014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG  ( * ( ( volatile uint32_t * ) 0xE000E018 ) )
#define portNVIC_SYSPRI2_REG                ( * ( ( volatile uint32_t * ) 0xE000ED1C ) )
#define portNVIC_SYSPRI1_REG                ( * ( ( volatile uint32_t * ) 0xE000ED18 ) )
#define portNVIC_SYS_CTRL_STATE_REG         ( * ( ( volatile uint32_t * ) 0xE000ED24 ) )
#define portNVIC_MEM_FAULT_ENABLE_REG       ( * ( ( volatile uint32_t * ) 0xE000ED28 ) )
#define portNVIC_SYS_CTRL_STATE_FAULT_MASK  0x70000

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR                    ( 0x01000000 )
#define portINITIAL_EXEC_RETURN             ( 0xFFFFFFFD )

/* The offset used to get the next stack pointer after a context switch to
 * allocate space for saving the remaining registers (those not saved/restored
 * by the hardware). */
/*-----------------------------------------------------------*/

/* Each task maintains its own interrupt status in the critical nesting
 * variable. */
static volatile uint32_t ulCriticalNesting = 0xAAAAAAAAUL;

/*-----------------------------------------------------------*/

/*
 * Setup the timer to generate the tick interrupts.
 */
static void prvSetupTimerInterrupt( void );

/*
 * Exception handlers.
 */
void vPortSVCHandler( void ) __attribute__( ( naked ) );
void xPortPendSVHandler( void ) __attribute__( ( naked ) );
void xPortSysTickHandler( void );

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
static void prvStartFirstTask( void );

/*
 * Used to flag tasks that are delaying or waiting so that they can be woken by
 * a tick interrupt or other event.
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/*
 * The number of SysTick ticks that have occurred since the scheduler was
 * started.  The port uses this to determine if a critical section has taken
 * too long.
 */
volatile uint32_t ulPortTickCount = 0UL;

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                     TaskFunction_t pxCode,
                                     void * pvParameters )
{
    /* Simulate the stack frame as it would be created by a context switch
     * interrupt. */

    /* Offset added to account for the way the MCU uses the stack on entry/exit
     * of interrupts, and to ensure alignment. */
    pxTopOfStack--;

    /* The 8 auto-saved registers by the hardware on exception entry. */
    *pxTopOfStack = portINITIAL_XPSR;           /* xPSR */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pxCode;     /* PC */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) prvTaskExitError; /* LR */
    pxTopOfStack--;

    /* R12, R3, R2 and R1. */
    pxTopOfStack -= 4;

    /* R0 is the parameter. */
    *pxTopOfStack = ( StackType_t ) pvParameters; /* R0 */
    pxTopOfStack--;

    /* The 8 remaining registers to be manually saved on exception entry. */
    /* R11, R10, R9, R8, R7, R6, R5, R4 */
    pxTopOfStack -= 8;

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
    configASSERT( ulCriticalNesting == ~0UL );
    portDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

static void prvStartFirstTask( void )
{
    __asm volatile (
        " ldr r0, =0xE000ED08    \n"  /* Use the NVIC offset register to locate the stack. */
        " ldr r0, [r0]            \n"
        " ldr r0, [r0]            \n"  /* The start of the stack is the first entry in the vector table. */
        " msr msp, r0             \n"  /* Set the msp back to the start of the stack. */
        " cpsie i                 \n"  /* Globally enable interrupts. */
        " cpsie f                 \n"
        " dsb                     \n"
        " isb                     \n"
        " svc 0                   \n"  /* System call to start first task. */
        " nop                     \n"
        " nop                     \n"
    );
}
/*-----------------------------------------------------------*/

void vPortSVCHandler( void )
{
    __asm volatile (
        "   ldr     r3, =pxCurrentTCB              \n"  /* Restore the context. */
        "   ldr     r1, [r3]                       \n"  /* Use pxCurrentTCB to get the task context. */
        "   ldr     r0, [r1]                       \n"  /* The first item in the TCB is the top of stack. */
        "   ldmia   r0!, {r4-r11}                  \n"  /* Pop the registers that are not automatically saved on exception entry. */
        "   msr     psp, r0                        \n"  /* Restore the task stack pointer. */
        "   isb                                     \n"
        "   mov     r0, #0                         \n"
        "   msr     basepri, r0                    \n"  /* Enable all interrupts - no nesting effect. */
        "   orr     r14, r14, #13                  \n"  /* Use the process stack pointer when returning. */
        "   bx      r14                             \n"
    );
}
/*-----------------------------------------------------------*/

void xPortPendSVHandler( void )
{
    /* This is a naked function. */

    __asm volatile (
    "   mrs     r0, psp                           \n"
    "   isb                                         \n"
    "                                               \n"
    "   ldr     r3, =pxCurrentTCB                  \n"  /* Get the location of the current TCB. */
    "   ldr     r2, [r3]                           \n"
    "                                               \n"
    "   stmdb   r0!, {r4-r11}                      \n"  /* Save the remaining registers. */
    "   str     r0, [r2]                           \n"  /* Save the new top of stack into the first member of the TCB. */
    "                                               \n"
    "   stmdb   sp!, {r3, r14}                     \n"  /* Temporarily save R3 and R14 (LR). */
    "   mov     r0, %[ulMaxSyscallPriority]         \n"
    "   msr     basepri, r0                        \n"  /* Mask interrupts at and below the kernel priority. */
    "   dsb                                         \n"
    "   isb                                         \n"
    "   bl      vTaskSwitchContext                  \n"  /* Select the next task. */
    "   mov     r0, #0                             \n"  /* Clear the mask. */
    "   msr     basepri, r0                        \n"
    "   ldmia   sp!, {r3, r14}                     \n"
    "                                               \n"
    "   ldr     r1, [r3]                           \n"  /* The first item in pxCurrentTCB is the task top of stack. */
    "   ldr     r0, [r1]                           \n"
    "                                               \n"
    "   ldmia   r0!, {r4-r11}                      \n"  /* Pop the registers and the critical nesting count. */
    "   msr     psp, r0                            \n"
    "   isb                                         \n"
    "   bx      r14                                 \n"
    "   .align  4                                   \n"
    ::"[ulMaxSyscallPriority]""i"( configMAX_SYSCALL_INTERRUPT_PRIORITY )
    );
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
    /* The SysTick runs at the lowest interrupt priority, so when this interrupt
     * executes all interrupts must be unmasked.  There is therefore no need to
     * save and then restore the interrupt mask value as its value is already
     * known. */
    portDISABLE_INTERRUPTS();
    {
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            /* A context switch is required.  Context switching is performed in
             * the PendSV interrupt.  Pend the PendSV interrupt. */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
    }
    portENABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/

static void prvSetupTimerInterrupt( void )
{
    /* Configure SysTick to interrupt at the requested rate. */
    portNVIC_SYSTICK_LOAD_REG = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
    portNVIC_SYSTICK_CTRL_REG = ( 1UL << 2UL ) | ( 1UL << 1UL ) | 1UL;

    /* Set SysTick priority to the lowest possible. */
    portNVIC_SYSPRI2_REG |= ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 24UL;

    /* Also set PendSV priority to the lowest. */
    portNVIC_SYSPRI2_REG |= ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 16UL;
}
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
    /* Make PendSV and SysTick the lowest priority interrupts. */
    portNVIC_SYSPRI2_REG |= portNVIC_PENDSV_PRI;
    portNVIC_SYSPRI2_REG |= portNVIC_SYSTICK_PRI;

    /* Start the timer that generates the tick ISR. */
    prvSetupTimerInterrupt();

    /* Initialise the critical nesting count ready for the first task. */
    ulCriticalNesting = 0;

    /* Start the first task. */
    prvStartFirstTask();

    /* Should never reach here. */
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* Not implemented in ports where there is no way to get back to the
     * following the scheduler.  This function must be implemented for
     * completeness. */
    configASSERT( ulCriticalNesting == 1000UL );

    portDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    ulCriticalNesting++;

    /* This is not the interrupt safe version of the enter critical function so
     * assert() if it is being called from an interrupt context.  Only API
     * functions that end in "FromISR" can be used in an interrupt.  Only assert
     * if the critical nesting count is 1 to protect against recursive calls if
     * the assert function also uses a critical section. */
    if( ulCriticalNesting == 1 )
    {
        configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
    }
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
    configASSERT( ulCriticalNesting > 0 );
    ulCriticalNesting--;

    if( ulCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/*-----------------------------------------------------------*/

void vPortYield( void )
{
    /* Set a PendSV to request a context switch. */
    portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

    /* Barriers are normally not required but do ensure the code is
     * completely within the specified behaviour for the architecture. */
    __asm volatile ( "dsb" ::: "memory" );
    __asm volatile ( "isb" );
}
/*-----------------------------------------------------------*/

#if( configUSE_TICKLESS_IDLE == 1 )

    void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
    {
        uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements;
        TickType_t xModifiableIdleTime;

        /* Make sure the SysTick reload value does not overflow the counter. */
        if( xExpectedIdleTime > configEXPECTED_IDLE_TIME_BEFORE_SLEEP )
        {
            xExpectedIdleTime = configEXPECTED_IDLE_TIME_BEFORE_SLEEP;
        }

        portDISABLE_INTERRUPTS();

        /* If it is executing this function then it is likely that the scheduler
         * is not running and a tick has just occurred. */

        /* Calculate the reload value required to wait xExpectedIdleTime
         * tick periods. */
        ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG +
            ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) * xExpectedIdleTime;

        if( ulReloadValue > portNVIC_SYSTICK_LOAD_REG )
        {
            ulReloadValue = portNVIC_SYSTICK_LOAD_REG;
        }

        /* Enter a critical section but do not use the taskENTER_CRITICAL()
         * macro as that has an effect on the critical nesting variable. */
        __asm volatile ( "cpsid i" ::: "memory" );
        __asm volatile ( "dsb" );
        __asm volatile ( "isb" );

        /* If a context switch is pending then abandon the low power entry as
         * the context switch might have been pended by an external interrupt
         * that requires processing. */
        if( eTaskConfirmSleepModeStatus() == eAbortSleep )
        {
            /* Restart tick. */
            portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;
            portNVIC_SYSTICK_CURRENT_VALUE_REG = 0;
            portNVIC_SYSTICK_LOAD_REG = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
            portNVIC_SYSTICK_CTRL_REG |= ( 1 << 1 ) | 1;

            /* Re-enable interrupts - see comments above the cpsid instruction()
             * above. */
            __asm volatile ( "cpsie i" ::: "memory" );
        }
        else
        {
            /* Set the new reload value. */
            portNVIC_SYSTICK_LOAD_REG = ulReloadValue;
            portNVIC_SYSTICK_CURRENT_VALUE_REG = 0;
            portNVIC_SYSTICK_CTRL_REG |= ( 1 << 1 ) | 1;

            /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
             * set its own interrupts to allow SysTick to wake the CPU. */
            __asm volatile ( "dsb" ::: "memory" );
            __asm volatile ( "wfi" );
            __asm volatile ( "isb" );

            configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

            /* Stop SysTick.  Again, the time the SysTick is stopped is
             * accounted for as best it can be, but using the tickless mode
             * will inevitably result in some tiny drift. */
            ulCompleteTickPeriods = ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG;

            portNVIC_SYSTICK_CTRL_REG &= ~( 1 << 1 );

            portNVIC_SYSTICK_LOAD_REG = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
            portNVIC_SYSTICK_CURRENT_VALUE_REG = 0;

            /* Re-enable interrupts. */
            __asm volatile ( "cpsie i" ::: "memory" );
            __asm volatile ( "dsb" );
            __asm volatile ( "isb" );

            if( ulCompleteTickPeriods > 0 )
            {
                /* Correct the tick count for the number of complete tick periods. */
                vTaskStepTick( ( TickType_t ) ( ulCompleteTickPeriods / ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) ) );
            }
        }
    }

#endif /* configUSE_TICKLESS_IDLE */
/*-----------------------------------------------------------*/

BaseType_t xPortIsInsideInterrupt( void )
{
    uint32_t ulCurrentInterrupt;
    BaseType_t xReturn;

    /* Obtain the number of the currently executing interrupt. */
    ulCurrentInterrupt = ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) >> 8;

    if( ulCurrentInterrupt == 0 )
    {
        xReturn = pdFALSE;
    }
    else
    {
        xReturn = pdTRUE;
    }

    return xReturn;
}
/*-----------------------------------------------------------*/
