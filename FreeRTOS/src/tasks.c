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
 * ARM Cortex-M3 / STM32F103 / Keil MDK-ARM
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"

/*-----------------------------------------------------------*/

/*
 * Task notification states per array entry.
 */
#define eNotWaitingNotification      ( ( uint8_t ) 0 )
#define eWaitingNotification         ( ( uint8_t ) 1 )
#define ePendingNotification         ( ( uint8_t ) 2 )

/*
 * Notification action values passed to xTaskGenericNotify().
 */
#define eNoAction                    ( ( uint32_t ) 0 )
#define eSetBits                     ( ( uint32_t ) 1 )
#define eIncrement                   ( ( uint32_t ) 2 )
#define eSetValueWithOverwrite       ( ( uint32_t ) 3 )
#define eSetValueWithoutOverwrite    ( ( uint32_t ) 4 )

/*
 * Used to build a mask from a priority that can be ANDed with
 * uxTopReadyPriority to check whether any task at that priority is ready.
 */
#define taskREADY_PRIORITY_MASK( uxPriority )   ( ( UBaseType_t ) 1 << ( uxPriority ) )

/*
 * The maximum value of uxTopReadyPriority — used when no tasks are ready.
 */
#define taskTOP_READY_PRIORITY_MASK_ALL          ( ( ( UBaseType_t ) 1 << configMAX_PRIORITIES ) - ( UBaseType_t ) 1 )

/*
 * Stack fill byte for stack overflow detection method 2.
 */
#define tskSTACK_FILL_BYTE            ( 0xA5U )

/*
 * Number of bytes per stack overflow check margin at the end of the stack.
 */
#define tskSTACK_OVERFLOW_MARGIN      ( 20U )

/*-----------------------------------------------------------*/

/*
 * Task control block.  The first member (pxTopOfStack) MUST be at offset 0
 * because the port's PendSV handler loads it directly via pointer.
 */
typedef struct tskTaskControlBlock
{
    volatile StackType_t * pxTopOfStack;     /* MUST be first — accessed by port asm. */

    ListItem_t             xStateListItem;   /* For ready / blocked / suspended lists. */
    ListItem_t             xEventListItem;   /* For event lists (queues, semaphores, event groups). */

    UBaseType_t            uxPriority;       /* Current (running) priority. */
    StackType_t           * pxStack;         /* Base (lowest address) of the stack. */
    char                   pcTaskName[ configMAX_TASK_NAME_LEN ];

    UBaseType_t            uxBasePriority;   /* Priority "inherited" from before mutex boosting. */
    uint32_t               ulRunTimeCounter;

    #if ( configUSE_MUTEXES == 1 )
        UBaseType_t        uxMutexesHeld;
    #endif

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        uint32_t           ulNotifiedValue[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
        uint8_t            ucNotifyState[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
    #endif

    #if ( configCHECK_FOR_STACK_OVERFLOW > 0 )
        StackType_t       * pxEndOfStack;     /* Limit address for overflow check. */
    #endif

    #if ( configUSE_NEWLIB_REENTRANT == 1 )
        struct _reent      xNewLibReent;
    #endif
} tskTCB;

typedef tskTCB TCB_t;

/*-----------------------------------------------------------*/

/*
 * The currently executing task.  The port layer reads this directly from
 * assembly, so it must be a global variable named exactly pxCurrentTCB.
 */
TCB_t * volatile pxCurrentTCB = NULL;

/* Ready tasks: one doubly-linked list per priority level. */
static List_t pxReadyTasksLists[ configMAX_PRIORITIES ];

/* Delayed-task lists (two for tick-overflow handling). */
static List_t xDelayedTaskList1;
static List_t xDelayedTaskList2;
static List_t * volatile pxDelayedTaskList;
static List_t * volatile pxOverflowDelayedTaskList;

/* Tasks that became ready while the scheduler was suspended. */
static List_t xPendingReadyList;

/* Tasks that have been deleted but whose memory the idle task must free. */
#if ( INCLUDE_vTaskDelete == 1 )
    static List_t xTasksWaitingTermination;
    static volatile UBaseType_t uxDeletedTasksWaitingCleanUp = ( UBaseType_t ) 0U;
#endif

/* Count of tasks that currently exist. */
static volatile UBaseType_t uxCurrentNumberOfTasks = ( UBaseType_t ) 0U;

/* The system tick counter. */
static volatile TickType_t xTickCount = ( TickType_t ) 0U;

/* The next time a task will be unblocked — used to optimise tick interrupts. */
static volatile TickType_t xNextTaskUnblockTime = ( TickType_t ) 0U;

/* Bitmap of priority levels that have at least one ready task. */
static volatile UBaseType_t uxTopReadyPriority = ( UBaseType_t ) 0U;

/* Scheduler is / is not running. */
static volatile BaseType_t xSchedulerRunning = pdFALSE;

/* Scheduler suspension nesting counter.  0 = not suspended. */
static volatile UBaseType_t uxSchedulerSuspended = ( UBaseType_t ) 0U;

/* Set to pdTRUE if a yield was requested while the scheduler was suspended. */
static volatile BaseType_t xYieldPending = pdFALSE;

/* Tracks the number of times the tick counter has overflowed (for 32-bit ticks). */
static volatile BaseType_t xNumOfOverflows = ( BaseType_t ) 0;

/* A monotonically incrementing number assigned to each created task (for debug). */
static UBaseType_t uxTaskNumber = ( UBaseType_t ) 0U;

/* Handle to the idle task — returned by xTaskGetIdleTaskHandle(). */
static TaskHandle_t xIdleTaskHandle = NULL;

/* Used by tickless-idle mode to accumulate pending ticks. */
#if ( configUSE_TICKLESS_IDLE == 1 )
    static TickType_t xPendedTicks = ( TickType_t ) 0U;
#endif

/*-----------------------------------------------------------*/

/*
 * Private function prototypes.
 */
static void prvInitialiseTaskLists( void );
static void prvIdleTask( void * pvParameters );
static void prvAddNewTaskToReadyList( TCB_t * pxNewTCB );
static void prvAddTaskToReadyList( TCB_t * pxTCB );
static void prvDeleteTCB( TCB_t * pxTCB );

#if ( INCLUDE_vTaskDelete == 1 )
    static void prvCheckTasksWaitingTermination( void );
#endif

#if ( configUSE_TICKLESS_IDLE == 1 )
    static void prvResetNextTaskUnblockTime( void );
#endif

/*
 * Checks for stack overflow on the current task (called from vTaskSwitchContext).
 */
#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )
    static void prvCheckStackOverflow( void );
#endif

/*-----------------------------------------------------------*/

/*
 * Initialise the kernel lists before any tasks are created.
 */
static void prvInitialiseTaskLists( void )
{
    UBaseType_t uxPriority;

    for( uxPriority = ( UBaseType_t ) 0U; uxPriority < ( UBaseType_t ) configMAX_PRIORITIES; uxPriority++ )
    {
        vListInitialise( &( pxReadyTasksLists[ uxPriority ] ) );
    }

    vListInitialise( &xDelayedTaskList1 );
    vListInitialise( &xDelayedTaskList2 );
    vListInitialise( &xPendingReadyList );

    #if ( INCLUDE_vTaskDelete == 1 )
    {
        vListInitialise( &xTasksWaitingTermination );
    }
    #endif

    /* Start with xDelayedTaskList1 as the active delayed list. */
    pxDelayedTaskList = &xDelayedTaskList1;
    pxOverflowDelayedTaskList = &xDelayedTaskList2;
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelete == 1 )

    static void prvCheckTasksWaitingTermination( void )
    {
        /* This function is called from the idle task, so no need to guard
         * against the idle task deleting itself. */
        while( uxDeletedTasksWaitingCleanUp > ( UBaseType_t ) 0U )
        {
            TCB_t * pxTCB;

            taskENTER_CRITICAL();
            {
                /* Since the list is guarded by the critical section,
                 * pop the first item. */
                pxTCB = ( TCB_t * ) listGET_OWNER_OF_HEAD_ENTRY( &xTasksWaitingTermination );
                ( void ) uxListRemove( &( pxTCB->xStateListItem ) );
                uxDeletedTasksWaitingCleanUp--;
            }
            taskEXIT_CRITICAL();

            prvDeleteTCB( pxTCB );
        }
    }

#endif /* INCLUDE_vTaskDelete */
/*-----------------------------------------------------------*/

static void prvDeleteTCB( TCB_t * pxTCB )
{
    if( pxTCB->pxStack != NULL )
    {
        vPortFree( pxTCB->pxStack );
    }
    vPortFree( pxTCB );
}
/*-----------------------------------------------------------*/

/*
 * Add a NEWLY CREATED task to the appropriate ready list (or to
 * pxCurrentTCB if it should run immediately).  Called from xTaskCreate()
 * while the scheduler is suspended.
 */
static void prvAddNewTaskToReadyList( TCB_t * pxNewTCB )
{
    taskENTER_CRITICAL();
    {
        uxCurrentNumberOfTasks++;

        /* Check if a task is already running. */
        if( pxCurrentTCB == NULL )
        {
            /* No task running — make this one the current task. */
            pxCurrentTCB = pxNewTCB;

            if( uxCurrentNumberOfTasks == ( UBaseType_t ) 1 )
            {
                /* This is the first task to be created, so initialise
                 * the kernel lists. */
                prvInitialiseTaskLists();
            }
        }
        else
        {
            /* A task is already running.  If the new task has a higher
             * priority, or the scheduler isn't running yet, set
             * pxCurrentTCB to the highest-priority task. */
            if( xSchedulerRunning == pdFALSE )
            {
                /* Scheduler hasn't started — always take the
                 * highest-priority task. */
                if( pxCurrentTCB->uxPriority <= pxNewTCB->uxPriority )
                {
                    pxCurrentTCB = pxNewTCB;
                }
            }
        }

        /* Mark uxTopReadyPriority to include this priority. */
        uxTopReadyPriority |= taskREADY_PRIORITY_MASK( pxNewTCB->uxPriority );

        /* Place the task on its ready list. */
        vListInsertEnd( &( pxReadyTasksLists[ pxNewTCB->uxPriority ] ),
                        &( pxNewTCB->xStateListItem ) );
    }
    taskEXIT_CRITICAL();

    /* If the scheduler is running and the new task has a higher priority
     * than the current task, force a context switch. */
    if( xSchedulerRunning != pdFALSE )
    {
        if( pxCurrentTCB->uxPriority < pxNewTCB->uxPriority )
        {
            portYIELD();
        }
    }
}
/*-----------------------------------------------------------*/

/*
 * Add an ALREADY EXISTING task into the appropriate ready list.
 * Used when a task unblocks / resumes.
 */
static void prvAddTaskToReadyList( TCB_t * pxTCB )
{
    taskENTER_CRITICAL();
    {
        /* Place on ready list. */
        vListInsertEnd( &( pxReadyTasksLists[ pxTCB->uxPriority ] ),
                        &( pxTCB->xStateListItem ) );

        /* Update the priority bitmap. */
        uxTopReadyPriority |= taskREADY_PRIORITY_MASK( pxTCB->uxPriority );
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )

    static void prvCheckStackOverflow( void )
    {
        /* Method 2: check the stack fill pattern at the base of the
         * current task's stack.  The first word should still be the
         * fill pattern if the stack hasn't overflowed. */
        if( *( pxCurrentTCB->pxStack ) != ( ( StackType_t ) 0xA5A5A5A5UL ) )
        {
            vApplicationStackOverflowHook( ( TaskHandle_t ) pxCurrentTCB,
                                           pxCurrentTCB->pcTaskName );
        }

        /* Also check that the stack pointer hasn't gone past the end
         * of the stack area (leaving a small margin). */
        if( ( ( uint32_t ) pxCurrentTCB->pxTopOfStack ) <=
            ( ( uint32_t ) pxCurrentTCB->pxEndOfStack ) )
        {
            vApplicationStackOverflowHook( ( TaskHandle_t ) pxCurrentTCB,
                                           pxCurrentTCB->pcTaskName );
        }
    }

#endif /* configCHECK_FOR_STACK_OVERFLOW */
/*-----------------------------------------------------------*/

/*
 * The idle task — created automatically by vTaskStartScheduler().
 */
static void prvIdleTask( void * pvParameters )
{
    /* Suppress compiler warning about unused parameter. */
    ( void ) pvParameters;

    /* This task never blocks; it runs whenever nothing else is ready. */
    for( ; ; )
    {
        /* Check for tasks that have been deleted and need their memory
         * freed. */
        #if ( INCLUDE_vTaskDelete == 1 )
        {
            prvCheckTasksWaitingTermination();
        }
        #endif

        #if ( configUSE_IDLE_HOOK == 1 )
        {
            extern void vApplicationIdleHook( void );
            vApplicationIdleHook();
        }
        #endif

        /* If there is another task at the same priority as the idle
         * task (priority 0) that is ready, yield to give it time. */
        #if ( configIDLE_SHOULD_YIELD == 1 )
        {
            if( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ 0 ] ) ) >
                ( UBaseType_t ) 1 )
            {
                portYIELD();
            }
        }
        #endif
    }
}
/*-----------------------------------------------------------*/

/*
 * Public API — Task creation.
 * See task.h for parameter descriptions.
 */
BaseType_t xTaskCreate( TaskFunction_t pxTaskCode,
                        const char * const pcName,
                        const uint16_t usStackDepth,
                        void * const pvParameters,
                        UBaseType_t uxPriority,
                        TaskHandle_t * const pxCreatedTask )
{
    TCB_t * pxNewTCB;
    StackType_t * pxStack;
    uint32_t ulStackSize;
    BaseType_t xReturn;

    /* The stack depth must be at least configMINIMAL_STACK_SIZE words. */
    if( usStackDepth < configMINIMAL_STACK_SIZE )
    {
        ulStackSize = configMINIMAL_STACK_SIZE;
    }
    else
    {
        ulStackSize = usStackDepth;
    }

    /* Allocate the stack first. */
    pxStack = ( StackType_t * ) pvPortMalloc( sizeof( StackType_t ) * ulStackSize );
    if( pxStack == NULL )
    {
        return pdFAIL;           /* errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY */
    }

    /* Allocate the TCB. */
    pxNewTCB = ( TCB_t * ) pvPortMalloc( sizeof( TCB_t ) );
    if( pxNewTCB == NULL )
    {
        vPortFree( pxStack );
        return pdFAIL;
    }

    /* Fill the stack with a known pattern for stack overflow detection
     * (method 2). */
    {
        uint32_t i;
        for( i = 0U; i < ulStackSize; i++ )
        {
            pxStack[ i ] = ( StackType_t ) tskSTACK_FILL_BYTE;
        }

        /* Refill as a word pattern for easier checking. */
        for( i = 0U; i < ulStackSize; i++ )
        {
            pxStack[ i ] = ( StackType_t ) 0xA5A5A5A5UL;
        }
    }

    /* Initialise the stack pointer that will be used by the first context
     * switch.  The port layer provides this function. */
    pxNewTCB->pxTopOfStack = pxPortInitialiseStack(
        /* Top of stack (highest address).  Stack grows down on CM3. */
        &( pxStack[ ulStackSize - ( uint32_t ) 1 ] ),
        pxTaskCode,
        pvParameters );

    /* Sanity check: make sure the stack pointer is not NULL. */
    configASSERT( pxNewTCB->pxTopOfStack != NULL );

    /* Copy the task name. */
    {
        size_t x;
        for( x = ( size_t ) 0; x < ( size_t ) ( configMAX_TASK_NAME_LEN - 1 ); x++ )
        {
            pxNewTCB->pcTaskName[ x ] = pcName[ x ];
            if( pcName[ x ] == ( char ) 0x00 )
            {
                break;
            }
        }
        pxNewTCB->pcTaskName[ configMAX_TASK_NAME_LEN - 1 ] = '\0';
    }

    /* Populate the rest of the TCB. */
    pxNewTCB->pxStack        = pxStack;
    pxNewTCB->uxPriority     = uxPriority;
    pxNewTCB->uxBasePriority = uxPriority;
    pxNewTCB->ulRunTimeCounter = 0UL;

    #if ( configUSE_MUTEXES == 1 )
    {
        pxNewTCB->uxMutexesHeld = ( UBaseType_t ) 0U;
    }
    #endif

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
    {
        UBaseType_t uxNotifyIndex;
        for( uxNotifyIndex = 0U; uxNotifyIndex < ( UBaseType_t ) configTASK_NOTIFICATION_ARRAY_ENTRIES; uxNotifyIndex++ )
        {
            pxNewTCB->ulNotifiedValue[ uxNotifyIndex ] = 0UL;
            pxNewTCB->ucNotifyState[ uxNotifyIndex ] = eNotWaitingNotification;
        }
    }
    #endif

    #if ( configCHECK_FOR_STACK_OVERFLOW > 0 )
    {
        /* Set the end-of-stack limit — stack pointer should never go
         * below this address (plus a small margin). */
        pxNewTCB->pxEndOfStack = &( pxStack[ tskSTACK_OVERFLOW_MARGIN ] );
    }
    #endif

    /* Initialise the TCB list items. */
    vListInitialiseItem( &( pxNewTCB->xStateListItem ) );
    vListInitialiseItem( &( pxNewTCB->xEventListItem ) );

    /* Store the TCB pointer in the list items so the kernel can
     * navigate from a list item back to the TCB. */
    listSET_LIST_ITEM_OWNER( &( pxNewTCB->xStateListItem ), pxNewTCB );
    listSET_LIST_ITEM_OWNER( &( pxNewTCB->xEventListItem ), pxNewTCB );

    /* Event list item value stores the priority. */
    listSET_LIST_ITEM_VALUE( &( pxNewTCB->xEventListItem ),
                             ( TickType_t ) configMAX_PRIORITIES -
                             ( TickType_t ) uxPriority );

    /* Set the task number (for debug). */
    {
        taskENTER_CRITICAL();
        uxTaskNumber++;
        taskEXIT_CRITICAL();
    }

    /* Add the task to the ready list — this must be done with the
     * scheduler suspended to prevent a concurrent context switch from
     * seeing a partially-initialised TCB. */
    {
        UBaseType_t uxSavedSuspension;

        /* Prevent xTaskResumeAll from performing a context switch
         * until the TCB is fully registered. */
        uxSavedSuspension = uxSchedulerSuspended;
        uxSchedulerSuspended = ( UBaseType_t ) 1U;

        prvAddNewTaskToReadyList( pxNewTCB );

        /* Restore the previous suspension count.  If the scheduler
         * was already running and the new task has higher priority
         * than the current one, force a yield. */
        if( uxSavedSuspension == ( UBaseType_t ) 0U )
        {
            uxSchedulerSuspended = ( UBaseType_t ) 0U;

            if( xSchedulerRunning != pdFALSE )
            {
                if( pxCurrentTCB->uxPriority < pxNewTCB->uxPriority )
                {
                    /* A higher priority task is now ready — yield. */
                    portYIELD();
                }
            }
        }
        else
        {
            uxSchedulerSuspended = uxSavedSuspension;
        }
    }

    /* Return the handle if the caller requested one. */
    if( pxCreatedTask != NULL )
    {
        *pxCreatedTask = ( TaskHandle_t ) pxNewTCB;
    }

    xReturn = pdPASS;
    return xReturn;
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelete == 1 )

    void vTaskDelete( TaskHandle_t xTaskToDelete )
    {
        TCB_t * pxTCB;
        UBaseType_t uxSavedInterruptStatus;

        taskENTER_CRITICAL();
        {
            /* If NULL is passed, delete the calling task. */
            if( xTaskToDelete == NULL )
            {
                pxTCB = pxCurrentTCB;
            }
            else
            {
                pxTCB = ( TCB_t * ) xTaskToDelete;
            }

            /* Remove the task from whichever state list it is on. */
            if( uxListRemove( &( pxTCB->xStateListItem ) ) == ( UBaseType_t ) 0U )
            {
                /* The task was the only one in its list — if it was a
                 * ready list, clear the corresponding priority bit. */
                if( listLIST_IS_EMPTY(
                    &( pxReadyTasksLists[ pxTCB->uxPriority ] ) ) != pdFALSE )
                {
                    uxTopReadyPriority &= ~taskREADY_PRIORITY_MASK( pxTCB->uxPriority );
                }
            }

            /* If the task was on an event list, remove it from there too. */
            if( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) != NULL )
            {
                ( void ) uxListRemove( &( pxTCB->xEventListItem ) );
            }

            /* Update task count. */
            uxCurrentNumberOfTasks--;

            /* If the calling task just deleted itself, the idle task
             * will free the memory.  Otherwise free it now. */
            if( pxTCB == pxCurrentTCB )
            {
                /* Place on the termination list for the idle task to
                 * clean up.  We cannot free the stack we are running on. */
                vListInsertEnd( &xTasksWaitingTermination,
                                &( pxTCB->xStateListItem ) );
                uxDeletedTasksWaitingCleanUp++;

                /* Force a context switch so the idle task runs. */
                portYIELD();
            }
            else
            {
                prvDeleteTCB( pxTCB );
            }
        }
        taskEXIT_CRITICAL();

        /* If the task being deleted is the current one, the task is now
         * dead — it will never return from this function.  If it is a
         * different task, we need to check whether a yield is needed. */
        if( pxTCB != pxCurrentTCB )
        {
            /* Re-compute the highest priority.  This is a simplified
             * approach — the full vTaskSwitchContext() does a proper
             * scan. */
            if( xSchedulerRunning != pdFALSE )
            {
                /* If the current TCB has the lowest priority and the
                 * deleted task was at a higher priority, we should
                 * yield.  For simplicity, always yield within an API. */
                portYIELD_WITHIN_API();
            }
        }
    }

#endif /* INCLUDE_vTaskDelete */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelay == 1 )

    void vTaskDelay( const TickType_t xTicksToDelay )
    {
        TickType_t xTimeToWake;
        BaseType_t xAlreadyYielded = pdFALSE;

        /* A delay of zero forces a yield but doesn't actually block. */
        if( xTicksToDelay > ( TickType_t ) 0U )
        {
            configASSERT( xSchedulerRunning == pdTRUE );

            vTaskSuspendAll();
            {
                /* Remove the current task from the ready list. */
                if( uxListRemove( &( pxCurrentTCB->xStateListItem ) ) ==
                    ( UBaseType_t ) 0U )
                {
                    /* The task was the only one at its priority — clear
                     * the priority bitmap. */
                    uxTopReadyPriority &= ~taskREADY_PRIORITY_MASK(
                        pxCurrentTCB->uxPriority );
                }

                /* Calculate the wake time and insert into the delayed list. */
                xTimeToWake = xTickCount + xTicksToDelay;
                listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xStateListItem ),
                                         xTimeToWake );

                /* If the wake time overflows (i.e., is less than the
                 * current tick), put it in the overflow delayed list. */
                if( xTimeToWake < xTickCount )
                {
                    vListInsert( pxOverflowDelayedTaskList,
                                 &( pxCurrentTCB->xStateListItem ) );
                }
                else
                {
                    vListInsert( pxDelayedTaskList,
                                 &( pxCurrentTCB->xStateListItem ) );

                    /* Update the next unblock time if this task will
                     * be the earliest to wake. */
                    if( xNextTaskUnblockTime > xTimeToWake )
                    {
                        xNextTaskUnblockTime = xTimeToWake;
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }
            }
            xAlreadyYielded = xTaskResumeAll();

            /* If xTaskResumeAll did not yield, force one now (the
             * calling task is delayed and cannot run). */
            if( xAlreadyYielded == pdFALSE )
            {
                portYIELD_WITHIN_API();
            }
        }
        else
        {
            /* Zero delay — just yield. */
            portYIELD();
        }
    }

#endif /* INCLUDE_vTaskDelay */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelayUntil == 1 )

    void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime,
                          const TickType_t xTimeIncrement )
    {
        TickType_t xTimeToWake;
        BaseType_t xAlreadyYielded;
        BaseType_t xShouldDelay = pdFALSE;

        configASSERT( pxPreviousWakeTime != NULL );
        configASSERT( xSchedulerRunning == pdTRUE );

        vTaskSuspendAll();
        {
            /* Get the current tick count. */
            const TickType_t xConstTickCount = xTickCount;

            /* Calculate the target wake time — this is the previous wake
             * time plus the increment. */
            xTimeToWake = *pxPreviousWakeTime + xTimeIncrement;

            /* If the target wake time is in the past (due to a long
             * execution time on the previous iteration), set it to
             * one tick in the future. */
            if( xConstTickCount >= *pxPreviousWakeTime )
            {
                if( xTimeToWake <= xConstTickCount )
                {
                    xTimeToWake = xConstTickCount + ( TickType_t ) 1U;
                }
            }

            /* Update the previous wake time for the next call. */
            *pxPreviousWakeTime = xTimeToWake;

            /* Only delay if we're not already late. */
            if( xTimeToWake > xConstTickCount )
            {
                xShouldDelay = pdTRUE;

                /* Remove from ready list. */
                if( uxListRemove( &( pxCurrentTCB->xStateListItem ) ) ==
                    ( UBaseType_t ) 0U )
                {
                    uxTopReadyPriority &= ~taskREADY_PRIORITY_MASK(
                        pxCurrentTCB->uxPriority );
                }

                /* Put on the delayed list. */
                listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xStateListItem ),
                                         xTimeToWake );

                if( xTimeToWake < xConstTickCount )
                {
                    vListInsert( pxOverflowDelayedTaskList,
                                 &( pxCurrentTCB->xStateListItem ) );
                }
                else
                {
                    vListInsert( pxDelayedTaskList,
                                 &( pxCurrentTCB->xStateListItem ) );

                    if( xNextTaskUnblockTime > xTimeToWake )
                    {
                        xNextTaskUnblockTime = xTimeToWake;
                    }
                }
            }
        }
        xAlreadyYielded = xTaskResumeAll();

        if( xAlreadyYielded == pdFALSE )
        {
            portYIELD_WITHIN_API();
        }
    }

#endif /* INCLUDE_vTaskDelayUntil */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )

    void vTaskSuspend( TaskHandle_t xTaskToSuspend )
    {
        TCB_t * pxTCB;

        taskENTER_CRITICAL();
        {
            /* If NULL, suspend the calling task. */
            if( xTaskToSuspend == NULL )
            {
                pxTCB = pxCurrentTCB;
            }
            else
            {
                pxTCB = ( TCB_t * ) xTaskToSuspend;
            }

            /* Remove the task from whatever state list it's on. */
            if( uxListRemove( &( pxTCB->xStateListItem ) ) ==
                ( UBaseType_t ) 0U )
            {
                /* It was the only task at that priority — clear the
                 * ready priority mask. */
                uxTopReadyPriority &= ~taskREADY_PRIORITY_MASK(
                    pxTCB->uxPriority );
            }

            /* Also remove from any event list. */
            if( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) != NULL )
            {
                ( void ) uxListRemove( &( pxTCB->xEventListItem ) );
            }

            /* Place on the delayed list with a wake time of
             * portMAX_DELAY (never wakes without an explicit resume). */
            listSET_LIST_ITEM_VALUE( &( pxTCB->xStateListItem ),
                                     portMAX_DELAY );
            vListInsertEnd( &xSuspendedTaskList,
                            &( pxTCB->xStateListItem ) );
        }
        taskEXIT_CRITICAL();

        /* If the calling task suspended itself, yield. */
        if( pxTCB == pxCurrentTCB )
        {
            if( xSchedulerRunning != pdFALSE )
            {
                portYIELD_WITHIN_API();
            }
            else
            {
                /* The scheduler is not running, but this is the
                 * current task and has just been suspended.
                 * Manually select another task to run. */
                if( listCURRENT_LIST_LENGTH( &xSuspendedTaskList ) ==
                    uxCurrentNumberOfTasks )
                {
                    /* All tasks are suspended — reset pxCurrentTCB. */
                    pxCurrentTCB = NULL;
                }
                else
                {
                    vTaskSwitchContext();
                }
            }
        }
    }

#endif /* INCLUDE_vTaskSuspend */
/*-----------------------------------------------------------*/

static List_t xSuspendedTaskList;

#if ( INCLUDE_vTaskSuspend == 1 )

    void vTaskResume( TaskHandle_t xTaskToResume )
    {
        TCB_t * const pxTCB = ( TCB_t * ) xTaskToResume;
        configASSERT( xTaskToResume != NULL );

        /* The task must be suspended (on the suspended list). */
        if( listIS_CONTAINED_WITHIN( &xSuspendedTaskList,
                                     &( pxTCB->xStateListItem ) ) != pdFALSE )
        {
            taskENTER_CRITICAL();
            {
                ( void ) uxListRemove( &( pxTCB->xStateListItem ) );
                prvAddTaskToReadyList( pxTCB );

                /* If the resumed task has a higher priority than the
                 * running task, yield. */
                if( pxTCB->uxPriority > pxCurrentTCB->uxPriority )
                {
                    portYIELD();
                }
            }
            taskEXIT_CRITICAL();
        }
    }

#endif /* INCLUDE_vTaskSuspend */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )

    BaseType_t xTaskResumeFromISR( TaskHandle_t xTaskToResume )
    {
        BaseType_t xYieldRequired = pdFALSE;
        TCB_t * const pxTCB = ( TCB_t * ) xTaskToResume;

        configASSERT( xTaskToResume );

        if( listIS_CONTAINED_WITHIN( &xSuspendedTaskList,
                                     &( pxTCB->xStateListItem ) ) != pdFALSE )
        {
            UBaseType_t uxSavedInterruptStatus;

            uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
            {
                ( void ) uxListRemove( &( pxTCB->xStateListItem ) );

                /* Add to the ready list.  If the scheduler is
                 * suspended, place on the pending-ready list. */
                if( uxSchedulerSuspended == ( UBaseType_t ) 0U )
                {
                    prvAddTaskToReadyList( pxTCB );
                }
                else
                {
                    vListInsertEnd( &xPendingReadyList,
                                    &( pxTCB->xEventListItem ) );
                }

                /* Check if a yield is required. */
                if( pxTCB->uxPriority > pxCurrentTCB->uxPriority )
                {
                    xYieldRequired = pdTRUE;
                }
            }
            portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );
        }

        return xYieldRequired;
    }

#endif /* INCLUDE_vTaskSuspend */
/*-----------------------------------------------------------*/

void vTaskSuspendAll( void )
{
    /* No critical section needed — writes to UBaseType_t are atomic on
     * Cortex-M3, and the API functions that check this variable will
     * be running in a task context (not an ISR). */
    uxSchedulerSuspended++;
}
/*-----------------------------------------------------------*/

BaseType_t xTaskResumeAll( void )
{
    TCB_t * pxTCB;
    BaseType_t xAlreadyYielded = pdFALSE;

    /* This is safe on Cortex-M3 because the variable is only written
     * by tasks (not ISRs) and reads are atomic. */
    taskENTER_CRITICAL();
    {
        uxSchedulerSuspended--;

        if( uxSchedulerSuspended == ( UBaseType_t ) 0U )
        {
            /* No longer suspended — move any pending-ready tasks to
             * the actual ready lists. */
            while( listLIST_IS_EMPTY( &xPendingReadyList ) == pdFALSE )
            {
                pxTCB = ( TCB_t * ) listGET_OWNER_OF_HEAD_ENTRY(
                    &xPendingReadyList );
                ( void ) uxListRemove( &( pxTCB->xEventListItem ) );
                ( void ) uxListRemove( &( pxTCB->xStateListItem ) );
                prvAddTaskToReadyList( pxTCB );

                /* If the unblocked task has higher priority than the
                 * current task, remember to yield. */
                if( pxTCB->uxPriority > pxCurrentTCB->uxPriority )
                {
                    xYieldPending = pdTRUE;
                }
            }

            /* Process any pending yield. */
            if( xYieldPending != pdFALSE )
            {
                xAlreadyYielded = pdTRUE;
                xYieldPending = pdFALSE;
                portYIELD();
            }
        }
    }
    taskEXIT_CRITICAL();

    return xAlreadyYielded;
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskPrioritySet == 1 )

    void vTaskPrioritySet( TaskHandle_t xTask, UBaseType_t uxNewPriority )
    {
        TCB_t * pxTCB;
        UBaseType_t uxPriorityUsedOnEntry;

        if( uxNewPriority >= ( UBaseType_t ) configMAX_PRIORITIES )
        {
            uxNewPriority = ( UBaseType_t ) configMAX_PRIORITIES - ( UBaseType_t ) 1U;
        }

        taskENTER_CRITICAL();
        {
            if( xTask == NULL )
            {
                pxTCB = pxCurrentTCB;
            }
            else
            {
                pxTCB = ( TCB_t * ) xTask;
            }

            uxPriorityUsedOnEntry = pxTCB->uxPriority;

            /* Only change if the new priority is different. */
            if( uxPriorityUsedOnEntry != uxNewPriority )
            {
                /* Task can only have its priority boosted by a mutex
                 * if it already holds one or more mutexes.  Check
                 * against the base priority for mutex-discipline
                 * enforcement. */
                #if ( configUSE_MUTEXES == 1 )
                {
                    if( uxNewPriority > pxTCB->uxBasePriority )
                    {
                        if( pxTCB->uxMutexesHeld == ( UBaseType_t ) 0U )
                        {
                            /* Not holding a mutex — can only lower or
                             * stay at base priority.  Keep the base
                             * priority as the ceiling. */
                        }
                    }
                }
                #endif

                /* Update the TCB. */
                pxTCB->uxPriority = uxNewPriority;
                pxTCB->uxBasePriority = uxNewPriority;

                /* Update the event list item value (used to sort
                 * event lists by priority). */
                listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ),
                    ( TickType_t ) configMAX_PRIORITIES -
                    ( TickType_t ) uxNewPriority );

                /* If the task is currently ready, move it to the
                 * correct ready list. */
                if( listIS_CONTAINED_WITHIN(
                    &( pxReadyTasksLists[ uxPriorityUsedOnEntry ] ),
                    &( pxTCB->xStateListItem ) ) != pdFALSE )
                {
                    ( void ) uxListRemove( &( pxTCB->xStateListItem ) );

                    if( listLIST_IS_EMPTY(
                        &( pxReadyTasksLists[ uxPriorityUsedOnEntry ] ) ) != pdFALSE )
                    {
                        uxTopReadyPriority &= ~taskREADY_PRIORITY_MASK(
                            uxPriorityUsedOnEntry );
                    }

                    prvAddTaskToReadyList( pxTCB );
                }

                /* If the task is now higher priority than the
                 * running task, yield. */
                if( uxNewPriority > pxCurrentTCB->uxPriority )
                {
                    portYIELD();
                }
            }
        }
        taskEXIT_CRITICAL();
    }

#endif /* INCLUDE_vTaskPrioritySet */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskPriorityGet == 1 )

    UBaseType_t uxTaskPriorityGet( const TaskHandle_t xTask )
    {
        TCB_t const * pxTCB;
        UBaseType_t uxReturn;

        taskENTER_CRITICAL();
        {
            if( xTask == NULL )
            {
                pxTCB = pxCurrentTCB;
            }
            else
            {
                pxTCB = ( TCB_t * ) xTask;
            }

            uxReturn = pxTCB->uxPriority;
        }
        taskEXIT_CRITICAL();

        return uxReturn;
    }

#endif /* INCLUDE_uxTaskPriorityGet */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskPriorityGet == 1 )

    UBaseType_t uxTaskPriorityGetFromISR( const TaskHandle_t xTask )
    {
        TCB_t const * pxTCB;
        UBaseType_t uxReturn;
        UBaseType_t uxSavedInterruptStatus;

        uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
        {
            if( xTask == NULL )
            {
                pxTCB = pxCurrentTCB;
            }
            else
            {
                pxTCB = ( TCB_t * ) xTask;
            }

            uxReturn = pxTCB->uxPriority;
        }
        portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );

        return uxReturn;
    }

#endif /* INCLUDE_uxTaskPriorityGet */
/*-----------------------------------------------------------*/

void vTaskStartScheduler( void )
{
    BaseType_t xReturn;

    /* Initialise the kernel lists if not already done. */
    prvInitialiseTaskLists();
    vListInitialise( &xSuspendedTaskList );

    /* Create the idle task — it runs at priority 0 (lowest). */
    xReturn = xTaskCreate( prvIdleTask,
                           "IDLE",
                           configMINIMAL_STACK_SIZE,
                           ( void * ) NULL,
                           ( UBaseType_t ) 0,
                           &xIdleTaskHandle );

    #if ( configUSE_TIMERS == 1 )
    {
        if( xReturn == pdPASS )
        {
            xReturn = xTimerCreateTimerTask();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    #endif

    if( xReturn == pdPASS )
    {
        /* Interrupts should be disabled before starting the scheduler. */
        portDISABLE_INTERRUPTS();

        /* Set the next unblock time to max — will be updated once
         * tasks start delaying. */
        xNextTaskUnblockTime = portMAX_DELAY;
        xSchedulerRunning = pdTRUE;
        xTickCount = ( TickType_t ) 0U;

        /* If the application provides a function to obtain the time
         * spent in the current tick, initialise it. */
        /* (No tick hook used in this configuration.) */

        /* Hand control to the port layer — never returns. */
        if( xPortStartScheduler() != pdFALSE )
        {
            /* Should never reach here — scheduler start failed. */
        }
        else
        {
            /* Scheduler started successfully — never returns. */
        }
    }
    else
    {
        /* The idle task could not be created — the application must
         * handle this error. */
        configASSERT( xReturn != pdFAIL );
    }
}
/*-----------------------------------------------------------*/

void vTaskEndScheduler( void )
{
    /* Not fully implemented in ports that cannot return from the
     * scheduler.  Stops SysTick and disables interrupts. */
    portDISABLE_INTERRUPTS();
    xSchedulerRunning = pdFALSE;
    vPortEndScheduler();
}
/*-----------------------------------------------------------*/

TickType_t xTaskGetTickCount( void )
{
    TickType_t xReturn;
    UBaseType_t uxSavedInterruptStatus;

    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        xReturn = xTickCount;
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );

    return xReturn;
}
/*-----------------------------------------------------------*/

TickType_t xTaskGetTickCountFromISR( void )
{
    return xTickCount;
}
/*-----------------------------------------------------------*/

UBaseType_t uxTaskGetNumberOfTasks( void )
{
    return uxCurrentNumberOfTasks;
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_pcTaskGetTaskName == 1 )

    char * pcTaskGetName( TaskHandle_t xTaskToQuery )
    {
        TCB_t * pxTCB;

        if( xTaskToQuery == NULL )
        {
            pxTCB = pxCurrentTCB;
        }
        else
        {
            pxTCB = ( TCB_t * ) xTaskToQuery;
        }

        return pxTCB->pcTaskName;
    }

#endif /* INCLUDE_pcTaskGetTaskName */
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetIdleTaskHandle == 1 )

    TaskHandle_t xTaskGetIdleTaskHandle( void )
    {
        return xIdleTaskHandle;
    }

#endif /* INCLUDE_xTaskGetIdleTaskHandle */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )

    UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask )
    {
        TCB_t * pxTCB;
        StackType_t * pxEndOfStack;
        StackType_t * pxStackWalker;
        UBaseType_t uxHighWaterMark;
        uint32_t ulStackDepth;

        if( xTask == NULL )
        {
            pxTCB = pxCurrentTCB;
        }
        else
        {
            pxTCB = ( TCB_t * ) xTask;
        }

        /* Walk the stack from the bottom up, counting how many
         * entries still contain the fill pattern (0xA5A5A5A5). */
        pxEndOfStack = pxTCB->pxStack;
        ulStackDepth = 0U;

        /* Determine the stack size by finding the first non-fill
         * pattern word. */
        for( pxStackWalker = pxEndOfStack;
             *pxStackWalker == ( StackType_t ) 0xA5A5A5A5UL;
             pxStackWalker++ )
        {
            ulStackDepth++;
        }

        /* The high water mark is the number of WORDS remaining. */
        uxHighWaterMark = ( UBaseType_t ) ulStackDepth;

        return uxHighWaterMark;
    }

#endif /* INCLUDE_uxTaskGetStackHighWaterMark */
/*-----------------------------------------------------------*/

#if ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_eTaskGetState == 1 ) || \
      ( INCLUDE_xTaskGetSchedulerState == 1 ) )

    BaseType_t xTaskGetSchedulerState( void )
    {
        BaseType_t xReturn;

        if( xSchedulerRunning == pdFALSE )
        {
            xReturn = taskSCHEDULER_NOT_STARTED;
        }
        else if( uxSchedulerSuspended != ( UBaseType_t ) 0U )
        {
            xReturn = taskSCHEDULER_SUSPENDED;
        }
        else
        {
            xReturn = taskSCHEDULER_RUNNING;
        }

        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) || \
    ( INCLUDE_eTaskGetState == 1 )

    UBaseType_t uxTaskGetSystemState( TaskStatus_t * const pxTaskStatusArray,
                                      const UBaseType_t uxArraySize,
                                      uint32_t * const pulTotalRunTime )
    {
        UBaseType_t uxTaskCount = 0U;
        UBaseType_t uxPriority;
        List_t const * pxList;
        ListItem_t const * pxListItem;
        TCB_t const * pxTCB;

        vTaskSuspendAll();
        {
            for( uxPriority = ( UBaseType_t ) 0U;
                 uxPriority < ( UBaseType_t ) configMAX_PRIORITIES;
                 uxPriority++ )
            {
                pxList = &( pxReadyTasksLists[ uxPriority ] );

                pxListItem = listGET_HEAD_ENTRY( pxList );
                while( pxListItem != listGET_END_MARKER( pxList ) )
                {
                    if( uxTaskCount < uxArraySize )
                    {
                        pxTCB = ( TCB_t * ) listGET_LIST_ITEM_OWNER(
                            pxListItem );

                        pxTaskStatusArray[ uxTaskCount ].xHandle =
                            ( TaskHandle_t ) pxTCB;
                        pxTaskStatusArray[ uxTaskCount ].pcTaskName =
                            pxTCB->pcTaskName;
                        pxTaskStatusArray[ uxTaskCount ].xTaskNumber = 0;
                        pxTaskStatusArray[ uxTaskCount ].eCurrentState =
                            eReady;
                        pxTaskStatusArray[ uxTaskCount ].uxCurrentPriority =
                            pxTCB->uxPriority;
                        pxTaskStatusArray[ uxTaskCount ].uxBasePriority =
                            pxTCB->uxBasePriority;
                        pxTaskStatusArray[ uxTaskCount ].ulRunTimeCounter =
                            pxTCB->ulRunTimeCounter;
                        pxTaskStatusArray[ uxTaskCount ].pxStackBase =
                            pxTCB->pxStack;
                        #if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )
                        {
                            pxTaskStatusArray[ uxTaskCount ]
                                .usStackHighWaterMark =
                                uxTaskGetStackHighWaterMark(
                                    ( TaskHandle_t ) pxTCB );
                        }
                        #else
                        {
                            pxTaskStatusArray[ uxTaskCount ]
                                .usStackHighWaterMark = 0;
                        }
                        #endif

                        uxTaskCount++;
                    }

                    pxListItem = listGET_NEXT( pxListItem );
                }
            }
        }
        ( void ) xTaskResumeAll();

        /* Suppress compiler warning for unused parameter. */
        ( void ) pulTotalRunTime;

        return uxTaskCount;
    }

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_eTaskGetState == 1 )

    void vTaskGetInfo( TaskHandle_t xTask,
                       TaskStatus_t * pxTaskStatus,
                       BaseType_t xGetFreeStackSpace,
                       eTaskState eState )
    {
        TCB_t const * pxTCB;

        if( xTask == NULL )
        {
            pxTCB = pxCurrentTCB;
        }
        else
        {
            pxTCB = ( TCB_t * ) xTask;
        }

        pxTaskStatus->xHandle           = ( TaskHandle_t ) pxTCB;
        pxTaskStatus->pcTaskName        = pxTCB->pcTaskName;
        pxTaskStatus->xTaskNumber       = 0;
        pxTaskStatus->eCurrentState     = eState;
        pxTaskStatus->uxCurrentPriority = pxTCB->uxPriority;
        pxTaskStatus->uxBasePriority    = pxTCB->uxBasePriority;
        pxTaskStatus->ulRunTimeCounter  = pxTCB->ulRunTimeCounter;
        pxTaskStatus->pxStackBase       = pxTCB->pxStack;

        if( xGetFreeStackSpace != pdFALSE )
        {
            pxTaskStatus->usStackHighWaterMark =
                uxTaskGetStackHighWaterMark( ( TaskHandle_t ) pxTCB );
        }
        else
        {
            pxTaskStatus->usStackHighWaterMark = 0;
        }
    }

#endif /* INCLUDE_eTaskGetState */
/*-----------------------------------------------------------*/

/*
 * Called from the PendSV handler to select the next task to run.
 * This function runs with interrupts masked at kernel level.
 */
void vTaskSwitchContext( void )
{
    /* If the scheduler is suspended, do not switch — but remember that
     * a switch was requested. */
    if( uxSchedulerSuspended != ( UBaseType_t ) 0U )
    {
        xYieldPending = pdTRUE;
        return;
    }

    #if ( configCHECK_FOR_STACK_OVERFLOW > 0 )
    {
        prvCheckStackOverflow();
    }
    #endif

    /* Find the highest priority that has a ready task. */
    {
        UBaseType_t uxPriority;

        /* __clz (count-leading-zeros) is efficient on ARM; we use a
         * loop for portability with any compiler. */
        for( uxPriority = ( UBaseType_t ) configMAX_PRIORITIES;
             uxPriority > ( UBaseType_t ) 0U;
             uxPriority-- )
        {
            if( ( uxTopReadyPriority &
                  taskREADY_PRIORITY_MASK( uxPriority - ( UBaseType_t ) 1U ) ) != 0U )
            {
                break;
            }
        }

        /* At this point, uxPriority has been decremented past the
         * loop condition, but its value is either 0 (no ready task,
         * should not happen) or the priority index + 1.  We want
         * the priority index. */
        if( uxPriority > ( UBaseType_t ) 0U )
        {
            uxPriority--;
        }

        /* Get the TCB from the head of that priority's ready list. */
        {
            List_t * pxList = &( pxReadyTasksLists[ uxPriority ] );

            /* The same task may be at the head — apply time slicing. */
            #if ( configUSE_TIME_SLICING == 1 )
            {
                if( listCURRENT_LIST_LENGTH( pxList ) >
                    ( UBaseType_t ) 1U )
                {
                    /* Move the current head to the tail, then
                     * select the new head. */
                    ListItem_t * pxItem = listGET_HEAD_ENTRY( pxList );
                    ( void ) uxListRemove( pxItem );
                    vListInsertEnd( pxList, pxItem );
                }
            }
            #endif

            pxCurrentTCB = ( TCB_t * ) listGET_OWNER_OF_HEAD_ENTRY(
                pxList );
        }
    }
}
/*-----------------------------------------------------------*/

/*
 * Called from the SysTick ISR (xPortSysTickHandler) on each tick.
 * Returns pdTRUE if a context switch is required.
 */
BaseType_t xTaskIncrementTick( void )
{
    TCB_t * pxTCB;
    TickType_t xItemValue;
    BaseType_t xSwitchRequired = pdFALSE;

    /* Only increment if the scheduler is not suspended. */
    if( uxSchedulerSuspended == ( UBaseType_t ) 0U )
    {
        /* Increment the tick counter. */
        TickType_t xConstTickCount = xTickCount + ( TickType_t ) 1U;
        xTickCount = xConstTickCount;

        /* Handle overflow. */
        if( xConstTickCount == ( TickType_t ) 0U )
        {
            /* Swap the delayed-task list pointers. */
            List_t * pxTemp;

            pxTemp = pxDelayedTaskList;
            pxDelayedTaskList = pxOverflowDelayedTaskList;
            pxOverflowDelayedTaskList = pxTemp;
            xNumOfOverflows++;

            /* Reset the next unblock time — it must be recalculated
             * after the overflow. */
            xNextTaskUnblockTime = portMAX_DELAY;
        }

        /* Check if any delayed tasks have reached their wake time. */
        if( xConstTickCount >= xNextTaskUnblockTime )
        {
            for( ; ; )
            {
                if( listLIST_IS_EMPTY( pxDelayedTaskList ) != pdFALSE )
                {
                    /* No more delayed tasks — set to max so this
                     * loop is skipped until a task is delayed. */
                    xNextTaskUnblockTime = portMAX_DELAY;
                    break;
                }
                else
                {
                    pxTCB = ( TCB_t * )
                        listGET_OWNER_OF_HEAD_ENTRY( pxDelayedTaskList );
                    xItemValue = listGET_LIST_ITEM_VALUE(
                        &( pxTCB->xStateListItem ) );

                    if( xConstTickCount < xItemValue )
                    {
                        /* The item at the head hasn't expired yet.
                         * Update the next unblock time and exit. */
                        xNextTaskUnblockTime = xItemValue;
                        break;
                    }

                    /* Remove from the delayed list. */
                    ( void ) uxListRemove(
                        &( pxTCB->xStateListItem ) );

                    /* If the task was waiting for an event, remove
                     * it from the event list as well. */
                    if( listLIST_ITEM_CONTAINER(
                        &( pxTCB->xEventListItem ) ) != NULL )
                    {
                        ( void ) uxListRemove(
                            &( pxTCB->xEventListItem ) );
                    }

                    /* Add to the ready list. */
                    prvAddTaskToReadyList( pxTCB );

                    /* If the unblocked task has a higher priority
                     * than the current task, request a switch. */
                    #if ( configUSE_PREEMPTION == 1 )
                    {
                        if( pxTCB->uxPriority >
                            pxCurrentTCB->uxPriority )
                        {
                            xSwitchRequired = pdTRUE;
                        }
                    }
                    #endif
                }
            }
        }
    }
    else
    {
        /* Scheduler is suspended — accumulate ticks.  The ticks will
         * be processed when the scheduler is resumed. */
        #if ( configUSE_TICKLESS_IDLE == 1 )
        {
            xPendedTicks++;
        }
        #endif
    }

    #if ( configUSE_TICK_HOOK == 1 )
    {
        if( uxSchedulerSuspended == ( UBaseType_t ) 0U )
        {
            extern void vApplicationTickHook( void );
            vApplicationTickHook();
        }
    }
    #endif

    return xSwitchRequired;
}
/*-----------------------------------------------------------*/

#if ( configUSE_TICKLESS_IDLE == 1 )

    void vTaskStepTick( const TickType_t xTicksToJump )
    {
        /* Correct the tick count for the time the MCU spent in
         * low-power mode.  Called from vPortSuppressTicksAndSleep(). */
        TickType_t xCount;

        for( xCount = ( TickType_t ) 0U;
             xCount < xTicksToJump;
             xCount++ )
        {
            /* Call xTaskIncrementTick() without the ISR wrapper.
             * We need to manually request a context switch if needed. */
            BaseType_t xYieldRequired;
            UBaseType_t uxSavedInterruptStatus;

            uxSavedInterruptStatus =
                portSET_INTERRUPT_MASK_FROM_ISR();
            {
                xYieldRequired = xTaskIncrementTick();
            }
            portCLEAR_INTERRUPT_MASK_FROM_ISR(
                uxSavedInterruptStatus );

            if( xYieldRequired != pdFALSE )
            {
                portYIELD_FROM_ISR( xYieldRequired );
            }
        }
    }

#endif /* configUSE_TICKLESS_IDLE */
/*-----------------------------------------------------------*/

BaseType_t eTaskConfirmSleepModeStatus( void )
{
    BaseType_t xReturn;

    #if ( INCLUDE_vTaskSuspend == 1 )
    {
        /* The scheduler must be running and not suspended. */
        if( xSchedulerRunning == pdFALSE )
        {
            xReturn = eAbortSleep;
        }
        else if( uxSchedulerSuspended != ( UBaseType_t ) 0U )
        {
            xReturn = eAbortSleep;
        }
        else if( listCURRENT_LIST_LENGTH( &xPendingReadyList ) !=
                 ( UBaseType_t ) 0U )
        {
            /* Tasks are waiting to be moved to the ready list —
             * do not sleep. */
            xReturn = eAbortSleep;
        }
        else if( xYieldPending != pdFALSE )
        {
            /* A yield is pending — do not sleep. */
            xReturn = eAbortSleep;
        }
        else
        {
            /* Safe to enter sleep mode. */
            xReturn = pdPASS;
        }
    }
    #else
    {
        /* Simplified check when task suspension is not included. */
        if( xSchedulerRunning == pdFALSE )
        {
            xReturn = eAbortSleep;
        }
        else if( uxSchedulerSuspended != ( UBaseType_t ) 0U )
        {
            xReturn = eAbortSleep;
        }
        else if( listCURRENT_LIST_LENGTH( &xPendingReadyList ) !=
                 ( UBaseType_t ) 0U )
        {
            xReturn = eAbortSleep;
        }
        else
        {
            xReturn = pdPASS;
        }
    }
    #endif

    return xReturn;
}
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    BaseType_t xTaskGenericNotify( TaskHandle_t xTaskToNotify,
                                   uint32_t ulValue,
                                   uint32_t eAction,
                                   uint32_t * pulPreviousNotificationValue )
    {
        TCB_t * pxTCB;
        BaseType_t xReturn = pdPASS;
        uint8_t ucOriginalNotifyState;
        BaseType_t xTaskWoken = pdFALSE;
        BaseType_t xShouldYield = pdFALSE;
        uint32_t ulPreviousValue;

        configASSERT( xTaskToNotify != NULL );

        pxTCB = ( TCB_t * ) xTaskToNotify;

        taskENTER_CRITICAL();
        {
            /* Read previous value before modifying. */
            ulPreviousValue = pxTCB->ulNotifiedValue[ 0 ];
            ucOriginalNotifyState = pxTCB->ucNotifyState[ 0 ];

            if( pulPreviousNotificationValue != NULL )
            {
                *pulPreviousNotificationValue = ulPreviousValue;
            }

            /* If the target task is waiting for a notification,
             * unblock it now. */
            if( ucOriginalNotifyState == eWaitingNotification )
            {
                /* Task is blocked waiting — unblock it. */
                ( void ) uxListRemove(
                    &( pxTCB->xStateListItem ) );

                /* Remove from event list too. */
                if( listLIST_ITEM_CONTAINER(
                    &( pxTCB->xEventListItem ) ) != NULL )
                {
                    ( void ) uxListRemove(
                        &( pxTCB->xEventListItem ) );
                }

                prvAddTaskToReadyList( pxTCB );

                /* Set the notification state to pending with the
                 * value according to the action. */
                switch( eAction )
                {
                    case eSetBits:
                        pxTCB->ulNotifiedValue[ 0 ] |= ulValue;
                        break;

                    case eIncrement:
                        pxTCB->ulNotifiedValue[ 0 ]++;
                        break;

                    case eSetValueWithOverwrite:
                        pxTCB->ulNotifiedValue[ 0 ] = ulValue;
                        break;

                    case eSetValueWithoutOverwrite:
                        if( ucOriginalNotifyState != ePendingNotification )
                        {
                            pxTCB->ulNotifiedValue[ 0 ] = ulValue;
                        }
                        break;

                    case eNoAction:
                    default:
                        /* No action on the value — just unblock. */
                        break;
                }

                pxTCB->ucNotifyState[ 0 ] = ePendingNotification;

                xTaskWoken = pdTRUE;

                if( pxTCB->uxPriority > pxCurrentTCB->uxPriority )
                {
                    xShouldYield = pdTRUE;
                }
            }
            else
            {
                /* Task is not waiting.  Update the notification value
                 * based on the action. */
                switch( eAction )
                {
                    case eSetBits:
                        pxTCB->ulNotifiedValue[ 0 ] |= ulValue;
                        break;

                    case eIncrement:
                        pxTCB->ulNotifiedValue[ 0 ]++;
                        break;

                    case eSetValueWithOverwrite:
                        pxTCB->ulNotifiedValue[ 0 ] = ulValue;
                        break;

                    case eSetValueWithoutOverwrite:
                        if( ucOriginalNotifyState != ePendingNotification )
                        {
                            pxTCB->ulNotifiedValue[ 0 ] = ulValue;
                            pxTCB->ucNotifyState[ 0 ] = ePendingNotification;
                        }
                        break;

                    case eNoAction:
                    default:
                        pxTCB->ucNotifyState[ 0 ] = ePendingNotification;
                        break;
                }
            }
        }
        taskEXIT_CRITICAL();

        /* Yield if a higher-priority task was woken. */
        if( xShouldYield != pdFALSE )
        {
            portYIELD_WITHIN_API();
        }

        return xReturn;
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    BaseType_t xTaskNotifyWait( uint32_t ulBitsToClearOnEntry,
                                uint32_t ulBitsToClearOnExit,
                                uint32_t * pulNotificationValue,
                                TickType_t xTicksToWait )
    {
        BaseType_t xReturn;
        BaseType_t xTimedOut = pdFALSE;
        uint32_t ulNotificationValue;

        taskENTER_CRITICAL();
        {
            /* Apply clear-on-entry mask. */
            pxCurrentTCB->ulNotifiedValue[ 0 ] &= ~ulBitsToClearOnEntry;

            if( pxCurrentTCB->ucNotifyState[ 0 ] == ePendingNotification )
            {
                /* A notification is already pending — consume it. */
                ulNotificationValue =
                    pxCurrentTCB->ulNotifiedValue[ 0 ];
                pxCurrentTCB->ucNotifyState[ 0 ] =
                    eNotWaitingNotification;
                xReturn = pdPASS;
            }
            else if( xTicksToWait == ( TickType_t ) 0U )
            {
                /* Don't block — return immediately. */
                ulNotificationValue =
                    pxCurrentTCB->ulNotifiedValue[ 0 ];
                xReturn = pdFAIL;
                xTimedOut = pdTRUE;
            }
            else
            {
                /* Block and wait for a notification. */
                pxCurrentTCB->ucNotifyState[ 0 ] =
                    eWaitingNotification;

                /* Remove from ready list. */
                if( uxListRemove(
                    &( pxCurrentTCB->xStateListItem ) ) ==
                    ( UBaseType_t ) 0U )
                {
                    uxTopReadyPriority &=
                        ~taskREADY_PRIORITY_MASK(
                            pxCurrentTCB->uxPriority );
                }

                /* Put on the delayed list. */
                TickType_t xTimeToWake = xTickCount + xTicksToWait;

                if( xTimeToWake == ( TickType_t ) 0U )
                {
                    xTimeToWake = ( TickType_t ) 1U;
                }

                listSET_LIST_ITEM_VALUE(
                    &( pxCurrentTCB->xStateListItem ),
                    xTimeToWake );

                if( xTimeToWake < xTickCount )
                {
                    vListInsert( pxOverflowDelayedTaskList,
                                 &( pxCurrentTCB->xStateListItem ) );
                }
                else
                {
                    vListInsert( pxDelayedTaskList,
                                 &( pxCurrentTCB->xStateListItem ) );

                    if( xNextTaskUnblockTime > xTimeToWake )
                    {
                        xNextTaskUnblockTime = xTimeToWake;
                    }
                }

                xReturn = pdFAIL;  /* Will be updated on wake. */
            }
        }
        taskEXIT_CRITICAL();

        /* If the task blocked, yield. */
        if( xTicksToWait > ( TickType_t ) 0U )
        {
            portYIELD_WITHIN_API();
        }

        if( xTimedOut == pdFALSE )
        {
            if( pulNotificationValue != NULL )
            {
                *pulNotificationValue = ulNotificationValue;
            }

            /* Apply clear-on-exit mask. */
            pxCurrentTCB->ulNotifiedValue[ 0 ] &=
                ~ulBitsToClearOnExit;
        }
        else
        {
            if( pulNotificationValue != NULL )
            {
                *pulNotificationValue = ulNotificationValue;
            }
        }

        return xReturn;
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    uint32_t ulTaskNotifyTake( BaseType_t xClearCountOnExit,
                               TickType_t xTicksToWait )
    {
        uint32_t ulReturn = 0U;
        BaseType_t xTimedOut = pdFALSE;

        taskENTER_CRITICAL();
        {
            if( pxCurrentTCB->ucNotifyState[ 0 ] == ePendingNotification )
            {
                /* Notification pending — consume it. */
                ulReturn = pxCurrentTCB->ulNotifiedValue[ 0 ];

                if( xClearCountOnExit != pdFALSE )
                {
                    pxCurrentTCB->ulNotifiedValue[ 0 ] = 0U;
                }
                else if( ulReturn > 0U )
                {
                    pxCurrentTCB->ulNotifiedValue[ 0 ] =
                        ulReturn - 1U;
                }

                pxCurrentTCB->ucNotifyState[ 0 ] =
                    eNotWaitingNotification;
            }
            else if( xTicksToWait == ( TickType_t ) 0U )
            {
                /* Don't block — return current value. */
                ulReturn = pxCurrentTCB->ulNotifiedValue[ 0 ];
                xTimedOut = pdTRUE;
            }
            else
            {
                /* Block until a notification arrives. */
                pxCurrentTCB->ucNotifyState[ 0 ] =
                    eWaitingNotification;

                if( uxListRemove(
                    &( pxCurrentTCB->xStateListItem ) ) ==
                    ( UBaseType_t ) 0U )
                {
                    uxTopReadyPriority &=
                        ~taskREADY_PRIORITY_MASK(
                            pxCurrentTCB->uxPriority );
                }

                TickType_t xTimeToWake = xTickCount + xTicksToWait;
                if( xTimeToWake == ( TickType_t ) 0U )
                {
                    xTimeToWake = ( TickType_t ) 1U;
                }

                listSET_LIST_ITEM_VALUE(
                    &( pxCurrentTCB->xStateListItem ),
                    xTimeToWake );

                if( xTimeToWake < xTickCount )
                {
                    vListInsert( pxOverflowDelayedTaskList,
                                 &( pxCurrentTCB->xStateListItem ) );
                }
                else
                {
                    vListInsert( pxDelayedTaskList,
                                 &( pxCurrentTCB->xStateListItem ) );

                    if( xNextTaskUnblockTime > xTimeToWake )
                    {
                        xNextTaskUnblockTime = xTimeToWake;
                    }
                }
            }
        }
        taskEXIT_CRITICAL();

        if( ( xTicksToWait > ( TickType_t ) 0U ) &&
            ( xTimedOut == pdFALSE ) )
        {
            portYIELD_WITHIN_API();
        }

        /* When woken, apply the clear-on-exit policy. */
        if( xTimedOut == pdFALSE )
        {
            taskENTER_CRITICAL();
            {
                ulReturn = pxCurrentTCB->ulNotifiedValue[ 0 ];

                if( xClearCountOnExit != pdFALSE )
                {
                    pxCurrentTCB->ulNotifiedValue[ 0 ] = 0U;
                }
                else if( ulReturn > 0U )
                {
                    pxCurrentTCB->ulNotifiedValue[ 0 ] =
                        ulReturn - 1U;
                }
            }
            taskEXIT_CRITICAL();
        }

        return ulReturn;
    }

#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------*/

/*
 * Event-list helpers used by event_groups.c and queue.c.
 *
 * vTaskPlaceOnUnorderedEventList() — puts a task into the blocked state
 * on an unordered event list (the list is sorted by event-list-item value,
 * not by the state-list-item value used for ready/delayed).
 *
 * vTaskRemoveFromUnorderedEventList() — removes a task from an
 * unordered event list and places it back on the ready list.
 *
 * uxTaskResetEventItemValue() — reads and resets the calling task's
 * event-list-item value.
 */

void vTaskPlaceOnUnorderedEventList( List_t * pxEventList,
                                     const TickType_t xEventListItemValue,
                                     const TickType_t xTicksToWait )
{
    TickType_t xTimeToWake;

    configASSERT( pxEventList != NULL );

    /* Remove from the ready list. */
    if( uxListRemove( &( pxCurrentTCB->xStateListItem ) ) ==
        ( UBaseType_t ) 0U )
    {
        uxTopReadyPriority &= ~taskREADY_PRIORITY_MASK(
            pxCurrentTCB->uxPriority );
    }

    /* Store the event value in the event list item. */
    listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xEventListItem ),
                             xEventListItemValue );

    /* Insert the task's event list item into the waiting list.
     * The position in the list is determined by xItemValue —
     * highest value first. */
    vListInsert( pxEventList, &( pxCurrentTCB->xEventListItem ) );

    /* Place on the delayed list for the timeout. */
    if( xTicksToWait != portMAX_DELAY )
    {
        xTimeToWake = xTickCount + xTicksToWait;
        if( xTimeToWake == ( TickType_t ) 0U )
        {
            xTimeToWake = ( TickType_t ) 1U;
        }

        listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xStateListItem ),
                                 xTimeToWake );

        if( xTimeToWake < xTickCount )
        {
            vListInsert( pxOverflowDelayedTaskList,
                         &( pxCurrentTCB->xStateListItem ) );
        }
        else
        {
            vListInsert( pxDelayedTaskList,
                         &( pxCurrentTCB->xStateListItem ) );

            if( xNextTaskUnblockTime > xTimeToWake )
            {
                xNextTaskUnblockTime = xTimeToWake;
            }
        }
    }
    else
    {
        /* Indefinite block — set the delay to max. */
        listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xStateListItem ),
                                 portMAX_DELAY );
        vListInsertEnd( &xSuspendedTaskList,
                        &( pxCurrentTCB->xStateListItem ) );
    }
}
/*-----------------------------------------------------------*/

void vTaskRemoveFromUnorderedEventList( ListItem_t * pxEventListItem,
                                        const TickType_t xItemValue )
{
    TCB_t * pxUnblockedTCB;

    /* The owner of the event list item is always a TCB. */
    pxUnblockedTCB = ( TCB_t * ) listGET_LIST_ITEM_OWNER(
        pxEventListItem );

    /* Remove the TCB's event list item from the event list. */
    ( void ) uxListRemove( pxEventListItem );

    /* Remove the TCB's state list item from the delayed/suspended list. */
    if( listLIST_ITEM_CONTAINER(
        &( pxUnblockedTCB->xStateListItem ) ) != NULL )
    {
        ( void ) uxListRemove(
            &( pxUnblockedTCB->xStateListItem ) );
    }

    /* Place the TCB on the appropriate ready list (or pending-ready
     * list if the scheduler is suspended). */
    if( uxSchedulerSuspended == ( UBaseType_t ) 0U )
    {
        prvAddTaskToReadyList( pxUnblockedTCB );
    }
    else
    {
        vListInsertEnd( &xPendingReadyList,
                        &( pxUnblockedTCB->xEventListItem ) );
    }

    /* Update the event list item value with the reason for unblocking. */
    if( pxUnblockedTCB->uxPriority > pxCurrentTCB->uxPriority )
    {
        /* A higher-priority task has been unblocked.  Store the
         * unblock reason in the event item value so the caller
         * can retrieve it. */
        listSET_LIST_ITEM_VALUE( &( pxUnblockedTCB->xEventListItem ),
                                 xItemValue );
    }
    else
    {
        listSET_LIST_ITEM_VALUE( &( pxUnblockedTCB->xEventListItem ),
                                 xItemValue );
    }
}
/*-----------------------------------------------------------*/

TickType_t uxTaskResetEventItemValue( void )
{
    TickType_t uxReturn;

    uxReturn = listGET_LIST_ITEM_VALUE(
        &( pxCurrentTCB->xEventListItem ) );

    /* Reset the value. */
    listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xEventListItem ), 0 );

    return uxReturn;
}
/*-----------------------------------------------------------*/
