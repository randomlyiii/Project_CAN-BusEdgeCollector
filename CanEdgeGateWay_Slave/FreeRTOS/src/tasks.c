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

#include <stdlib.h>
#include <string.h>

#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "StackMacros.h"
#include "timers.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Constants. */
#define tskSTACK_FILL_BYTE    ( 0xa5U )
#define tskIDLE_STACK_SIZE    configMINIMAL_STACK_SIZE

/* Task notification states. */
#define taskNOT_WAITING_NOTIFICATION     ( ( uint8_t ) 0 )
#define taskWAITING_NOTIFICATION         ( ( uint8_t ) 1 )
#define taskNOTIFICATION_RECEIVED        ( ( uint8_t ) 2 )

/* Event list item value bit. */
#define taskEVENT_LIST_ITEM_VALUE_IN_USE    ( ( TickType_t ) 0x80000000UL )

/* Allocation tracking. */
#define tskSTATICALLY_ALLOCATED     ( ( uint8_t ) 0x01 )
#define tskDYNAMICALLY_ALLOCATED    ( ( uint8_t ) 0x00 )

/*-----------------------------------------------------------*/

/* TCB definition.  Guarded — may already be defined by task.h MPI block. */
#ifndef tskTCB_DEFINED
#define tskTCB_DEFINED
#ifndef tskTCB_DEFINED
typedef struct tskTaskControlBlock
{
    volatile StackType_t * pxTopOfStack;
    #if ( portUSING_MPU_WRAPPERS == 1 )
        xMPU_SETTINGS xMPUSettings;
    #endif
    ListItem_t          xStateListItem;
    ListItem_t          xEventListItem;
    UBaseType_t         uxPriority;
    StackType_t *       pxStack;
    char                pcTaskName[ configMAX_TASK_NAME_LEN ];
    #if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
        StackType_t *   pxEndOfStack;
    #endif
    #if ( portCRITICAL_NESTING_IN_TCB == 1 )
        UBaseType_t     uxCriticalNesting;
    #endif
    #if ( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t     uxTCBNumber;
        UBaseType_t     uxTaskNumber;
    #endif
    #if ( configUSE_MUTEXES == 1 )
        UBaseType_t     uxBasePriority;
        UBaseType_t     uxMutexesHeld;
    #endif
    configSTACK_DEPTH_TYPE     uxStackDepth;
    #if ( configUSE_APPLICATION_TASK_TAG == 1 )
        TaskHookFunction_t pxTaskTag;
    #endif
    #if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
        void *          pvThreadLocalStoragePointers[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
    #endif
    #if ( configGENERATE_RUN_TIME_STATS == 1 )
        uint32_t        ulRunTimeCounter;
    #endif
    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        volatile uint32_t ulNotifiedValue[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
        volatile uint8_t  ucNotifyState[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
    #endif
    #if ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
        uint8_t         ucStaticallyAllocated;
    #endif
    #if ( INCLUDE_xTaskAbortDelay == 1 )
        uint8_t         ucDelayAborted;
    #endif
    #if ( configUSE_POSIX_ERRNO == 1 )
        int             iTaskErrno;
    #endif
} tskTCB;
#endif /* !tskTCB_DEFINED */

typedef tskTCB TCB_t;

#endif /* tskTCB_DEFINED */

/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )
    #if ( ( tskKERNEL_VERSION_NUMBER != ( ( ( uint32_t ) 10 << 24 ) | ( ( uint32_t ) 4 << 16 ) | ( ( uint32_t ) 6 << 8 ) ) ) || ( tskKERNEL_VERSION_BUILD != 0 ) )
        #error "Expected FreeRTOS V10.4.6"
    #endif
#endif

/*-----------------------------------------------------------*/

/* Private function prototypes. */
static void prvInitialiseTaskLists( void );
static void prvIdleTask( void * pvParameters );
static void prvAddCurrentTaskToDelayedList( TickType_t xTicksToWait, const BaseType_t xCanBlockIndefinitely );
static void prvInitialiseNewTask( TaskFunction_t pxTaskCode, const char * const pcName, const uint32_t ulStackDepth, void * const pvParameters, UBaseType_t uxPriority, TaskHandle_t * const pxCreatedTask, TCB_t * pxNewTCB, const MemoryRegion_t * const xRegions );
static void prvAddNewTaskToReadyList( TCB_t * pxNewTCB );
static void prvDeleteTCB( TCB_t * pxTCB );
static void prvCheckTasksWaitingTermination( void );
static void prvResetNextTaskUnblockTime( void );
#if ( configSUPPORT_STATIC_ALLOCATION == 1 )
    static TCB_t * prvAllocateTCBAndStack( const uint16_t usStackDepth, StackType_t * const puxStackBuffer, TCB_t * const pxTaskBuffer );
#endif
#if ( configUSE_TICKLESS_IDLE != 0 )
    static TickType_t prvGetExpectedIdleTime( void );
#endif

/* Helper: get TCB from handle, defaulting to current task. */
static TCB_t * prvGetTCBFromHandle( TaskHandle_t xTask )
{
    return ( xTask == NULL ) ? ( TCB_t * ) pxCurrentTCB : ( TCB_t * ) xTask;
}

/*-----------------------------------------------------------*/

/* Global data. */
PRIVILEGED_DATA TCB_t * volatile pxCurrentTCB = NULL;

PRIVILEGED_DATA static List_t pxReadyTasksLists[ configMAX_PRIORITIES ];
PRIVILEGED_DATA static List_t xDelayedTaskList1;
PRIVILEGED_DATA static List_t xDelayedTaskList2;
PRIVILEGED_DATA static List_t * volatile pxDelayedTaskList;
PRIVILEGED_DATA static List_t * volatile pxOverflowDelayedTaskList;
PRIVILEGED_DATA static List_t xPendingReadyList;

#if ( INCLUDE_vTaskDelete == 1 )
    PRIVILEGED_DATA static List_t xTasksWaitingTermination;
    PRIVILEGED_DATA static volatile UBaseType_t uxDeletedTasksWaitingCleanUp = ( UBaseType_t ) 0U;
#endif

#if ( INCLUDE_vTaskSuspend == 1 )
    PRIVILEGED_DATA static List_t xSuspendedTaskList;
#endif

#if ( INCLUDE_xTaskGetIdleTaskHandle == 1 )
    PRIVILEGED_DATA static TaskHandle_t xIdleTaskHandle = NULL;
#endif

PRIVILEGED_DATA static volatile UBaseType_t uxCurrentNumberOfTasks = ( UBaseType_t ) 0U;
PRIVILEGED_DATA static volatile TickType_t xTickCount = ( TickType_t ) 0U;
PRIVILEGED_DATA static volatile UBaseType_t uxTopReadyPriority = tskIDLE_PRIORITY;
PRIVILEGED_DATA static volatile BaseType_t xSchedulerRunning = pdFALSE;
PRIVILEGED_DATA static volatile TickType_t xPendedTicks = ( TickType_t ) 0U;
PRIVILEGED_DATA static volatile BaseType_t xYieldPending = pdFALSE;
PRIVILEGED_DATA static volatile BaseType_t xNumOfOverflows = ( BaseType_t ) 0;
PRIVILEGED_DATA static UBaseType_t uxTaskNumber = ( UBaseType_t ) 0U;
PRIVILEGED_DATA static volatile TickType_t xNextTaskUnblockTime = ( TickType_t ) 0U;

PRIVILEGED_DATA volatile UBaseType_t uxSchedulerSuspended = ( UBaseType_t ) pdFALSE;

#if ( configGENERATE_RUN_TIME_STATS == 1 )
    PRIVILEGED_DATA static uint32_t ulTaskSwitchedInTime = 0UL;
    PRIVILEGED_DATA static volatile uint32_t ulTotalRunTime = 0UL;
#endif

/*-----------------------------------------------------------*/

/*
 * Macros.
 */
#define prvAddTaskToReadyList( pxTCB )                                                    \
    do {                                                                                   \
        vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ),                   \
                        &( ( pxTCB )->xStateListItem ) );                                  \
        uxTopReadyPriority |= ( ( UBaseType_t ) 1U << ( pxTCB )->uxPriority );             \
    } while( 0 )

#define taskRESET_READY_PRIORITY( uxPriority )                                             \
    do {                                                                                   \
        if( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ ( uxPriority ) ] ) ) == 0U )    \
            uxTopReadyPriority &= ~( ( UBaseType_t ) 1U << ( uxPriority ) );               \
    } while( 0 )

#if ( configUSE_PORT_OPTIMISED_TASK_SELECTION == 0 )
    #define taskSELECT_HIGHEST_PRIORITY_TASK()                                             \
        do {                                                                               \
            UBaseType_t uxTopPriority = uxTopReadyPriority;                                \
            while( listLIST_IS_EMPTY( &( pxReadyTasksLists[ uxTopPriority ] ) ) )          \
                uxTopPriority--;                                                           \
            listGET_OWNER_OF_NEXT_ENTRY( pxCurrentTCB,                                     \
                &( pxReadyTasksLists[ uxTopPriority ] ) );                                 \
        } while( 0 )
#else
    #define taskSELECT_HIGHEST_PRIORITY_TASK()                                             \
        do {                                                                               \
            UBaseType_t uxTopPriority;                                                     \
            portGET_HIGHEST_PRIORITY( uxTopPriority, uxTopReadyPriority );                 \
            listGET_OWNER_OF_NEXT_ENTRY( pxCurrentTCB,                                     \
                &( pxReadyTasksLists[ uxTopPriority ] ) );                                 \
        } while( 0 )
#endif

#define taskSWITCH_DELAYED_LISTS()                                                         \
    do {                                                                                   \
        List_t * pxTemp;                                                                   \
        xNumOfOverflows++;                                                                 \
        pxTemp = pxDelayedTaskList;                                                        \
        pxDelayedTaskList = pxOverflowDelayedTaskList;                                     \
        pxOverflowDelayedTaskList = pxTemp;                                                \
        prvResetNextTaskUnblockTime();                                                     \
    } while( 0 )

/*-----------------------------------------------------------*/

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )

    static TCB_t * prvAllocateTCBAndStack( const uint16_t usStackDepth,
                                           StackType_t * const puxStackBuffer,
                                           TCB_t * const pxTaskBuffer )
    {
        TCB_t * pxNewTCB;

        if( pxTaskBuffer != NULL )
        {
            pxNewTCB = ( TCB_t * ) pxTaskBuffer;
            ( void ) memset( ( void * ) pxNewTCB, 0x00, sizeof( TCB_t ) );
            pxNewTCB->ucStaticallyAllocated = tskSTATICALLY_ALLOCATED;
        }
        else
        {
            pxNewTCB = ( TCB_t * ) pvPortMalloc( sizeof( TCB_t ) );
            if( pxNewTCB != NULL )
            {
                ( void ) memset( ( void * ) pxNewTCB, 0x00, sizeof( TCB_t ) );
                pxNewTCB->ucStaticallyAllocated = tskDYNAMICALLY_ALLOCATED;
            }
        }

        if( pxNewTCB != NULL )
        {
            if( puxStackBuffer != NULL )
                pxNewTCB->pxStack = ( StackType_t * ) puxStackBuffer;
            else
                pxNewTCB->pxStack = ( StackType_t * ) pvPortMalloc( ( uint32_t ) usStackDepth * sizeof( StackType_t ) );
        }

        return pxNewTCB;
    }

#endif
/*-----------------------------------------------------------*/

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

    BaseType_t xTaskCreate( TaskFunction_t pxTaskCode, const char * const pcName,
                            const configSTACK_DEPTH_TYPE usStackDepth, void * const pvParameters,
                            UBaseType_t uxPriority, TaskHandle_t * const pxCreatedTask )
    {
        TCB_t * pxNewTCB;
        BaseType_t xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;

        #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
            pxNewTCB = prvAllocateTCBAndStack( usStackDepth, NULL, NULL );
        #else
            pxNewTCB = ( TCB_t * ) pvPortMalloc( sizeof( TCB_t ) );
            if( pxNewTCB != NULL )
            {
                ( void ) memset( ( void * ) pxNewTCB, 0x00, sizeof( TCB_t ) );
                pxNewTCB->pxStack = ( StackType_t * ) pvPortMalloc( ( uint32_t ) usStackDepth * sizeof( StackType_t ) );
            }
        #endif

        if( ( pxNewTCB != NULL ) && ( pxNewTCB->pxStack != NULL ) )
        {
            #if ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
                pxNewTCB->ucStaticallyAllocated = tskDYNAMICALLY_ALLOCATED;
            #endif
            prvInitialiseNewTask( pxTaskCode, pcName, ( uint32_t ) usStackDepth, pvParameters,
                                  uxPriority, pxCreatedTask, pxNewTCB, NULL );
            prvAddNewTaskToReadyList( pxNewTCB );
            xReturn = pdPASS;
        }
        else if( pxNewTCB != NULL )
        {
            vPortFree( pxNewTCB );
        }

        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )

    TaskHandle_t xTaskCreateStatic( TaskFunction_t pxTaskCode, const char * const pcName,
                                    const uint32_t ulStackDepth, void * const pvParameters,
                                    UBaseType_t uxPriority, StackType_t * const puxStackBuffer,
                                    StaticTask_t * const pxTaskBuffer )
    {
        TCB_t * pxNewTCB;
        TaskHandle_t xReturn = NULL;

        configASSERT( puxStackBuffer != NULL );
        configASSERT( pxTaskBuffer != NULL );

        #if ( configASSERT_DEFINED == 1 )
        {
            volatile size_t xSize = sizeof( StaticTask_t );
            configASSERT( xSize == sizeof( TCB_t ) );
            ( void ) xSize;
        }
        #endif

        if( ( pxTaskBuffer != NULL ) && ( puxStackBuffer != NULL ) )
        {
            pxNewTCB = ( TCB_t * ) pxTaskBuffer;
            ( void ) memset( ( void * ) pxNewTCB, 0x00, sizeof( TCB_t ) );
            pxNewTCB->pxStack = ( StackType_t * ) puxStackBuffer;
            #if ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
                pxNewTCB->ucStaticallyAllocated = tskSTATICALLY_ALLOCATED;
            #endif
            prvInitialiseNewTask( pxTaskCode, pcName, ulStackDepth, pvParameters,
                                  uxPriority, &xReturn, pxNewTCB, NULL );
            prvAddNewTaskToReadyList( pxNewTCB );
        }

        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/

static void prvInitialiseNewTask( TaskFunction_t pxTaskCode, const char * const pcName,
                                  const uint32_t ulStackDepth, void * const pvParameters,
                                  UBaseType_t uxPriority, TaskHandle_t * const pxCreatedTask,
                                  TCB_t * pxNewTCB, const MemoryRegion_t * const xRegions )
{
    UBaseType_t x;
    ( void ) xRegions;

    #if ( configMAX_TASK_NAME_LEN > 0 )
    {
        for( x = ( UBaseType_t ) 0; x < ( UBaseType_t ) configMAX_TASK_NAME_LEN; x++ )
        {
            pxNewTCB->pcTaskName[ x ] = pcName[ x ];
            if( pcName[ x ] == ( char ) 0x00 ) break;
        }
        pxNewTCB->pcTaskName[ configMAX_TASK_NAME_LEN - 1 ] = '\0';
    }
    #endif

    if( uxPriority >= ( UBaseType_t ) configMAX_PRIORITIES )
        uxPriority = ( UBaseType_t ) configMAX_PRIORITIES - ( UBaseType_t ) 1U;

    pxNewTCB->uxPriority = uxPriority;
    #if ( configUSE_MUTEXES == 1 )
        pxNewTCB->uxBasePriority = uxPriority;
        pxNewTCB->uxMutexesHeld = 0;
    #endif

    vListInitialiseItem( &( pxNewTCB->xStateListItem ) );
    vListInitialiseItem( &( pxNewTCB->xEventListItem ) );

    listSET_LIST_ITEM_OWNER( &( pxNewTCB->xStateListItem ), pxNewTCB );
    listSET_LIST_ITEM_VALUE( &( pxNewTCB->xEventListItem ),
        ( TickType_t ) configMAX_PRIORITIES - ( TickType_t ) uxPriority );
    listSET_LIST_ITEM_OWNER( &( pxNewTCB->xEventListItem ), pxNewTCB );

    #if ( portCRITICAL_NESTING_IN_TCB == 1 )
        pxNewTCB->uxCriticalNesting = ( UBaseType_t ) 0U;
    #endif

    #if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 )
        ( void ) memset( ( void * ) &( pxNewTCB->pvThreadLocalStoragePointers[ 0 ] ), 0x00, sizeof( pxNewTCB->pvThreadLocalStoragePointers ) );
    #endif

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        ( void ) memset( ( void * ) &( pxNewTCB->ulNotifiedValue[ 0 ] ), 0x00, sizeof( pxNewTCB->ulNotifiedValue ) );
        ( void ) memset( ( void * ) &( pxNewTCB->ucNotifyState[ 0 ] ), 0x00, sizeof( pxNewTCB->ucNotifyState ) );
    #endif

    #if ( INCLUDE_xTaskAbortDelay == 1 )
        pxNewTCB->ucDelayAborted = pdFALSE;
    #endif

    #if ( portUSING_MPU_WRAPPERS == 1 )
        pxNewTCB->pxTopOfStack = pxPortInitialiseStack( pxNewTCB->pxStack, pxTaskCode, pvParameters, prvTaskExitError );
    #else
        pxNewTCB->pxTopOfStack = pxPortInitialiseStack( pxNewTCB->pxStack + ( ulStackDepth - ( uint32_t ) 1 ), pxTaskCode, pvParameters );
    #endif

    if( pxCreatedTask != NULL )
        *pxCreatedTask = ( TaskHandle_t ) pxNewTCB;

    traceTASK_CREATE( pxNewTCB );
}
/*-----------------------------------------------------------*/

static void prvAddNewTaskToReadyList( TCB_t * pxNewTCB )
{
    taskENTER_CRITICAL();
    {
        uxCurrentNumberOfTasks++;
        if( pxCurrentTCB == NULL )
        {
            pxCurrentTCB = pxNewTCB;
            if( uxCurrentNumberOfTasks == ( UBaseType_t ) 1 )
                prvInitialiseTaskLists();
        }
        else if( xSchedulerRunning == pdFALSE )
        {
            if( pxCurrentTCB->uxPriority <= pxNewTCB->uxPriority )
                pxCurrentTCB = pxNewTCB;
        }
        uxTaskNumber++;
        #if ( configUSE_TRACE_FACILITY == 1 )
            pxNewTCB->uxTCBNumber = uxTaskNumber;
        #endif
        prvAddTaskToReadyList( pxNewTCB );
        portSETUP_TCB( pxNewTCB );
    }
    taskEXIT_CRITICAL();

    if( xSchedulerRunning != pdFALSE )
    {
        if( pxCurrentTCB->uxPriority < pxNewTCB->uxPriority )
            taskYIELD_IF_USING_PREEMPTION();
    }
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelete == 1 )
    void vTaskDelete( TaskHandle_t xTaskToDelete )
    {
        TCB_t * pxTCB;
        taskENTER_CRITICAL();
        {
            pxTCB = prvGetTCBFromHandle( xTaskToDelete );
            if( uxListRemove( &( pxTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
                taskRESET_READY_PRIORITY( pxTCB->uxPriority );
            if( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) != NULL )
                ( void ) uxListRemove( &( pxTCB->xEventListItem ) );
            uxTaskNumber++;
            if( pxTCB == pxCurrentTCB )
            {
                vListInsertEnd( &xTasksWaitingTermination, &( pxTCB->xStateListItem ) );
                ++uxDeletedTasksWaitingCleanUp;
                traceTASK_DELETE( pxTCB );
                portPRE_TASK_DELETE_HOOK( pxTCB, &xYieldPending );
            }
            else
            {
                --uxCurrentNumberOfTasks;
                traceTASK_DELETE( pxTCB );
                prvDeleteTCB( pxTCB );
                prvResetNextTaskUnblockTime();
            }
        }
        taskEXIT_CRITICAL();
        if( xSchedulerRunning != pdFALSE && pxTCB == pxCurrentTCB )
        {
            configASSERT( uxSchedulerSuspended == 0 );
            portYIELD_WITHIN_API();
        }
    }
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelay == 1 )
    void vTaskDelay( const TickType_t xTicksToDelay )
    {
        BaseType_t xAlreadyYielded = pdFALSE;
        if( xTicksToDelay > ( TickType_t ) 0U )
        {
            configASSERT( uxSchedulerSuspended == 0 );
            vTaskSuspendAll();
            {
                traceTASK_DELAY();
                prvAddCurrentTaskToDelayedList( xTicksToDelay, pdFALSE );
            }
            xAlreadyYielded = xTaskResumeAll();
        }
        if( xAlreadyYielded == pdFALSE )
            portYIELD_WITHIN_API();
    }
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelayUntil == 1 )
    void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement )
    {
        TickType_t xTimeToWake;
        BaseType_t xAlreadyYielded, xShouldDelay = pdFALSE;

        configASSERT( pxPreviousWakeTime );
        configASSERT( ( xTimeIncrement > 0U ) );
        configASSERT( uxSchedulerSuspended == 0 );

        vTaskSuspendAll();
        {
            const TickType_t xConstTickCount = xTickCount;
            xTimeToWake = *pxPreviousWakeTime + xTimeIncrement;
            if( xConstTickCount < *pxPreviousWakeTime )
            {
                if( ( xTimeToWake < *pxPreviousWakeTime ) && ( xTimeToWake > xConstTickCount ) )
                    xShouldDelay = pdTRUE;
            }
            else
            {
                if( ( xTimeToWake < *pxPreviousWakeTime ) || ( xTimeToWake > xConstTickCount ) )
                    xShouldDelay = pdTRUE;
            }
            *pxPreviousWakeTime = xTimeToWake;
            if( xShouldDelay != pdFALSE )
            {
                traceTASK_DELAY_UNTIL( xTimeToWake );
                prvAddCurrentTaskToDelayedList( xTimeToWake - xConstTickCount, pdFALSE );
            }
        }
        xAlreadyYielded = xTaskResumeAll();
        if( xAlreadyYielded == pdFALSE )
            portYIELD_WITHIN_API();
    }
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )
    void vTaskSuspend( TaskHandle_t xTaskToSuspend )
    {
        TCB_t * pxTCB;
        taskENTER_CRITICAL();
        {
            pxTCB = prvGetTCBFromHandle( xTaskToSuspend );
            traceTASK_SUSPEND( pxTCB );
            if( uxListRemove( &( pxTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
                taskRESET_READY_PRIORITY( pxTCB->uxPriority );
            if( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) != NULL )
                ( void ) uxListRemove( &( pxTCB->xEventListItem ) );
            vListInsertEnd( &xSuspendedTaskList, &( pxTCB->xStateListItem ) );
            #if ( configUSE_TASK_NOTIFICATIONS == 1 )
                { BaseType_t i; for( i = 0; i < configTASK_NOTIFICATION_ARRAY_ENTRIES; i++ )
                    if( pxTCB->ucNotifyState[ i ] == taskWAITING_NOTIFICATION )
                        pxTCB->ucNotifyState[ i ] = taskNOT_WAITING_NOTIFICATION; }
            #endif
        }
        taskEXIT_CRITICAL();
        if( xSchedulerRunning != pdFALSE )
        {
            taskENTER_CRITICAL(); prvResetNextTaskUnblockTime(); taskEXIT_CRITICAL();
        }
        if( pxTCB == pxCurrentTCB )
        {
            if( xSchedulerRunning != pdFALSE )
            {
                configASSERT( uxSchedulerSuspended == 0 );
                portYIELD_WITHIN_API();
            }
            else
            {
                if( listCURRENT_LIST_LENGTH( &xSuspendedTaskList ) == uxCurrentNumberOfTasks )
                    pxCurrentTCB = NULL;
                else
                    vTaskSwitchContext();
            }
        }
    }

    void vTaskResume( TaskHandle_t xTaskToResume )
    {
        TCB_t * const pxTCB = xTaskToResume;
        configASSERT( xTaskToResume );
        if( ( pxTCB != pxCurrentTCB ) && ( pxTCB != NULL ) )
        {
            taskENTER_CRITICAL();
            {
                if( listIS_CONTAINED_WITHIN( &xSuspendedTaskList, &( pxTCB->xStateListItem ) ) != pdFALSE )
                {
                    traceTASK_RESUME( pxTCB );
                    ( void ) uxListRemove( &( pxTCB->xStateListItem ) );
                    prvAddTaskToReadyList( pxTCB );
                    if( pxTCB->uxPriority >= pxCurrentTCB->uxPriority )
                        taskYIELD_IF_USING_PREEMPTION();
                }
            }
            taskEXIT_CRITICAL();
        }
    }
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskPrioritySet == 1 )
    void vTaskPrioritySet( TaskHandle_t xTask, UBaseType_t uxNewPriority )
    {
        TCB_t * pxTCB;
        UBaseType_t uxCurrentPriority, uxPriorityUsedOnEntry;
        BaseType_t xYieldRequired = pdFALSE;

        configASSERT( uxNewPriority < configMAX_PRIORITIES );
        if( uxNewPriority >= ( UBaseType_t ) configMAX_PRIORITIES )
            uxNewPriority = ( UBaseType_t ) configMAX_PRIORITIES - ( UBaseType_t ) 1U;

        taskENTER_CRITICAL();
        {
            pxTCB = prvGetTCBFromHandle( xTask );
            traceTASK_PRIORITY_SET( pxTCB, uxNewPriority );
            #if ( configUSE_MUTEXES == 1 )
                uxCurrentPriority = pxTCB->uxBasePriority;
            #else
                uxCurrentPriority = pxTCB->uxPriority;
            #endif
            if( uxCurrentPriority != uxNewPriority )
            {
                uxPriorityUsedOnEntry = pxTCB->uxPriority;
                #if ( configUSE_MUTEXES == 1 )
                    if( pxTCB->uxBasePriority == pxTCB->uxPriority )
                        pxTCB->uxPriority = uxNewPriority;
                    pxTCB->uxBasePriority = uxNewPriority;
                #else
                    pxTCB->uxPriority = uxNewPriority;
                #endif
                listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ),
                    ( TickType_t ) configMAX_PRIORITIES - ( TickType_t ) uxNewPriority );
                if( listIS_CONTAINED_WITHIN( &( pxReadyTasksLists[ uxPriorityUsedOnEntry ] ), &( pxTCB->xStateListItem ) ) != pdFALSE )
                {
                    if( uxListRemove( &( pxTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
                        taskRESET_READY_PRIORITY( uxPriorityUsedOnEntry );
                    prvAddTaskToReadyList( pxTCB );
                    #if ( configUSE_PREEMPTION == 1 )
                        if( uxNewPriority > uxCurrentPriority ) xYieldRequired = pdTRUE;
                    #endif
                }
                else
                    taskRESET_READY_PRIORITY( uxPriorityUsedOnEntry );
            }
        }
        taskEXIT_CRITICAL();
        if( xYieldRequired == pdTRUE )
            taskYIELD_IF_USING_PREEMPTION();
    }
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskPriorityGet == 1 )
    UBaseType_t uxTaskPriorityGet( const TaskHandle_t xTask )
    {
        TCB_t const * pxTCB;
        UBaseType_t uxReturn;
        taskENTER_CRITICAL();
        { pxTCB = prvGetTCBFromHandle( xTask ); uxReturn = pxTCB->uxPriority; }
        taskEXIT_CRITICAL();
        return uxReturn;
    }
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_eTaskGetState == 1 )
    eTaskState eTaskGetState( TaskHandle_t xTask )
    {
        eTaskState eReturn;
        List_t const * pxStateList;
        const TCB_t * const pxTCB = xTask;
        configASSERT( pxTCB );
        if( pxTCB == pxCurrentTCB ) return eRunning;
        taskENTER_CRITICAL();
        { pxStateList = listLIST_ITEM_CONTAINER( &( pxTCB->xStateListItem ) ); }
        taskEXIT_CRITICAL();
        if( pxStateList == &pxReadyTasksLists[ pxTCB->uxPriority ] ) eReturn = eReady;
        else if( pxStateList == &xSuspendedTaskList ) eReturn = eSuspended;
        else if( ( pxStateList == pxDelayedTaskList ) || ( pxStateList == pxOverflowDelayedTaskList ) ) eReturn = eBlocked;
        else if( pxStateList == &xPendingReadyList ) eReturn = eReady;
        else eReturn = eBlocked;
        return eReturn;
    }
#endif
/*-----------------------------------------------------------*/

UBaseType_t uxTaskGetNumberOfTasks( void )
{
    return uxCurrentNumberOfTasks;
}
/*-----------------------------------------------------------*/

TaskHandle_t xTaskGetCurrentTaskHandle( void )
{
    TaskHandle_t xReturn;
    taskENTER_CRITICAL();
    {
        xReturn = ( TaskHandle_t ) pxCurrentTCB;
    }
    taskEXIT_CRITICAL();
    return xReturn;
}
/*-----------------------------------------------------------*/

char * pcTaskGetName( TaskHandle_t xTaskToQuery )
{
    TCB_t * pxTCB = prvGetTCBFromHandle( xTaskToQuery );
    configASSERT( pxTCB );
    return &( pxTCB->pcTaskName[ 0 ] );
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetIdleTaskHandle == 1 )
    TaskHandle_t xTaskGetIdleTaskHandle( void )
    {
        configASSERT( xIdleTaskHandle != NULL );
        return xIdleTaskHandle;
    }
#endif
/*-----------------------------------------------------------*/

TickType_t xTaskGetTickCount( void )
{
    TickType_t xTicks;
    portTICK_TYPE_ENTER_CRITICAL();
    { xTicks = xTickCount; }
    portTICK_TYPE_EXIT_CRITICAL();
    return xTicks;
}
/*-----------------------------------------------------------*/

TickType_t xTaskGetTickCountFromISR( void )
{
    TickType_t xReturn;
    UBaseType_t uxSaved;
    portASSERT_IF_INTERRUPT_PRIORITY_INVALID();
    uxSaved = portTICK_TYPE_SET_INTERRUPT_MASK_FROM_ISR();
    { xReturn = xTickCount; }
    portTICK_TYPE_CLEAR_INTERRUPT_MASK_FROM_ISR( uxSaved );
    return xReturn;
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetSchedulerState == 1 )
    BaseType_t xTaskGetSchedulerState( void )
    {
        if( xSchedulerRunning == pdFALSE ) return taskSCHEDULER_NOT_STARTED;
        if( uxSchedulerSuspended == ( UBaseType_t ) pdFALSE ) return taskSCHEDULER_RUNNING;
        return taskSCHEDULER_SUSPENDED;
    }
#endif
/*-----------------------------------------------------------*/

void vTaskSuspendAll( void )
{
    portSOFTWARE_BARRIER();
    ++uxSchedulerSuspended;
    portMEMORY_BARRIER();
}
/*-----------------------------------------------------------*/

BaseType_t xTaskResumeAll( void )
{
    TCB_t * pxTCB = NULL;
    BaseType_t xAlreadyYielded = pdFALSE;
    configASSERT( uxSchedulerSuspended );
    taskENTER_CRITICAL();
    {
        --uxSchedulerSuspended;
        if( uxSchedulerSuspended == ( UBaseType_t ) pdFALSE )
        {
            if( uxCurrentNumberOfTasks > ( UBaseType_t ) 0U )
            {
                while( listLIST_IS_EMPTY( &xPendingReadyList ) == pdFALSE )
                {
                    pxTCB = listGET_OWNER_OF_HEAD_ENTRY( &xPendingReadyList );
                    ( void ) uxListRemove( &( pxTCB->xEventListItem ) );
                    ( void ) uxListRemove( &( pxTCB->xStateListItem ) );
                    prvAddTaskToReadyList( pxTCB );
                    if( pxTCB->uxPriority >= pxCurrentTCB->uxPriority )
                        xYieldPending = pdTRUE;
                }
                if( pxTCB != NULL ) prvResetNextTaskUnblockTime();
                {
                    TickType_t xPendedCounts = xPendedTicks;
                    if( xPendedCounts > ( TickType_t ) 0U )
                    {
                        do {
                            if( xTaskIncrementTick() != pdFALSE ) xYieldPending = pdTRUE;
                            --xPendedCounts;
                        } while( xPendedCounts > ( TickType_t ) 0U );
                        xPendedTicks = 0;
                    }
                }
                if( xYieldPending != pdFALSE )
                {
                    #if ( configUSE_PREEMPTION != 0 )
                        xAlreadyYielded = pdTRUE;
                    #endif
                    taskYIELD_IF_USING_PREEMPTION();
                }
            }
        }
    }
    taskEXIT_CRITICAL();
    return xAlreadyYielded;
}
/*-----------------------------------------------------------*/

void vTaskStartScheduler( void )
{
    BaseType_t xReturn;
    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
    {
        StaticTask_t * pxIdleTaskTCBBuffer = NULL;
        StackType_t * pxIdleTaskStackBuffer = NULL;
        uint32_t ulIdleTaskStackSize;
        vApplicationGetIdleTaskMemory( &pxIdleTaskTCBBuffer, &pxIdleTaskStackBuffer, &ulIdleTaskStackSize );
        xIdleTaskHandle = xTaskCreateStatic( prvIdleTask, configIDLE_TASK_NAME, ulIdleTaskStackSize,
            ( void * ) NULL, portPRIVILEGE_BIT, pxIdleTaskStackBuffer, pxIdleTaskTCBBuffer );
        xReturn = ( xIdleTaskHandle != NULL ) ? pdPASS : errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
    }
    #else
        xReturn = xTaskCreate( prvIdleTask, configIDLE_TASK_NAME, configMINIMAL_STACK_SIZE,
            ( void * ) NULL, portPRIVILEGE_BIT, &xIdleTaskHandle );
    #endif

    #if ( configUSE_TIMERS == 1 )
        if( xReturn == pdPASS ) xReturn = xTimerCreateTimerTask();
    #endif

    if( xReturn == pdPASS )
    {
        portDISABLE_INTERRUPTS();
        #if ( configGENERATE_RUN_TIME_STATS == 1 )
            ulTaskSwitchedInTime = portGET_RUN_TIME_COUNTER_VALUE();
        #endif
        xSchedulerRunning = pdTRUE;
        xTickCount = ( TickType_t ) 0U;
        portCONFIGURE_TIMER_FOR_RUN_TIME_STATS();
        traceTASK_START_SCHEDULER();
        if( xPortStartScheduler() != pdFALSE ) { /* never returns */ }
    }
    else
        configASSERT( xReturn != errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY );
    ( void ) xIdleTaskHandle;
    ( void ) xReturn;
}
/*-----------------------------------------------------------*/

void vTaskEndScheduler( void )
{
    portDISABLE_INTERRUPTS();
    xSchedulerRunning = pdFALSE;
    vPortEndScheduler();
}
/*-----------------------------------------------------------*/

void vTaskSwitchContext( void )
{
    if( uxSchedulerSuspended != ( UBaseType_t ) pdFALSE )
    {
        xYieldPending = pdTRUE;
    }
    else
    {
        xYieldPending = pdFALSE;
        traceTASK_SWITCHED_OUT();
        #if ( configGENERATE_RUN_TIME_STATS == 1 )
        {
            uint32_t ulTimeNow = portGET_RUN_TIME_COUNTER_VALUE();
            uint32_t ulTimeDelta = ulTimeNow - ulTaskSwitchedInTime;
            if( ulTimeDelta > 0UL )
            {
                pxCurrentTCB->ulRunTimeCounter += ulTimeDelta;
                ulTotalRunTime += ulTimeDelta;
            }
            ulTaskSwitchedInTime = ulTimeNow;
        }
        #endif
        taskCHECK_FOR_STACK_OVERFLOW();
        #if ( configUSE_POSIX_ERRNO == 1 )
            pxCurrentTCB->iTaskErrno = FreeRTOS_errno;
        #endif
        taskSELECT_HIGHEST_PRIORITY_TASK();
        traceTASK_SWITCHED_IN();
        #if ( configUSE_POSIX_ERRNO == 1 )
            FreeRTOS_errno = pxCurrentTCB->iTaskErrno;
        #endif
        #if ( configUSE_TICKLESS_IDLE != 0 )
            prvGetExpectedIdleTime();
        #endif
    }
}
/*-----------------------------------------------------------*/

void vTaskPlaceOnEventList( List_t * const pxEventList, const TickType_t xTicksToWait )
{
    configASSERT( pxEventList );
    vListInsert( pxEventList, &( pxCurrentTCB->xEventListItem ) );
    prvAddCurrentTaskToDelayedList( xTicksToWait, pdTRUE );
}
/*-----------------------------------------------------------*/

void vTaskPlaceOnUnorderedEventList( List_t * pxEventList, const TickType_t xItemValue, const TickType_t xTicksToWait )
{
    configASSERT( pxEventList );
    listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xEventListItem ), xItemValue | taskEVENT_LIST_ITEM_VALUE_IN_USE );
    vListInsertEnd( pxEventList, &( pxCurrentTCB->xEventListItem ) );
    prvAddCurrentTaskToDelayedList( xTicksToWait, pdTRUE );
}
/*-----------------------------------------------------------*/

BaseType_t xTaskRemoveFromEventList( const List_t * const pxEventList )
{
    TCB_t * pxUnblockedTCB;
    BaseType_t xReturn;
    pxUnblockedTCB = listGET_OWNER_OF_HEAD_ENTRY( pxEventList );
    configASSERT( pxUnblockedTCB );
    ( void ) uxListRemove( &( pxUnblockedTCB->xEventListItem ) );
    if( uxSchedulerSuspended == ( UBaseType_t ) pdFALSE )
    {
        ( void ) uxListRemove( &( pxUnblockedTCB->xStateListItem ) );
        prvAddTaskToReadyList( pxUnblockedTCB );
    }
    else
        vListInsertEnd( &xPendingReadyList, &( pxUnblockedTCB->xEventListItem ) );
    if( pxUnblockedTCB->uxPriority > pxCurrentTCB->uxPriority )
    { xReturn = pdTRUE; xYieldPending = pdTRUE; }
    else
        xReturn = pdFALSE;
    return xReturn;
}
/*-----------------------------------------------------------*/

void vTaskRemoveFromUnorderedEventList( ListItem_t * pxEventListItem, const TickType_t xItemValue )
{
    TCB_t * pxUnblockedTCB;
    listSET_LIST_ITEM_VALUE( pxEventListItem, xItemValue | taskEVENT_LIST_ITEM_VALUE_IN_USE );
    pxUnblockedTCB = listGET_LIST_ITEM_OWNER( pxEventListItem );
    configASSERT( pxUnblockedTCB );
    ( void ) uxListRemove( pxEventListItem );
    ( void ) uxListRemove( &( pxUnblockedTCB->xStateListItem ) );
    prvAddTaskToReadyList( pxUnblockedTCB );
    if( pxUnblockedTCB->uxPriority > pxCurrentTCB->uxPriority )
        xYieldPending = pdTRUE;
}
/*-----------------------------------------------------------*/

void vTaskSetTimeOutState( TimeOut_t * const pxTimeOut )
{
    configASSERT( pxTimeOut );
    taskENTER_CRITICAL();
    { pxTimeOut->xOverflowCount = xNumOfOverflows; pxTimeOut->xTimeOnEntering = xTickCount; }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

void vTaskInternalSetTimeOutState( TimeOut_t * const pxTimeOut )
{
    pxTimeOut->xOverflowCount = xNumOfOverflows;
    pxTimeOut->xTimeOnEntering = xTickCount;
}
/*-----------------------------------------------------------*/

BaseType_t xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut, TickType_t * const pxTicksToWait )
{
    BaseType_t xReturn;
    configASSERT( pxTimeOut ); configASSERT( pxTicksToWait );
    taskENTER_CRITICAL();
    {
        const TickType_t xConstTickCount = xTickCount;
        const TickType_t xElapsedTime = xConstTickCount - pxTimeOut->xTimeOnEntering;
        #if ( INCLUDE_xTaskAbortDelay == 1 )
            if( pxCurrentTCB->ucDelayAborted != ( uint8_t ) pdFALSE )
                { pxCurrentTCB->ucDelayAborted = pdFALSE; xReturn = pdTRUE; return xReturn; }
        #endif
        #if ( INCLUDE_vTaskSuspend == 1 )
            if( *pxTicksToWait == portMAX_DELAY ) return pdFALSE;
        #endif
        if( ( xNumOfOverflows != pxTimeOut->xOverflowCount ) && ( xConstTickCount >= pxTimeOut->xTimeOnEntering ) )
            xReturn = pdTRUE;
        else if( xElapsedTime < *pxTicksToWait )
        { *pxTicksToWait -= xElapsedTime; vTaskInternalSetTimeOutState( pxTimeOut ); xReturn = pdFALSE; }
        else
        { *pxTicksToWait = 0; xReturn = pdTRUE; }
    }
    taskEXIT_CRITICAL();
    return xReturn;
}
/*-----------------------------------------------------------*/

void vTaskMissedYield( void ) { xYieldPending = pdTRUE; }
/*-----------------------------------------------------------*/

TickType_t uxTaskResetEventItemValue( void )
{
    TickType_t uxReturn;
    uxReturn = listGET_LIST_ITEM_VALUE( &( pxCurrentTCB->xEventListItem ) );
    listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xEventListItem ),
        ( ( TickType_t ) configMAX_PRIORITIES - ( TickType_t ) pxCurrentTCB->uxPriority ) );
    return uxReturn;
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )
    UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask )
    {
        TCB_t * pxTCB = prvGetTCBFromHandle( xTask );
        StackType_t * pcEndOfStack;
        UBaseType_t uxReturn = ( UBaseType_t ) 0U;
        #if ( portSTACK_GROWTH < 0 )
            pcEndOfStack = ( StackType_t * ) pxTCB->pxStack;
        #else
            pcEndOfStack = pxTCB->pxEndOfStack;
        #endif
        while( ( *pcEndOfStack == ( StackType_t ) tskSTACK_FILL_BYTE ) && ( uxReturn < ( UBaseType_t ) pxTCB->uxStackDepth ) )
        { pcEndOfStack++; uxReturn++; }
        return uxReturn;
    }
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_MUTEXES == 1 )
    BaseType_t xTaskPriorityInherit( TaskHandle_t const pxMutexHolder )
    {
        TCB_t * const pxMutexHolderTCB = pxMutexHolder;
        BaseType_t xReturn = pdFALSE;
        if( pxMutexHolder != NULL )
        {
            if( pxMutexHolderTCB->uxPriority < pxCurrentTCB->uxPriority )
            {
                if( ( listGET_LIST_ITEM_VALUE( &( pxMutexHolderTCB->xEventListItem ) ) & taskEVENT_LIST_ITEM_VALUE_IN_USE ) == 0UL )
                    listSET_LIST_ITEM_VALUE( &( pxMutexHolderTCB->xEventListItem ),
                        ( TickType_t ) configMAX_PRIORITIES - ( TickType_t ) pxCurrentTCB->uxPriority );
                if( listIS_CONTAINED_WITHIN( &( pxReadyTasksLists[ pxMutexHolderTCB->uxPriority ] ), &( pxMutexHolderTCB->xStateListItem ) ) != pdFALSE )
                {
                    if( uxListRemove( &( pxMutexHolderTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
                        taskRESET_READY_PRIORITY( pxMutexHolderTCB->uxPriority );
                    pxMutexHolderTCB->uxPriority = pxCurrentTCB->uxPriority;
                    prvAddTaskToReadyList( pxMutexHolderTCB );
                    #if ( configUSE_PREEMPTION == 1 )
                        if( pxMutexHolderTCB->uxPriority > pxCurrentTCB->uxPriority ) xReturn = pdTRUE;
                    #endif
                }
                else
                    pxMutexHolderTCB->uxPriority = pxCurrentTCB->uxPriority;
                traceTASK_PRIORITY_INHERIT( pxMutexHolderTCB, pxCurrentTCB->uxPriority );
            }
        }
        return xReturn;
    }

    BaseType_t xTaskPriorityDisinherit( TaskHandle_t const pxMutexHolder )
    {
        TCB_t * const pxTCB = pxMutexHolder;
        BaseType_t xReturn = pdFALSE;
        if( pxMutexHolder != NULL )
        {
            configASSERT( pxTCB->uxMutexesHeld > 0 );
            ( pxTCB->uxMutexesHeld )--;
            if( pxTCB->uxPriority != pxTCB->uxBasePriority )
            {
                if( pxTCB->uxMutexesHeld == ( UBaseType_t ) 0 )
                {
                    if( listIS_CONTAINED_WITHIN( &( pxReadyTasksLists[ pxTCB->uxPriority ] ), &( pxTCB->xStateListItem ) ) != pdFALSE )
                    {
                        if( uxListRemove( &( pxTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
                            taskRESET_READY_PRIORITY( pxTCB->uxPriority );
                    }
                    traceTASK_PRIORITY_DISINHERIT( pxTCB, pxTCB->uxBasePriority );
                    pxTCB->uxPriority = pxTCB->uxBasePriority;
                    listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ),
                        ( TickType_t ) configMAX_PRIORITIES - ( TickType_t ) pxTCB->uxPriority );
                    if( listIS_CONTAINED_WITHIN( NULL, &( pxTCB->xStateListItem ) ) != pdFALSE )
                        prvAddTaskToReadyList( pxTCB );
                    #if ( configUSE_PREEMPTION == 1 )
                        if( pxTCB->uxPriority > pxCurrentTCB->uxPriority ) xReturn = pdTRUE;
                    #endif
                }
            }
        }
        return xReturn;
    }

    TaskHandle_t pvTaskIncrementMutexHeldCount( void )
    {
        if( pxCurrentTCB != NULL ) ( pxCurrentTCB->uxMutexesHeld )++;
        return pxCurrentTCB;
    }
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_RECURSIVE_MUTEXES == 1 )
    BaseType_t xTaskGiveMutexRecursive( TaskHandle_t xMutex )
    {
        Queue_t * const pxMutex = ( Queue_t * ) xMutex;
        configASSERT( pxMutex );
        if( pxMutex->xMutexHolder == xTaskGetCurrentTaskHandle() )
        {
            ( pxMutex->u.uxRecursiveCallCount )--;
            if( pxMutex->u.uxRecursiveCallCount == ( UBaseType_t ) 0 )
                ( void ) xQueueGenericSend( pxMutex, NULL, ( TickType_t ) 0, queueSEND_TO_BACK );
            return pdPASS;
        }
        return pdFAIL;
    }

    BaseType_t xTaskTakeMutexRecursive( TaskHandle_t xMutex, TickType_t xTicksToWait )
    {
        BaseType_t xReturn;
        Queue_t * const pxMutex = ( Queue_t * ) xMutex;
        configASSERT( pxMutex );
        if( pxMutex->xMutexHolder == xTaskGetCurrentTaskHandle() )
        {
            ( pxMutex->u.uxRecursiveCallCount )++;
            xReturn = pdPASS;
        }
        else
        {
            xReturn = xQueueSemaphoreTake( pxMutex, xTicksToWait );
            if( xReturn != pdFAIL ) ( pxMutex->u.uxRecursiveCallCount )++;
        }
        return xReturn;
    }
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )
    BaseType_t xTaskGenericNotify( TaskHandle_t xTaskToNotify, uint32_t ulValue,
                                   eNotifyAction eAction, uint32_t * pulPreviousNotificationValue )
    {
        TCB_t * pxTCB;
        BaseType_t xReturn = pdPASS;
        uint8_t ucOriginalNotifyState;
        configASSERT( xTaskToNotify );
        pxTCB = xTaskToNotify;
        taskENTER_CRITICAL();
        {
            if( pulPreviousNotificationValue != NULL ) *pulPreviousNotificationValue = pxTCB->ulNotifiedValue[ 0 ];
            ucOriginalNotifyState = pxTCB->ucNotifyState[ 0 ];
            pxTCB->ucNotifyState[ 0 ] = taskNOTIFICATION_RECEIVED;
            switch( eAction )
            {
                case eSetBits:                pxTCB->ulNotifiedValue[ 0 ] |= ulValue; break;
                case eIncrement:              ( pxTCB->ulNotifiedValue[ 0 ] )++; break;
                case eSetValueWithOverwrite:  pxTCB->ulNotifiedValue[ 0 ] = ulValue; break;
                case eSetValueWithoutOverwrite:
                    if( ucOriginalNotifyState != taskNOTIFICATION_RECEIVED ) pxTCB->ulNotifiedValue[ 0 ] = ulValue;
                    else xReturn = pdFAIL;
                    break;
                case eNoAction: break;
            }
            if( ucOriginalNotifyState == taskWAITING_NOTIFICATION )
            {
                ( void ) uxListRemove( &( pxTCB->xStateListItem ) );
                prvAddTaskToReadyList( pxTCB );
                configASSERT( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) == NULL );
                #if ( configUSE_PREEMPTION == 1 )
                    if( pxTCB->uxPriority > pxCurrentTCB->uxPriority ) xYieldPending = pdTRUE;
                #endif
            }
        }
        taskEXIT_CRITICAL();
        return xReturn;
    }

    BaseType_t xTaskGenericNotifyFromISR( TaskHandle_t xTaskToNotify, uint32_t ulValue,
                                          eNotifyAction eAction, uint32_t * pulPreviousNotificationValue,
                                          BaseType_t * pxHigherPriorityTaskWoken )
    {
        TCB_t * pxTCB;
        uint8_t ucOriginalNotifyState;
        BaseType_t xReturn = pdPASS;
        configASSERT( xTaskToNotify );
        pxTCB = xTaskToNotify;
        if( pulPreviousNotificationValue != NULL ) *pulPreviousNotificationValue = pxTCB->ulNotifiedValue[ 0 ];
        ucOriginalNotifyState = pxTCB->ucNotifyState[ 0 ];
        pxTCB->ucNotifyState[ 0 ] = taskNOTIFICATION_RECEIVED;
        switch( eAction )
        {
            case eSetBits:                pxTCB->ulNotifiedValue[ 0 ] |= ulValue; break;
            case eIncrement:              ( pxTCB->ulNotifiedValue[ 0 ] )++; break;
            case eSetValueWithOverwrite:  pxTCB->ulNotifiedValue[ 0 ] = ulValue; break;
            case eSetValueWithoutOverwrite:
                if( ucOriginalNotifyState != taskNOTIFICATION_RECEIVED ) pxTCB->ulNotifiedValue[ 0 ] = ulValue; else xReturn = pdFAIL;
                break;
            case eNoAction: break;
        }
        if( ucOriginalNotifyState == taskWAITING_NOTIFICATION )
        {
            ( void ) uxListRemove( &( pxTCB->xStateListItem ) );
            prvAddTaskToReadyList( pxTCB );
            if( pxTCB->uxPriority > pxCurrentTCB->uxPriority )
            {
                if( pxHigherPriorityTaskWoken != NULL ) *pxHigherPriorityTaskWoken = pdTRUE;
                xYieldPending = pdTRUE;
            }
        }
        return xReturn;
    }

    uint32_t ulTaskGenericNotifyTake( UBaseType_t uxIndexToWait, BaseType_t xClearCountOnExit, TickType_t xTicksToWait )
    {
        uint32_t ulReturn;
        configASSERT( uxIndexToWait < configTASK_NOTIFICATION_ARRAY_ENTRIES );
        taskENTER_CRITICAL();
        {
            if( pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] == 0UL )
            {
                pxCurrentTCB->ucNotifyState[ uxIndexToWait ] = taskWAITING_NOTIFICATION;
                if( xTicksToWait > ( TickType_t ) 0 ) { prvAddCurrentTaskToDelayedList( xTicksToWait, pdTRUE ); portYIELD_WITHIN_API(); }
            }
        }
        taskEXIT_CRITICAL();
        taskENTER_CRITICAL();
        {
            ulReturn = pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ];
            if( ulReturn != 0UL )
            {
                if( xClearCountOnExit != pdFALSE ) pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] = 0UL;
                else pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] = ulReturn - ( uint32_t ) 1;
            }
            pxCurrentTCB->ucNotifyState[ uxIndexToWait ] = taskNOT_WAITING_NOTIFICATION;
        }
        taskEXIT_CRITICAL();
        return ulReturn;
    }

    BaseType_t xTaskGenericNotifyWait( UBaseType_t uxIndexToWait, uint32_t ulBitsToClearOnEntry,
                                       uint32_t ulBitsToClearOnExit, uint32_t * pulNotificationValue, TickType_t xTicksToWait )
    {
        BaseType_t xReturn;
        configASSERT( uxIndexToWait < configTASK_NOTIFICATION_ARRAY_ENTRIES );
        taskENTER_CRITICAL();
        {
            if( pxCurrentTCB->ucNotifyState[ uxIndexToWait ] != taskNOTIFICATION_RECEIVED )
            {
                pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] &= ~ulBitsToClearOnEntry;
                pxCurrentTCB->ucNotifyState[ uxIndexToWait ] = taskWAITING_NOTIFICATION;
                if( xTicksToWait > ( TickType_t ) 0 ) { prvAddCurrentTaskToDelayedList( xTicksToWait, pdTRUE ); portYIELD_WITHIN_API(); }
            }
        }
        taskEXIT_CRITICAL();
        taskENTER_CRITICAL();
        {
            if( pulNotificationValue != NULL ) *pulNotificationValue = pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ];
            if( pxCurrentTCB->ucNotifyState[ uxIndexToWait ] != taskNOTIFICATION_RECEIVED ) xReturn = pdFALSE;
            else { pxCurrentTCB->ulNotifiedValue[ uxIndexToWait ] &= ~ulBitsToClearOnExit; xReturn = pdTRUE; }
            pxCurrentTCB->ucNotifyState[ uxIndexToWait ] = taskNOT_WAITING_NOTIFICATION;
        }
        taskEXIT_CRITICAL();
        return xReturn;
    }
#endif
/*-----------------------------------------------------------*/

BaseType_t xTaskIncrementTick( void )
{
    TCB_t * pxTCB;
    TickType_t xItemValue;
    BaseType_t xSwitchRequired = pdFALSE;
    traceTASK_INCREMENT_TICK( xTickCount );
    if( uxSchedulerSuspended == ( UBaseType_t ) pdFALSE )
    {
        const TickType_t xConstTickCount = xTickCount + ( TickType_t ) 1;
        xTickCount = xConstTickCount;
        if( xConstTickCount == ( TickType_t ) 0U ) taskSWITCH_DELAYED_LISTS();
        if( xConstTickCount >= xNextTaskUnblockTime )
        {
            for( ; ; )
            {
                if( listLIST_IS_EMPTY( pxDelayedTaskList ) != pdFALSE )
                { xNextTaskUnblockTime = portMAX_DELAY; break; }
                pxTCB = listGET_OWNER_OF_HEAD_ENTRY( pxDelayedTaskList );
                xItemValue = listGET_LIST_ITEM_VALUE( &( pxTCB->xStateListItem ) );
                if( xConstTickCount < xItemValue )
                { xNextTaskUnblockTime = xItemValue; break; }
                ( void ) uxListRemove( &( pxTCB->xStateListItem ) );
                if( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) != NULL )
                    ( void ) uxListRemove( &( pxTCB->xEventListItem ) );
                prvAddTaskToReadyList( pxTCB );
                #if ( configUSE_PREEMPTION == 1 )
                    if( pxTCB->uxPriority > pxCurrentTCB->uxPriority ) xSwitchRequired = pdTRUE;
                #endif
            }
        }
        #if ( ( configUSE_PREEMPTION == 1 ) && ( configUSE_TIME_SLICING == 1 ) )
            if( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ pxCurrentTCB->uxPriority ] ) ) > ( UBaseType_t ) 1 )
                xSwitchRequired = pdTRUE;
        #endif
        #if ( configUSE_TICK_HOOK == 1 )
            { extern void vApplicationTickHook( void ); vApplicationTickHook(); }
        #endif
    }
    else
    {
        ++xPendedTicks;
        #if ( configUSE_TICK_HOOK == 1 )
            { extern void vApplicationTickHook( void ); vApplicationTickHook(); }
        #endif
    }
    #if ( configUSE_PREEMPTION == 1 )
        if( xYieldPending != pdFALSE ) xSwitchRequired = pdTRUE;
    #endif
    return xSwitchRequired;
}
/*-----------------------------------------------------------*/

#if ( configUSE_TICKLESS_IDLE != 0 )
    static TickType_t prvGetExpectedIdleTime( void )
    {
        TickType_t xReturn;
        UBaseType_t uxHigherPriorityReadyTasks = pdFALSE;
        #if ( configUSE_PREEMPTION == 1 )
            if( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ tskIDLE_PRIORITY ] ) ) > 1 ) uxHigherPriorityReadyTasks = pdTRUE;
        #endif
        if( uxHigherPriorityReadyTasks != pdFALSE ) return 0;
        if( pxCurrentTCB->uxPriority > tskIDLE_PRIORITY ) return 0;
        if( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ tskIDLE_PRIORITY ] ) ) > 1 ) return 0;
        xReturn = xNextTaskUnblockTime - xTickCount;
        return xReturn;
    }

    void vTaskStepTick( const TickType_t xTicksToJump )
    {
        configASSERT( ( xTickCount + xTicksToJump ) <= xNextTaskUnblockTime );
        xTickCount += xTicksToJump;
        traceINCREASE_TICK_COUNT( xTicksToJump );
    }

    eSleepModeStatus eTaskConfirmSleepModeStatus( void )
    {
        #if ( INCLUDE_vTaskSuspend == 1 )
            if( listCURRENT_LIST_LENGTH( &xSuspendedTaskList ) > 0 ) return eAbortSleep;
        #endif
        if( xPendingReadyList.uxNumberOfItems != 0 ) return eAbortSleep;
        if( xYieldPending != pdFALSE ) return eAbortSleep;
        if( xSchedulerRunning == pdFALSE ) return eAbortSleep;
        if( uxSchedulerSuspended != 0 ) return eAbortSleep;
        return eStandardSleep;
    }
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t uxTaskGetSystemState( TaskStatus_t * const pxTaskStatusArray, const UBaseType_t uxArraySize, uint32_t * const pulTotalRunTime )
    {
        UBaseType_t uxTask = 0, uxQueue = configMAX_PRIORITIES;
        vTaskSuspendAll();
        {
            if( uxArraySize > 0U )
            {
                do { uxQueue--; } while( ( uxQueue > 0 ) && listLIST_IS_EMPTY( &( pxReadyTasksLists[ uxQueue ] ) ) );
                if( uxTask < uxArraySize )
                {
                    List_t * pxList = &( pxReadyTasksLists[ uxQueue ] );
                    ListItem_t const * pxEnd = listGET_END_MARKER( pxList );
                    ListItem_t * pxIterator = listGET_HEAD_ENTRY( pxList );
                    while( ( pxIterator != pxEnd ) && ( uxTask < uxArraySize ) )
                    {
                        TCB_t * pxTCB = listGET_LIST_ITEM_OWNER( pxIterator );
                        TaskStatus_t * const pxTaskStatus = &( pxTaskStatusArray[ uxTask ] );
                        pxTaskStatus->xHandle = ( TaskHandle_t ) pxTCB;
                        pxTaskStatus->pcTaskName = ( const char * ) &( pxTCB->pcTaskName[ 0 ] );
                        pxTaskStatus->uxCurrentPriority = pxTCB->uxPriority;
                        pxTaskStatus->eCurrentState = eReady;
                        #if ( configUSE_MUTEXES == 1 )
                            pxTaskStatus->uxBasePriority = pxTCB->uxBasePriority;
                        #else
                            pxTaskStatus->uxBasePriority = 0;
                        #endif
                        #if ( configGENERATE_RUN_TIME_STATS == 1 )
                            pxTaskStatus->ulRunTimeCounter = pxTCB->ulRunTimeCounter;
                        #else
                            pxTaskStatus->ulRunTimeCounter = 0;
                        #endif
                        pxTaskStatus->usStackHighWaterMark = 0;
                        uxTask++;
                        pxIterator = listGET_NEXT( pxIterator );
                    }
                }
            }
        }
        ( void ) xTaskResumeAll();
        if( pulTotalRunTime != NULL )
        {
            #if ( configGENERATE_RUN_TIME_STATS == 1 )
                *pulTotalRunTime = ulTotalRunTime;
            #else
                *pulTotalRunTime = 0;
            #endif
        }
        return uxTask;
    }
#endif
/*-----------------------------------------------------------*/

static void prvInitialiseTaskLists( void )
{
    UBaseType_t uxPriority;
    for( uxPriority = ( UBaseType_t ) 0U; uxPriority < ( UBaseType_t ) configMAX_PRIORITIES; uxPriority++ )
        vListInitialise( &( pxReadyTasksLists[ uxPriority ] ) );
    vListInitialise( &xDelayedTaskList1 );
    vListInitialise( &xDelayedTaskList2 );
    vListInitialise( &xPendingReadyList );
    #if ( INCLUDE_vTaskDelete == 1 )
        vListInitialise( &xTasksWaitingTermination );
    #endif
    #if ( INCLUDE_vTaskSuspend == 1 )
        vListInitialise( &xSuspendedTaskList );
    #endif
    pxDelayedTaskList = &xDelayedTaskList1;
    pxOverflowDelayedTaskList = &xDelayedTaskList2;
}
/*-----------------------------------------------------------*/

static void prvCheckTasksWaitingTermination( void )
{
    #if ( INCLUDE_vTaskDelete == 1 )
        TCB_t * pxTCB;
        while( uxDeletedTasksWaitingCleanUp > ( UBaseType_t ) 0U )
        {
            taskENTER_CRITICAL();
            {
                pxTCB = listGET_OWNER_OF_HEAD_ENTRY( &xTasksWaitingTermination );
                ( void ) uxListRemove( &( pxTCB->xStateListItem ) );
                --uxCurrentNumberOfTasks;
                --uxDeletedTasksWaitingCleanUp;
            }
            taskEXIT_CRITICAL();
            prvDeleteTCB( pxTCB );
        }
    #endif
}
/*-----------------------------------------------------------*/

static void prvDeleteTCB( TCB_t * pxTCB )
{
    portCLEAN_UP_TCB( pxTCB );
    #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
        #if ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
            if( pxTCB->ucStaticallyAllocated == tskDYNAMICALLY_ALLOCATED )
            { vPortFree( pxTCB->pxStack ); vPortFree( pxTCB ); }
        #else
            vPortFree( pxTCB->pxStack ); vPortFree( pxTCB );
        #endif
    #elif ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
        if( pxTCB->ucStaticallyAllocated == tskDYNAMICALLY_ALLOCATED )
        { vPortFree( pxTCB->pxStack ); vPortFree( pxTCB ); }
    #else
        ( void ) pxTCB;
    #endif
}
/*-----------------------------------------------------------*/

static void prvResetNextTaskUnblockTime( void )
{
    if( listLIST_IS_EMPTY( pxDelayedTaskList ) != pdFALSE )
        xNextTaskUnblockTime = portMAX_DELAY;
    else
    {
        TCB_t * pxTCB = listGET_OWNER_OF_HEAD_ENTRY( pxDelayedTaskList );
        xNextTaskUnblockTime = listGET_LIST_ITEM_VALUE( &( pxTCB->xStateListItem ) );
    }
}
/*-----------------------------------------------------------*/

void prvAddCurrentTaskToDelayedList( TickType_t xTicksToWait, const BaseType_t xCanBlockIndefinitely )
{
    TickType_t xTimeToWake;
    const TickType_t xConstTickCount = xTickCount;
    #if ( INCLUDE_xTaskAbortDelay == 1 )
        pxCurrentTCB->ucDelayAborted = pdFALSE;
    #endif
    if( uxListRemove( &( pxCurrentTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
        taskRESET_READY_PRIORITY( pxCurrentTCB->uxPriority );
    #if ( INCLUDE_vTaskSuspend == 1 )
        if( ( xTicksToWait == portMAX_DELAY ) && ( xCanBlockIndefinitely != pdFALSE ) )
            vListInsertEnd( &xSuspendedTaskList, &( pxCurrentTCB->xStateListItem ) );
        else
        {
            xTimeToWake = xConstTickCount + xTicksToWait;
            listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xStateListItem ), xTimeToWake );
            if( xTimeToWake < xConstTickCount )
                vListInsert( pxOverflowDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
            else
            {
                vListInsert( pxDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
                if( xTimeToWake < xNextTaskUnblockTime ) xNextTaskUnblockTime = xTimeToWake;
            }
        }
    #else
        xTimeToWake = xConstTickCount + xTicksToWait;
        listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xStateListItem ), xTimeToWake );
        if( xTimeToWake < xConstTickCount )
            vListInsert( pxOverflowDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
        else
        {
            vListInsert( pxDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
            if( xTimeToWake < xNextTaskUnblockTime ) xNextTaskUnblockTime = xTimeToWake;
        }
    #endif
}
/*-----------------------------------------------------------*/

static void prvIdleTask( void * pvParameters )
{
    ( void ) pvParameters;
    for( ; ; )
    {
        prvCheckTasksWaitingTermination();
        #if ( configUSE_PREEMPTION == 0 )
            taskYIELD();
        #endif
        #if ( ( configUSE_PREEMPTION == 1 ) && ( configIDLE_SHOULD_YIELD == 1 ) )
            if( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ tskIDLE_PRIORITY ] ) ) > ( UBaseType_t ) 1 )
                taskYIELD();
        #endif
        #if ( configUSE_IDLE_HOOK == 1 )
            { extern void vApplicationIdleHook( void ); vApplicationIdleHook(); }
        #endif
        #if ( configUSE_TICKLESS_IDLE != 0 )
        {
            TickType_t xExpectedIdleTime = prvGetExpectedIdleTime();
            if( xExpectedIdleTime >= configEXPECTED_IDLE_TIME_BEFORE_SLEEP )
            {
                vTaskSuspendAll();
                {
                    configASSERT( xNextTaskUnblockTime >= xTickCount );
                    xExpectedIdleTime = prvGetExpectedIdleTime();
                    if( xExpectedIdleTime >= configEXPECTED_IDLE_TIME_BEFORE_SLEEP )
                    {
                        traceLOW_POWER_IDLE_BEGIN();
                        portSUPPRESS_TICKS_AND_SLEEP( xExpectedIdleTime );
                        traceLOW_POWER_IDLE_END();
                    }
                }
                ( void ) xTaskResumeAll();
            }
        }
        #endif
    }
}
/*-----------------------------------------------------------*/
