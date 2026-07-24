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
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#if ( configUSE_TIMERS == 1 )

    #undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

    /* The definition of the timers themselves. */
    typedef struct tmrTimerControl
    {
        const char *            pcTimerName;
        ListItem_t              xTimerListItem;
        TickType_t              xTimerPeriodInTicks;
        void *                  pvTimerID;
        TimerCallbackFunction_t pxCallbackFunction;
        #if( configSUPPORT_STATIC_ALLOCATION == 1 )
            uint8_t             ucStaticallyAllocated;
        #endif
        #if( configUSE_TRACE_FACILITY == 1 )
            UBaseType_t         uxTimerNumber;
        #endif
        uint8_t                 ucStatus;
    } xTIMER;

    typedef xTIMER Timer_t;

    #define tmrCOMMAND_EXECUTE_CALLBACK_FROM_ISR   ( ( BaseType_t ) -2 )
    #define tmrCOMMAND_EXECUTE_CALLBACK            ( ( BaseType_t ) -1 )
    #define tmrCOMMAND_START_DONT_TRACE            ( ( BaseType_t ) 0 )
    #define tmrCOMMAND_START                       ( ( BaseType_t ) 1 )
    #define tmrCOMMAND_START_FROM_ISR              ( ( BaseType_t ) 2 )
    #define tmrCOMMAND_RESET                       ( ( BaseType_t ) 3 )
    #define tmrCOMMAND_RESET_FROM_ISR              ( ( BaseType_t ) 4 )
    #define tmrCOMMAND_STOP                        ( ( BaseType_t ) 5 )
    #define tmrCOMMAND_STOP_FROM_ISR               ( ( BaseType_t ) 6 )
    #define tmrCOMMAND_CHANGE_PERIOD               ( ( BaseType_t ) 7 )
    #define tmrCOMMAND_CHANGE_PERIOD_FROM_ISR      ( ( BaseType_t ) 8 )
    #define tmrCOMMAND_DELETE                      ( ( BaseType_t ) 9 )

    #define tmrSTATUS_IS_ACTIVE                    ( ( uint8_t ) 1 << 0 )
    #define tmrSTATUS_IS_STATICALLY_ALLOCATED      ( ( uint8_t ) 1 << 1 )
    #define tmrSTATUS_COMMAND_PENDING              ( ( uint8_t ) 1 << 2 )

    /* The lists in which active timers are stored. */
    static List_t xActiveTimerList1;
    static List_t xActiveTimerList2;
    static List_t * pxCurrentTimerList;
    static List_t * pxOverflowTimerList;

    /* The queue used to send commands to the timer service task. */
    static QueueHandle_t xTimerQueue = NULL;
    static TaskHandle_t xTimerTaskHandle = NULL;

    /*-----------------------------------------------------------*/

    static void prvCheckForValidListAndQueue( void );
    static void prvTimerTask( void * pvParameters );
    static void prvProcessReceivedCommands( void );
    static BaseType_t prvInsertTimerInActiveList( Timer_t * const pxTimer,
                                                   const TickType_t xNextExpiryTime,
                                                   const TickType_t xTimeNow,
                                                   const TickType_t xCommandTime );
    static void prvProcessExpiredTimer( const TickType_t xNextExpireTime,
                                        const TickType_t xTimeNow );
    static void prvSwitchTimerLists( void );
    static TickType_t prvSampleTimeNow( BaseType_t * const pxTimerListsWereSwitched );
    static TickType_t prvGetNextExpireTime( BaseType_t * const pxListWasEmpty );
    static void prvProcessTimerOrBlockTask( const TickType_t xNextExpireTime,
                                            BaseType_t xListWasEmpty );
    static void prvInitialiseNewTimer( const char * const pcTimerName,
                                       const TickType_t xTimerPeriodInTicks,
                                       const UBaseType_t uxAutoReload,
                                       void * const pvTimerID,
                                       TimerCallbackFunction_t pxCallbackFunction,
                                       Timer_t * pxNewTimer );

    /*-----------------------------------------------------------*/

    BaseType_t xTimerCreateTimerTask( void )
    {
        BaseType_t xReturn = pdFAIL;

        prvCheckForValidListAndQueue();

        if( xTimerQueue != NULL )
        {
            #if( configSUPPORT_STATIC_ALLOCATION == 1 )
            {
                StaticTask_t * pxTimerTaskTCBBuffer = NULL;
                StackType_t * pxTimerTaskStackBuffer = NULL;
                uint32_t ulTimerTaskStackSize;

                vApplicationGetTimerTaskMemory(
                    &pxTimerTaskTCBBuffer,
                    &pxTimerTaskStackBuffer,
                    &ulTimerTaskStackSize );

                xTimerTaskHandle = xTaskCreateStatic(
                    prvTimerTask,
                    "Tmr Svc",
                    ulTimerTaskStackSize,
                    NULL,
                    ( ( UBaseType_t ) configTIMER_TASK_PRIORITY ) | portPRIVILEGE_BIT,
                    pxTimerTaskStackBuffer,
                    pxTimerTaskTCBBuffer );

                if( xTimerTaskHandle != NULL )
                {
                    xReturn = pdPASS;
                }
            }
            #else
            {
                xReturn = xTaskCreate( prvTimerTask,
                    "Tmr Svc",
                    configTIMER_TASK_STACK_DEPTH,
                    NULL,
                    ( ( UBaseType_t ) configTIMER_TASK_PRIORITY ) | portPRIVILEGE_BIT,
                    &xTimerTaskHandle );
            }
            #endif
        }

        configASSERT( xReturn );
        return xReturn;
    }
    /*-----------------------------------------------------------*/

    TimerHandle_t xTimerCreate( const char * const pcTimerName,
                                const TickType_t xTimerPeriodInTicks,
                                const UBaseType_t uxAutoReload,
                                void * const pvTimerID,
                                TimerCallbackFunction_t pxCallbackFunction )
    {
        Timer_t * pxNewTimer;

        pxNewTimer = ( Timer_t * ) pvPortMalloc( sizeof( Timer_t ) );

        if( pxNewTimer != NULL )
        {
            prvInitialiseNewTimer( pcTimerName, xTimerPeriodInTicks,
                                   uxAutoReload, pvTimerID, pxCallbackFunction,
                                   pxNewTimer );

            #if( configSUPPORT_STATIC_ALLOCATION == 1 )
            {
                pxNewTimer->ucStaticallyAllocated = pdFALSE;
            }
            #endif

            traceTIMER_CREATE( pxNewTimer );
        }

        return pxNewTimer;
    }
    /*-----------------------------------------------------------*/

    #if( configSUPPORT_STATIC_ALLOCATION == 1 )

        TimerHandle_t xTimerCreateStatic( const char * const pcTimerName,
                                          const TickType_t xTimerPeriodInTicks,
                                          const UBaseType_t uxAutoReload,
                                          void * const pvTimerID,
                                          TimerCallbackFunction_t pxCallbackFunction,
                                          StaticTimer_t * pxTimerBuffer )
        {
            Timer_t * pxNewTimer;

            #if( configASSERT_DEFINED == 1 )
            {
                volatile size_t xSize = sizeof( StaticTimer_t );
                configASSERT( xSize == sizeof( Timer_t ) );
                ( void ) xSize;
            }
            #endif

            configASSERT( pxTimerBuffer );
            pxNewTimer = ( Timer_t * ) pxTimerBuffer;

            if( pxNewTimer != NULL )
            {
                prvInitialiseNewTimer( pcTimerName, xTimerPeriodInTicks,
                                       uxAutoReload, pvTimerID, pxCallbackFunction,
                                       pxNewTimer );

                pxNewTimer->ucStaticallyAllocated = pdTRUE;
            }

            return pxNewTimer;
        }

    #endif /* configSUPPORT_STATIC_ALLOCATION */
    /*-----------------------------------------------------------*/

    static void prvInitialiseNewTimer( const char * const pcTimerName,
                                       const TickType_t xTimerPeriodInTicks,
                                       const UBaseType_t uxAutoReload,
                                       void * const pvTimerID,
                                       TimerCallbackFunction_t pxCallbackFunction,
                                       Timer_t * pxNewTimer )
    {
        configASSERT( ( xTimerPeriodInTicks > 0 ) );

        if( pxNewTimer != NULL )
        {
            prvCheckForValidListAndQueue();

            pxNewTimer->pcTimerName = pcTimerName;
            pxNewTimer->xTimerPeriodInTicks = xTimerPeriodInTicks;
            pxNewTimer->pvTimerID = pvTimerID;
            pxNewTimer->pxCallbackFunction = pxCallbackFunction;
            pxNewTimer->ucStatus = ( uint8_t ) 0x00;
            vListInitialiseItem( &( pxNewTimer->xTimerListItem ) );

            if( uxAutoReload != pdFALSE )
            {
                pxNewTimer->ucStatus |= tmrSTATUS_IS_ACTIVE;
            }

            traceTIMER_CREATE( pxNewTimer );
        }
    }
    /*-----------------------------------------------------------*/

    BaseType_t xTimerGenericCommand( TimerHandle_t xTimer,
                                     const BaseType_t xCommandID,
                                     const TickType_t xOptionalValue,
                                     BaseType_t * const pxHigherPriorityTaskWoken,
                                     const TickType_t xTicksToWait )
    {
        BaseType_t xReturn = pdFAIL;
        TimerQueueMessage_t xMessage;

        if( xTimer == NULL )
        {
            return pdFAIL;
        }

        if( xCommandID < tmrFIRST_FROM_ISR_COMMAND )
        {
            if( xTimerQueue != NULL )
            {
                xMessage.xMessageID = xCommandID;
                xMessage.u.xTimerHandle = xTimer;
                xMessage.u.xOptionalValue = xOptionalValue;

                xReturn = xQueueSendToBack( xTimerQueue,
                    &xMessage, xTicksToWait );
            }
        }
        else
        {
            if( xTimerQueue != NULL )
            {
                xMessage.xMessageID = xCommandID;
                xMessage.u.xTimerHandle = xTimer;
                xMessage.u.xOptionalValue = xOptionalValue;

                xReturn = xQueueSendToBackFromISR( xTimerQueue,
                    &xMessage, pxHigherPriorityTaskWoken );
            }
        }

        traceTIMER_COMMAND_SEND( xTimer, xCommandID, xOptionalValue, xReturn );

        return xReturn;
    }
    /*-----------------------------------------------------------*/

    TaskHandle_t xTimerGetTimerDaemonTaskHandle( void )
    {
        configASSERT( ( xTimerTaskHandle != NULL ) );
        return xTimerTaskHandle;
    }
    /*-----------------------------------------------------------*/

    TickType_t xTimerGetPeriod( TimerHandle_t xTimer )
    {
        Timer_t * pxTimer = xTimer;
        configASSERT( xTimer );
        return pxTimer->xTimerPeriodInTicks;
    }
    /*-----------------------------------------------------------*/

    TickType_t xTimerGetExpiryTime( TimerHandle_t xTimer )
    {
        Timer_t * pxTimer = xTimer;
        TickType_t xReturn;

        configASSERT( xTimer );
        xReturn = listGET_LIST_ITEM_VALUE( &( pxTimer->xTimerListItem ) );
        return xReturn;
    }
    /*-----------------------------------------------------------*/

    const char * pcTimerGetName( TimerHandle_t xTimer )
    {
        Timer_t * pxTimer = xTimer;
        configASSERT( xTimer );
        return pxTimer->pcTimerName;
    }
    /*-----------------------------------------------------------*/

    static void prvProcessExpiredTimer( const TickType_t xNextExpireTime,
                                        const TickType_t xTimeNow )
    {
        BaseType_t xResult;
        Timer_t * const pxTimer = ( Timer_t * )
            listGET_OWNER_OF_HEAD_ENTRY( pxCurrentTimerList );

        ( void ) uxListRemove( &( pxTimer->xTimerListItem ) );
        traceTIMER_EXPIRED( pxTimer );

        if( ( pxTimer->ucStatus & tmrSTATUS_IS_ACTIVE ) != 0 )
        {
            pxTimer->ucStatus |= tmrSTATUS_IS_ACTIVE;

            xResult = prvInsertTimerInActiveList( pxTimer,
                ( xNextExpireTime + pxTimer->xTimerPeriodInTicks ),
                xTimeNow, xNextExpireTime );
        }
        else
        {
            xResult = pdFALSE;
        }

        pxTimer->pxCallbackFunction( ( TimerHandle_t ) pxTimer );

        if( xResult != pdFALSE )
        {
            /* The timer was re-inserted, still active. */
        }
        else
        {
            pxTimer->ucStatus &= ~tmrSTATUS_IS_ACTIVE;
        }
    }
    /*-----------------------------------------------------------*/

    static void prvTimerTask( void * pvParameters )
    {
        TickType_t xNextExpireTime;
        BaseType_t xListWasEmpty;

        ( void ) pvParameters;

        #if( configUSE_DAEMON_TASK_STARTUP_HOOK == 1 )
        {
            extern void vApplicationDaemonTaskStartupHook( void );
            vApplicationDaemonTaskStartupHook();
        }
        #endif

        for( ; ; )
        {
            xNextExpireTime = prvGetNextExpireTime( &xListWasEmpty );

            prvProcessTimerOrBlockTask( xNextExpireTime, xListWasEmpty );

            prvProcessReceivedCommands();
        }
    }
    /*-----------------------------------------------------------*/

    static void prvProcessTimerOrBlockTask( const TickType_t xNextExpireTime,
                                            BaseType_t xListWasEmpty )
    {
        TickType_t xTimeNow;
        BaseType_t xTimerListsWereSwitched;

        vTaskSuspendAll();
        {
            xTimeNow = prvSampleTimeNow( &xTimerListsWereSwitched );

            if( xTimerListsWereSwitched == pdFALSE )
            {
                if( ( xListWasEmpty == pdFALSE ) &&
                    ( xNextExpireTime <= xTimeNow ) )
                {
                    ( void ) xTaskResumeAll();
                    prvProcessExpiredTimer( xNextExpireTime, xTimeNow );
                }
            }
            else
            {
                prvSwitchTimerLists();
            }
        }
        ( void ) xTaskResumeAll();
    }
    /*-----------------------------------------------------------*/

    static TickType_t prvGetNextExpireTime( BaseType_t * const pxListWasEmpty )
    {
        TickType_t xNextExpireTime;

        *pxListWasEmpty = listLIST_IS_EMPTY( pxCurrentTimerList );

        if( *pxListWasEmpty == pdFALSE )
        {
            xNextExpireTime = listGET_ITEM_VALUE_OF_HEAD_ENTRY(
                pxCurrentTimerList );
        }
        else
        {
            xNextExpireTime = ( TickType_t ) 0;
        }

        return xNextExpireTime;
    }
    /*-----------------------------------------------------------*/

    static TickType_t prvSampleTimeNow( BaseType_t * const pxTimerListsWereSwitched )
    {
        TickType_t xTimeNow;
        static TickType_t xLastTime = ( TickType_t ) 0;

        xTimeNow = xTaskGetTickCount();

        if( xTimeNow < xLastTime )
        {
            prvSwitchTimerLists();
            *pxTimerListsWereSwitched = pdTRUE;
        }
        else
        {
            *pxTimerListsWereSwitched = pdFALSE;
        }

        xLastTime = xTimeNow;

        return xTimeNow;
    }
    /*-----------------------------------------------------------*/

    static BaseType_t prvInsertTimerInActiveList( Timer_t * const pxTimer,
                                                   const TickType_t xNextExpiryTime,
                                                   const TickType_t xTimeNow,
                                                   const TickType_t xCommandTime )
    {
        BaseType_t xProcessTimerNow = pdFALSE;

        listSET_LIST_ITEM_VALUE( &( pxTimer->xTimerListItem ), xNextExpiryTime );
        listSET_LIST_ITEM_OWNER( &( pxTimer->xTimerListItem ), pxTimer );

        if( xNextExpiryTime <= xTimeNow )
        {
            if( ( ( TickType_t ) ( xTimeNow - xCommandTime ) ) >=
                pxTimer->xTimerPeriodInTicks )
            {
                if( ( pxTimer->ucStatus & tmrSTATUS_IS_ACTIVE ) != 0 )
                {
                    listSET_LIST_ITEM_VALUE(
                        &( pxTimer->xTimerListItem ), xTimeNow );
                }
            }
            else
            {
                xProcessTimerNow = pdTRUE;
            }
        }
        else
        {
            if( xTimeNow < xCommandTime )
            {
                if( ( xNextExpiryTime >= xCommandTime ) &&
                    ( xNextExpiryTime >= xTimeNow ) )
                {
                    vListInsert( pxOverflowTimerList,
                                 &( pxTimer->xTimerListItem ) );
                }
                else
                {
                    vListInsert( pxCurrentTimerList,
                                 &( pxTimer->xTimerListItem ) );
                }
            }
            else
            {
                vListInsert( pxCurrentTimerList,
                             &( pxTimer->xTimerListItem ) );
            }
        }

        return xProcessTimerNow;
    }
    /*-----------------------------------------------------------*/

    static void prvProcessReceivedCommands( void )
    {
        TimerQueueMessage_t xMessage;
        Timer_t * pxTimer;
        BaseType_t xResult;
        TickType_t xTimeNow;

        while( xQueueReceive( xTimerQueue, &xMessage, tmrNO_DELAY ) != pdFAIL )
        {
            const BaseType_t xMessageID = xMessage.xMessageID;

            switch( xMessageID )
            {
                case tmrCOMMAND_START:
                case tmrCOMMAND_START_FROM_ISR:
                case tmrCOMMAND_START_DONT_TRACE:
                case tmrCOMMAND_RESET:
                case tmrCOMMAND_RESET_FROM_ISR:
                    pxTimer = xMessage.u.xTimerHandle;

                    if( listIS_CONTAINED_WITHIN(
                            &( pxCurrentTimerList->xListEnd ),
                            &( pxTimer->xTimerListItem ) ) == pdFALSE )
                    {
                        if( listIS_CONTAINED_WITHIN(
                                &( pxOverflowTimerList->xListEnd ),
                                &( pxTimer->xTimerListItem ) ) == pdFALSE )
                        {
                            xTimeNow = xTaskGetTickCount();
                            TickType_t xOptionalValue;

                            if( xMessageID == tmrCOMMAND_START_DONT_TRACE )
                            {
                                xOptionalValue = xTimeNow;
                            }
                            else
                            {
                                xOptionalValue = xMessage.u.xOptionalValue;
                            }

                            if( ( xMessageID == tmrCOMMAND_RESET ) ||
                                ( xMessageID == tmrCOMMAND_RESET_FROM_ISR ) )
                            {
                                if( ( pxTimer->ucStatus & tmrSTATUS_IS_ACTIVE ) == 0 )
                                {
                                    pxTimer->ucStatus |= tmrSTATUS_IS_ACTIVE;
                                }
                            }
                            else
                            {
                                if( ( pxTimer->ucStatus & tmrSTATUS_IS_ACTIVE ) == 0 )
                                {
                                    pxTimer->ucStatus |= tmrSTATUS_IS_ACTIVE;
                                }
                            }

                            xResult = prvInsertTimerInActiveList(
                                pxTimer,
                                xOptionalValue + pxTimer->xTimerPeriodInTicks,
                                xTimeNow, xOptionalValue );

                            if( xResult != pdFALSE )
                            {
                                pxTimer->pxCallbackFunction(
                                    ( TimerHandle_t ) pxTimer );

                                if( ( pxTimer->ucStatus & tmrSTATUS_IS_ACTIVE ) != 0 )
                                {
                                    xResult = prvInsertTimerInActiveList(
                                        pxTimer,
                                        xTimeNow + pxTimer->xTimerPeriodInTicks,
                                        xTimeNow, xTimeNow );

                                    if( xResult != pdFALSE )
                                    {
                                        pxTimer->pxCallbackFunction(
                                            ( TimerHandle_t ) pxTimer );
                                    }
                                }
                            }
                        }
                    }
                    break;

                case tmrCOMMAND_STOP:
                case tmrCOMMAND_STOP_FROM_ISR:
                    pxTimer = xMessage.u.xTimerHandle;
                    pxTimer->ucStatus &= ~tmrSTATUS_IS_ACTIVE;
                    ( void ) uxListRemove( &( pxTimer->xTimerListItem ) );
                    break;

                case tmrCOMMAND_CHANGE_PERIOD:
                case tmrCOMMAND_CHANGE_PERIOD_FROM_ISR:
                    pxTimer = xMessage.u.xTimerHandle;
                    pxTimer->ucStatus |= tmrSTATUS_COMMAND_PENDING;
                    pxTimer->xTimerPeriodInTicks = xMessage.u.xOptionalValue;
                    configASSERT( ( pxTimer->xTimerPeriodInTicks > 0 ) );

                    ( void ) prvInsertTimerInActiveList(
                        pxTimer,
                        ( xTaskGetTickCount() + pxTimer->xTimerPeriodInTicks ),
                        xTaskGetTickCount(), xTaskGetTickCount() );

                    pxTimer->ucStatus &= ~tmrSTATUS_COMMAND_PENDING;
                    break;

                case tmrCOMMAND_DELETE:
                    pxTimer = xMessage.u.xTimerHandle;

                    if( listIS_CONTAINED_WITHIN(
                            &( pxCurrentTimerList->xListEnd ),
                            &( pxTimer->xTimerListItem ) ) != pdFALSE )
                    {
                        ( void ) uxListRemove(
                            &( pxTimer->xTimerListItem ) );
                    }
                    else if( listIS_CONTAINED_WITHIN(
                            &( pxOverflowTimerList->xListEnd ),
                            &( pxTimer->xTimerListItem ) ) != pdFALSE )
                    {
                        ( void ) uxListRemove(
                            &( pxTimer->xTimerListItem ) );
                    }

                    traceTIMER_DELETE( pxTimer );

                    #if( configSUPPORT_STATIC_ALLOCATION == 1 )
                    {
                        if( ( pxTimer->ucStatus & tmrSTATUS_IS_STATICALLY_ALLOCATED ) == 0 )
                        {
                            vPortFree( pxTimer );
                        }
                    }
                    #else
                    {
                        vPortFree( pxTimer );
                    }
                    #endif
                    break;

                case tmrCOMMAND_EXECUTE_CALLBACK_FROM_ISR:
                case tmrCOMMAND_EXECUTE_CALLBACK:
                    xMessage.u.xCallbackParameters.pxCallbackFunction(
                        xMessage.u.xCallbackParameters.pvParameter1,
                        xMessage.u.xCallbackParameters.ulParameter2 );
                    break;

                default:
                    break;
            }
        }
    }
    /*-----------------------------------------------------------*/

    static void prvSwitchTimerLists( void )
    {
        TickType_t xNextExpireTime, xTimeNow;
        BaseType_t xResult;
        List_t * pxTemp;

        while( listLIST_IS_EMPTY( pxCurrentTimerList ) == pdFALSE )
        {
            xTimeNow = xTaskGetTickCount();
            xNextExpireTime = listGET_ITEM_VALUE_OF_HEAD_ENTRY(
                pxCurrentTimerList );
            Timer_t * pxExpiredTimer = ( Timer_t * )
                listGET_OWNER_OF_HEAD_ENTRY( pxCurrentTimerList );

            ( void ) uxListRemove( &( pxExpiredTimer->xTimerListItem ) );
            traceTIMER_EXPIRED( pxExpiredTimer );
            pxExpiredTimer->pxCallbackFunction(
                ( TimerHandle_t ) pxExpiredTimer );

            if( ( pxExpiredTimer->ucStatus & tmrSTATUS_IS_ACTIVE ) != 0 )
            {
                xResult = prvInsertTimerInActiveList(
                    pxExpiredTimer,
                    ( xNextExpireTime + pxExpiredTimer->xTimerPeriodInTicks ),
                    xTimeNow, xNextExpireTime );

                ( void ) xResult;
            }
        }

        pxTemp = pxCurrentTimerList;
        pxCurrentTimerList = pxOverflowTimerList;
        pxOverflowTimerList = pxTemp;
    }
    /*-----------------------------------------------------------*/

    static void prvCheckForValidListAndQueue( void )
    {
        taskENTER_CRITICAL();
        {
            if( xTimerQueue == NULL )
            {
                vListInitialise( &xActiveTimerList1 );
                vListInitialise( &xActiveTimerList2 );
                pxCurrentTimerList = &xActiveTimerList1;
                pxOverflowTimerList = &xActiveTimerList2;

                #if( configSUPPORT_STATIC_ALLOCATION == 1 )
                {
                    StaticQueue_t * pxStaticTimerQueue = NULL;
                    uint8_t * pucStaticTimerQueueStorage = NULL;
                    uint32_t ulStaticTimerQueueSize = 0;

                    vApplicationGetTimerTaskMemory(
                        NULL, NULL, NULL );

                    /* Fallback: create dynamically if no static memory provided. */
                    xTimerQueue = xQueueCreate(
                        ( UBaseType_t ) configTIMER_QUEUE_LENGTH,
                        ( UBaseType_t ) sizeof( TimerQueueMessage_t ) );
                }
                #else
                {
                    xTimerQueue = xQueueCreate(
                        ( UBaseType_t ) configTIMER_QUEUE_LENGTH,
                        ( UBaseType_t ) sizeof( TimerQueueMessage_t ) );
                }
                #endif

                #if( configQUEUE_REGISTRY_SIZE > 0 )
                {
                    if( xTimerQueue != NULL )
                    {
                        vQueueAddToRegistry( xTimerQueue, "TmrQ" );
                    }
                }
                #endif
            }
        }
        taskEXIT_CRITICAL();
    }
    /*-----------------------------------------------------------*/

    BaseType_t xTimerIsTimerActive( TimerHandle_t xTimer )
    {
        BaseType_t xReturn;
        Timer_t * pxTimer = xTimer;

        configASSERT( xTimer );

        taskENTER_CRITICAL();
        {
            if( ( pxTimer->ucStatus & tmrSTATUS_IS_ACTIVE ) == 0 )
            {
                xReturn = pdFALSE;
            }
            else
            {
                xReturn = pdTRUE;
            }
        }
        taskEXIT_CRITICAL();

        return xReturn;
    }
    /*-----------------------------------------------------------*/

    void * pvTimerGetTimerID( const TimerHandle_t xTimer )
    {
        Timer_t * const pxTimer = xTimer;
        void * pvReturn;

        configASSERT( xTimer );

        taskENTER_CRITICAL();
        {
            pvReturn = pxTimer->pvTimerID;
        }
        taskEXIT_CRITICAL();

        return pvReturn;
    }
    /*-----------------------------------------------------------*/

    void vTimerSetTimerID( TimerHandle_t xTimer, void * pvNewID )
    {
        Timer_t * const pxTimer = xTimer;

        configASSERT( xTimer );

        taskENTER_CRITICAL();
        {
            pxTimer->pvTimerID = pvNewID;
        }
        taskEXIT_CRITICAL();
    }
    /*-----------------------------------------------------------*/

    #if( configUSE_TRACE_FACILITY == 1 )

        UBaseType_t uxTimerGetTimerNumber( TimerHandle_t xTimer )
        {
            return ( ( Timer_t * ) xTimer )->uxTimerNumber;
        }

    #endif
    /*-----------------------------------------------------------*/

    #if ( ( configUSE_TRACE_FACILITY == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )

        void vTimerSetTimerNumber( TimerHandle_t xTimer, UBaseType_t uxTimerNumber )
        {
            ( ( Timer_t * ) xTimer )->uxTimerNumber = uxTimerNumber;
        }

    #endif
    /*-----------------------------------------------------------*/

    #if( INCLUDE_xTimerPendFunctionCall == 1 )

        BaseType_t xTimerPendFunctionCallFromISR(
            PendedFunction_t xFunctionToPend,
            void * pvParameter1,
            uint32_t ulParameter2,
            BaseType_t * pxHigherPriorityTaskWoken )
        {
            TimerQueueMessage_t xMessage;

            xMessage.xMessageID = tmrCOMMAND_EXECUTE_CALLBACK_FROM_ISR;
            xMessage.u.xCallbackParameters.pxCallbackFunction = xFunctionToPend;
            xMessage.u.xCallbackParameters.pvParameter1 = pvParameter1;
            xMessage.u.xCallbackParameters.ulParameter2 = ulParameter2;

            if( xTimerQueue != NULL )
            {
                return xQueueSendToBackFromISR( xTimerQueue,
                    &xMessage, pxHigherPriorityTaskWoken );
            }

            return pdFAIL;
        }

    #endif /* INCLUDE_xTimerPendFunctionCall */
    /*-----------------------------------------------------------*/

    #if( INCLUDE_xTimerPendFunctionCall == 1 )

        BaseType_t xTimerPendFunctionCall(
            PendedFunction_t xFunctionToPend,
            void * pvParameter1,
            uint32_t ulParameter2,
            TickType_t xTicksToWait )
        {
            TimerQueueMessage_t xMessage;

            xMessage.xMessageID = tmrCOMMAND_EXECUTE_CALLBACK;
            xMessage.u.xCallbackParameters.pxCallbackFunction = xFunctionToPend;
            xMessage.u.xCallbackParameters.pvParameter1 = pvParameter1;
            xMessage.u.xCallbackParameters.ulParameter2 = ulParameter2;

            return xQueueSendToBack( xTimerQueue, &xMessage, xTicksToWait );
        }

    #endif /* INCLUDE_xTimerPendFunctionCall */
    /*-----------------------------------------------------------*/

#endif /* configUSE_TIMERS == 1 */
