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
#include <string.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/*-----------------------------------------------------------*/

/* Queue type encoding. */
#define queueQUEUE_TYPE_BASE                ( ( uint8_t ) 0U )
#define queueQUEUE_TYPE_MUTEX               ( ( uint8_t ) 1U )
#define queueQUEUE_TYPE_RECURSIVE_MUTEX     ( ( uint8_t ) 2U )
#define queueQUEUE_TYPE_BINARY_SEMAPHORE    ( ( uint8_t ) 3U )
#define queueQUEUE_TYPE_COUNTING_SEMAPHORE  ( ( uint8_t ) 4U )

/* Send position encoding. */
#define queueSEND_TO_FRONT                  ( ( BaseType_t ) 1 )
#define queueOVERWRITE                      ( ( BaseType_t ) 2 )

/* Queue locking constants. */
#define queueUNLOCKED                       ( ( int8_t ) -1 )
#define queueLOCKED_UNMODIFIED              ( ( int8_t ) 0 )
#define queueINT8_MAX                       ( ( int8_t ) 127 )

/* Mutex sentinel. */
#define queueMUTEX_NO_HOLDER                ( ( TaskHandle_t ) NULL )

#define queueQUEUE_IS_MUTEX( pxQueue ) \
    ( ( ( pxQueue )->ucQueueType == queueQUEUE_TYPE_MUTEX ) || \
      ( ( pxQueue )->ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX ) )

/*-----------------------------------------------------------*/

/* QueueDefinition struct and Queue_t typedef are in queue.h (shared with tasks.c) */

/*-----------------------------------------------------------*/

/* Queue registry. */
#if ( configQUEUE_REGISTRY_SIZE > 0 )

    typedef struct QUEUE_REGISTRY_ITEM
    {
        const char *   pcQueueName;
        QueueHandle_t  xHandle;
    } xQueueRegistryItem;

    static xQueueRegistryItem xQueueRegistry[ configQUEUE_REGISTRY_SIZE ];

#endif

/*-----------------------------------------------------------*/

/* Private function prototypes. */
static BaseType_t prvCopyDataToQueue( Queue_t * const pxQueue,
                                      const void * pvItemToQueue,
                                      const BaseType_t xPosition );
static void prvCopyDataFromQueue( Queue_t * const pxQueue,
                                  void * const pvBuffer );
static void prvUnlockQueue( Queue_t * const pxQueue );
static BaseType_t prvIsQueueEmpty( const Queue_t * pxQueue );
static BaseType_t prvIsQueueFull( const Queue_t * pxQueue );

#if ( configUSE_MUTEXES == 1 )
    static BaseType_t prvPriorityDisinheritAfterTimeout( Queue_t * const pxQueue,
        TaskHandle_t xMutexHolder );
#endif

/*-----------------------------------------------------------*/

static BaseType_t prvCopyDataToQueue( Queue_t * const pxQueue,
                                      const void * pvItemToQueue,
                                      const BaseType_t xPosition )
{
    BaseType_t xReturn = pdFALSE;

    if( pxQueue->uxItemSize == ( UBaseType_t ) 0 )
    {
        #if ( configUSE_MUTEXES == 1 )
        {
            if( pxQueue->ucQueueTypeInternal == queueQUEUE_TYPE_MUTEX )
            {
                xReturn = prvPriorityDisinheritAfterTimeout( pxQueue,
                    pxQueue->xMutexHolder );
                pxQueue->xMutexHolder = NULL;
            }
        }
        #endif
    }
    else if( xPosition == queueSEND_TO_BACK )
    {
        ( void ) memcpy( ( void * ) pxQueue->pcWriteTo,
                         pvItemToQueue,
                         ( size_t ) pxQueue->uxItemSize );
        pxQueue->pcWriteTo += pxQueue->uxItemSize;
        if( pxQueue->pcWriteTo >= ( pxQueue->pcHead +
             ( int8_t )( pxQueue->uxLength * pxQueue->uxItemSize ) ) )
        {
            pxQueue->pcWriteTo = pxQueue->pcHead;
        }
    }
    else if( xPosition == queueSEND_TO_FRONT )
    {
        ( void ) memcpy( ( void * ) pxQueue->u.pcReadFrom,
                         pvItemToQueue,
                         ( size_t ) pxQueue->uxItemSize );
        pxQueue->u.pcReadFrom -= pxQueue->uxItemSize;
        if( pxQueue->u.pcReadFrom < pxQueue->pcHead )
        {
            pxQueue->u.pcReadFrom =
                ( pxQueue->pcHead +
                  ( int8_t )( ( pxQueue->uxLength - ( UBaseType_t ) 1U ) *
                               pxQueue->uxItemSize ) );
        }
    }
    else /* queueOVERWRITE */
    {
        configASSERT( pxQueue->uxLength == 1 );
        ( void ) memcpy( ( void * ) pxQueue->pcHead,
                         pvItemToQueue,
                         ( size_t ) pxQueue->uxItemSize );
        pxQueue->u.pcReadFrom = pxQueue->pcHead;
        pxQueue->pcWriteTo = pxQueue->pcHead + pxQueue->uxItemSize;
    }

    pxQueue->uxMessagesWaiting = pxQueue->uxMessagesWaiting + ( UBaseType_t ) 1;
    return xReturn;
}
/*-----------------------------------------------------------*/

static void prvCopyDataFromQueue( Queue_t * const pxQueue,
                                  void * const pvBuffer )
{
    if( pxQueue->uxItemSize != ( UBaseType_t ) 0 )
    {
        pxQueue->u.pcReadFrom += pxQueue->uxItemSize;
        if( pxQueue->u.pcReadFrom >= ( pxQueue->pcHead +
             ( int8_t )( pxQueue->uxLength * pxQueue->uxItemSize ) ) )
        {
            pxQueue->u.pcReadFrom = pxQueue->pcHead;
        }
        ( void ) memcpy( ( void * ) pvBuffer,
                         ( void * ) pxQueue->u.pcReadFrom,
                         ( size_t ) pxQueue->uxItemSize );
    }
}
/*-----------------------------------------------------------*/

#if ( configUSE_MUTEXES == 1 )

    static BaseType_t prvPriorityDisinheritAfterTimeout( Queue_t * const pxQueue,
        TaskHandle_t xMutexHolder )
    {
        BaseType_t xReturn = pdFALSE;
        TCB_t * pxTCB;

        if( xMutexHolder != NULL )
        {
            pxTCB = ( TCB_t * ) xMutexHolder;
            configASSERT( pxTCB->uxMutexesHeld > 0U );
            pxTCB->uxMutexesHeld--;

            if( pxTCB->uxPriority != pxTCB->uxBasePriority )
            {
                if( pxTCB->uxMutexesHeld == 0U )
                {
                    /* Restore base priority.  Will be moved to the
                     * correct ready list when the task next runs. */
                    pxTCB->uxPriority = pxTCB->uxBasePriority;
                    listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ),
                        ( TickType_t ) configMAX_PRIORITIES -
                        ( TickType_t ) pxTCB->uxBasePriority );
                    xReturn = pdTRUE;
                }
            }
        }
        return xReturn;
    }

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

static void prvUnlockQueue( Queue_t * const pxQueue )
{
    taskENTER_CRITICAL();
    {
        int8_t cTxLock = pxQueue->cTxLock;
        int8_t cRxLock = pxQueue->cRxLock;

        while( cTxLock > queueLOCKED_UNMODIFIED )
        {
            #if ( configUSE_QUEUE_SETS == 1 )
            {
                if( pxQueue->pxQueueSetContainer != NULL )
                {
                    /* Queue sets not fully implemented in this port. */
                }
                else
            #endif
                {
                    if( listLIST_IS_EMPTY(
                            &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                    {
                        if( xTaskRemoveFromEventList(
                                &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                        {
                            vTaskMissedYield();
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            #if ( configUSE_QUEUE_SETS == 1 )
            }
            #endif
            cTxLock--;
        }

        while( cRxLock > queueLOCKED_UNMODIFIED )
        {
            if( listLIST_IS_EMPTY(
                    &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
            {
                if( xTaskRemoveFromEventList(
                        &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                {
                    vTaskMissedYield();
                }
            }
            else
            {
                break;
            }
            cRxLock--;
        }

        pxQueue->cTxLock = queueUNLOCKED;
        pxQueue->cRxLock = queueUNLOCKED;
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

static BaseType_t prvIsQueueEmpty( const Queue_t * pxQueue )
{
    BaseType_t xReturn;
    taskENTER_CRITICAL();
    {
        xReturn = ( pxQueue->uxMessagesWaiting == ( UBaseType_t ) 0 ) ?
                  pdTRUE : pdFALSE;
    }
    taskEXIT_CRITICAL();
    return xReturn;
}
/*-----------------------------------------------------------*/

static BaseType_t prvIsQueueFull( const Queue_t * pxQueue )
{
    BaseType_t xReturn;
    taskENTER_CRITICAL();
    {
        xReturn = ( pxQueue->uxMessagesWaiting == pxQueue->uxLength ) ?
                  pdTRUE : pdFALSE;
    }
    taskEXIT_CRITICAL();
    return xReturn;
}
/*-----------------------------------------------------------*/

/*
 * Generic queue create.
 */
QueueHandle_t xQueueGenericCreate( const UBaseType_t uxQueueLength,
                                   const UBaseType_t uxItemSize,
                                   const uint8_t ucQueueType )
{
    Queue_t * pxNewQueue;
    size_t xQueueSizeInBytes;

    configASSERT( uxQueueLength > ( UBaseType_t ) 0 );

    if( uxItemSize == ( UBaseType_t ) 0 )
    {
        xQueueSizeInBytes = sizeof( Queue_t );
    }
    else
    {
        xQueueSizeInBytes = sizeof( Queue_t ) +
            ( ( size_t ) uxQueueLength * ( size_t ) uxItemSize );
    }

    pxNewQueue = ( Queue_t * ) pvPortMalloc( xQueueSizeInBytes );

    if( pxNewQueue != NULL )
    {
        ( void ) memset( pxNewQueue, 0x00, xQueueSizeInBytes );

        if( uxItemSize == ( UBaseType_t ) 0 )
        {
            pxNewQueue->pcHead = ( int8_t * ) pxNewQueue;
        }
        else
        {
            pxNewQueue->pcHead = ( int8_t * ) pxNewQueue + sizeof( Queue_t );
        }

        pxNewQueue->uxLength    = uxQueueLength;
        pxNewQueue->uxItemSize  = uxItemSize;

        pxNewQueue->u.pcReadFrom = pxNewQueue->pcHead +
            ( int8_t )( ( uxQueueLength - ( UBaseType_t ) 1U ) * uxItemSize );
        pxNewQueue->pcWriteTo   = pxNewQueue->pcHead;
        pxNewQueue->cTxLock     = queueUNLOCKED;
        pxNewQueue->cRxLock     = queueUNLOCKED;
        pxNewQueue->ucQueueTypeInternal = ucQueueType;

        vListInitialise( &( pxNewQueue->xTasksWaitingToSend ) );
        vListInitialise( &( pxNewQueue->xTasksWaitingToReceive ) );

        #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        {
            pxNewQueue->ucStaticallyAllocated = pdFALSE;
        }
        #endif

        traceQUEUE_CREATE( pxNewQueue );
    }
    else
    {
        traceQUEUE_CREATE_FAILED( ucQueueType );
    }

    return ( QueueHandle_t ) pxNewQueue;
}
/*-----------------------------------------------------------*/

QueueHandle_t xQueueCreate( UBaseType_t uxQueueLength,
                            UBaseType_t uxItemSize )
{
    return xQueueGenericCreate( uxQueueLength, uxItemSize,
                                queueQUEUE_TYPE_BASE );
}
/*-----------------------------------------------------------*/

QueueHandle_t xQueueCreateStatic( UBaseType_t uxQueueLength,
                                  UBaseType_t uxItemSize,
                                  uint8_t * pucQueueStorage,
                                  void * pxQueueBuffer )
{
    ( void ) pucQueueStorage;
    ( void ) pxQueueBuffer;
    return xQueueGenericCreate( uxQueueLength, uxItemSize,
                                queueQUEUE_TYPE_BASE );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericSend( QueueHandle_t xQueue,
                              const void * const pvItemToQueue,
                              TickType_t xTicksToWait,
                              const BaseType_t xCopyPosition )
{
    BaseType_t xEntryTimeSet = pdFALSE;
    TimeOut_t xTimeOut;
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;

    configASSERT( pxQueue );
    configASSERT( !( ( pvItemToQueue == NULL ) &&
                     ( pxQueue->uxItemSize != ( UBaseType_t ) 0U ) ) );
    configASSERT( !( ( xCopyPosition == queueOVERWRITE ) &&
                     ( pxQueue->uxLength != 1 ) ) );

    #if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
    {
        configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) &&
                         ( xTicksToWait != 0 ) ) );
    }
    #endif

    for( ; ; )
    {
        taskENTER_CRITICAL();
        {
            if( ( pxQueue->uxMessagesWaiting < pxQueue->uxLength ) ||
                ( xCopyPosition == queueOVERWRITE ) )
            {
                traceQUEUE_SEND( pxQueue );
                ( void ) prvCopyDataToQueue( pxQueue, pvItemToQueue,
                    xCopyPosition );

                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                    {
                        queueYIELD_IF_USING_PREEMPTION();
                    }
                }
                else if( xCopyPosition == queueOVERWRITE )
                {
                    /* Overwriting an item — no wake needed unless
                     * a sender was waiting.  Sender waking is
                     * handled by prvCopyDataToQueue for mutexes. */
                }

                taskEXIT_CRITICAL();
                return pdPASS;
            }
            else
            {
                if( xTicksToWait == ( TickType_t ) 0 )
                {
                    taskEXIT_CRITICAL();
                    traceQUEUE_SEND_FAILED( pxQueue );
                    return errQUEUE_FULL;
                }
                else if( xEntryTimeSet == pdFALSE )
                {
                    vTaskInternalSetTimeOutState( &xTimeOut );
                    xEntryTimeSet = pdTRUE;
                }
            }
        }
        taskEXIT_CRITICAL();

        vTaskSuspendAll();
        prvUnlockQueue( pxQueue );

        if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == pdFALSE )
        {
            if( prvIsQueueFull( pxQueue ) != pdFALSE )
            {
                traceBLOCKING_ON_QUEUE_SEND( pxQueue );
                vTaskPlaceOnEventList(
                    &( pxQueue->xTasksWaitingToSend ), xTicksToWait );
                prvUnlockQueue( pxQueue );

                if( xTaskResumeAll() == pdFALSE )
                {
                    portYIELD_WITHIN_API();
                }
            }
            else
            {
                prvUnlockQueue( pxQueue );
                ( void ) xTaskResumeAll();
            }
        }
        else
        {
            prvUnlockQueue( pxQueue );
            ( void ) xTaskResumeAll();
            traceQUEUE_SEND_FAILED( pxQueue );
            return errQUEUE_FULL;
        }
    }
}
/*-----------------------------------------------------------*/

BaseType_t xQueueSend( QueueHandle_t xQueue,
                       const void * pvItemToQueue,
                       TickType_t xTicksToWait )
{
    return xQueueGenericSend( xQueue, pvItemToQueue,
                              xTicksToWait, queueSEND_TO_BACK );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueSendToBack( QueueHandle_t xQueue,
                             const void * pvItemToQueue,
                             TickType_t xTicksToWait )
{
    return xQueueGenericSend( xQueue, pvItemToQueue,
                              xTicksToWait, queueSEND_TO_BACK );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueSendToFront( QueueHandle_t xQueue,
                              const void * pvItemToQueue,
                              TickType_t xTicksToWait )
{
    return xQueueGenericSend( xQueue, pvItemToQueue,
                              xTicksToWait, queueSEND_TO_FRONT );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueOverwrite( QueueHandle_t xQueue,
                            const void * pvItemToQueue )
{
    return xQueueGenericSend( xQueue, pvItemToQueue,
                              ( TickType_t ) 0, queueOVERWRITE );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueSendFromISR( QueueHandle_t xQueue,
                              const void * pvItemToQueue,
                              BaseType_t * pxHigherPriorityTaskWoken )
{
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;
    BaseType_t xReturn;
    UBaseType_t uxSavedInterruptStatus;

    configASSERT( pxQueue );
    configASSERT( !( ( pvItemToQueue == NULL ) &&
                     ( pxQueue->uxItemSize != ( UBaseType_t ) 0U ) ) );

    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        if( ( pxQueue->uxMessagesWaiting < pxQueue->uxLength ) ||
            ( pxQueue->cTxLock != queueUNLOCKED ) )
        {
            const int8_t cTxLock = pxQueue->cTxLock;
            traceQUEUE_SEND_FROM_ISR( pxQueue );
            ( void ) prvCopyDataToQueue( pxQueue, pvItemToQueue,
                queueSEND_TO_BACK );

            if( cTxLock == queueUNLOCKED )
            {
                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                    {
                        if( pxHigherPriorityTaskWoken != NULL )
                            *pxHigherPriorityTaskWoken = pdTRUE;
                    }
                }
            }
            else
            {
                configASSERT( cTxLock != queueINT8_MAX );
                pxQueue->cTxLock = ( int8_t )( cTxLock + 1 );
            }

            xReturn = pdPASS;
        }
        else
        {
            traceQUEUE_SEND_FROM_ISR_FAILED( pxQueue );
            xReturn = errQUEUE_FULL;
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );

    return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueSendToBackFromISR( QueueHandle_t xQueue,
                                    const void * pvItemToQueue,
                                    BaseType_t * pxHigherPriorityTaskWoken )
{
    return xQueueSendFromISR( xQueue, pvItemToQueue,
                              pxHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGiveFromISR( QueueHandle_t xQueue,
                              BaseType_t * pxHigherPriorityTaskWoken )
{
    return xQueueSendFromISR( xQueue, NULL, pxHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

BaseType_t xQueueReceive( QueueHandle_t xQueue,
                          void * pvBuffer,
                          TickType_t xTicksToWait )
{
    BaseType_t xEntryTimeSet = pdFALSE;
    TimeOut_t xTimeOut;
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;

    configASSERT( pxQueue );
    configASSERT( !( ( pvBuffer == NULL ) &&
                     ( pxQueue->uxItemSize != ( UBaseType_t ) 0U ) ) );

    #if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
    {
        configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) &&
                         ( xTicksToWait != 0 ) ) );
    }
    #endif

    for( ; ; )
    {
        taskENTER_CRITICAL();
        {
            const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;

            if( uxMessagesWaiting > ( UBaseType_t ) 0 )
            {
                prvCopyDataFromQueue( pxQueue, pvBuffer );
                traceQUEUE_RECEIVE( pxQueue );
                pxQueue->uxMessagesWaiting = uxMessagesWaiting - ( UBaseType_t ) 1;

                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                    {
                        queueYIELD_IF_USING_PREEMPTION();
                    }
                }

                taskEXIT_CRITICAL();
                return pdPASS;
            }
            else
            {
                if( xTicksToWait == ( TickType_t ) 0 )
                {
                    taskEXIT_CRITICAL();
                    traceQUEUE_RECEIVE_FAILED( pxQueue );
                    return errQUEUE_EMPTY;
                }
                else if( xEntryTimeSet == pdFALSE )
                {
                    vTaskInternalSetTimeOutState( &xTimeOut );
                    xEntryTimeSet = pdTRUE;
                }
            }
        }
        taskEXIT_CRITICAL();

        vTaskSuspendAll();
        prvUnlockQueue( pxQueue );

        if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == pdFALSE )
        {
            if( prvIsQueueEmpty( pxQueue ) != pdFALSE )
            {
                traceBLOCKING_ON_QUEUE_RECEIVE( pxQueue );
                vTaskPlaceOnEventList(
                    &( pxQueue->xTasksWaitingToReceive ), xTicksToWait );
                prvUnlockQueue( pxQueue );

                if( xTaskResumeAll() == pdFALSE )
                {
                    portYIELD_WITHIN_API();
                }
            }
            else
            {
                prvUnlockQueue( pxQueue );
                ( void ) xTaskResumeAll();
            }
        }
        else
        {
            prvUnlockQueue( pxQueue );
            ( void ) xTaskResumeAll();

            if( prvIsQueueEmpty( pxQueue ) != pdFALSE )
            {
                traceQUEUE_RECEIVE_FAILED( pxQueue );
                return errQUEUE_EMPTY;
            }
        }
    }
}
/*-----------------------------------------------------------*/

BaseType_t xQueuePeek( QueueHandle_t xQueue,
                       void * pvBuffer,
                       TickType_t xTicksToWait )
{
    BaseType_t xEntryTimeSet = pdFALSE;
    TimeOut_t xTimeOut;
    int8_t * pcOriginalReadPosition;
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;

    configASSERT( pxQueue );
    configASSERT( !( ( pvBuffer == NULL ) &&
                     ( pxQueue->uxItemSize != ( UBaseType_t ) 0U ) ) );

    for( ; ; )
    {
        taskENTER_CRITICAL();
        {
            if( pxQueue->uxMessagesWaiting > ( UBaseType_t ) 0 )
            {
                pcOriginalReadPosition = pxQueue->u.pcReadFrom;
                prvCopyDataFromQueue( pxQueue, pvBuffer );
                traceQUEUE_PEEK( pxQueue );
                pxQueue->u.pcReadFrom = pcOriginalReadPosition;
                pxQueue->uxMessagesWaiting++;

                taskEXIT_CRITICAL();
                return pdPASS;
            }
            else
            {
                if( xTicksToWait == ( TickType_t ) 0 )
                {
                    taskEXIT_CRITICAL();
                    traceQUEUE_PEEK_FAILED( pxQueue );
                    return errQUEUE_EMPTY;
                }
                else if( xEntryTimeSet == pdFALSE )
                {
                    vTaskInternalSetTimeOutState( &xTimeOut );
                    xEntryTimeSet = pdTRUE;
                }
            }
        }
        taskEXIT_CRITICAL();

        vTaskSuspendAll();
        prvUnlockQueue( pxQueue );

        if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == pdFALSE )
        {
            if( prvIsQueueEmpty( pxQueue ) != pdFALSE )
            {
                traceBLOCKING_ON_QUEUE_PEEK( pxQueue );
                vTaskPlaceOnEventList(
                    &( pxQueue->xTasksWaitingToReceive ), xTicksToWait );
                prvUnlockQueue( pxQueue );

                if( xTaskResumeAll() == pdFALSE )
                {
                    portYIELD_WITHIN_API();
                }
            }
            else
            {
                prvUnlockQueue( pxQueue );
                ( void ) xTaskResumeAll();
            }
        }
        else
        {
            prvUnlockQueue( pxQueue );
            ( void ) xTaskResumeAll();
            traceQUEUE_PEEK_FAILED( pxQueue );
            return errQUEUE_EMPTY;
        }
    }
}
/*-----------------------------------------------------------*/

BaseType_t xQueueSemaphoreTake( QueueHandle_t xQueue,
                                TickType_t xTicksToWait )
{
    BaseType_t xEntryTimeSet = pdFALSE;
    TimeOut_t xTimeOut;
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;

    configASSERT( pxQueue );
    configASSERT( pxQueue->uxItemSize == 0 );

    #if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
    {
        configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) &&
                         ( xTicksToWait != 0 ) ) );
    }
    #endif

    for( ; ; )
    {
        taskENTER_CRITICAL();
        {
            const UBaseType_t uxSemaphoreCount = pxQueue->uxMessagesWaiting;

            if( uxSemaphoreCount > ( UBaseType_t ) 0 )
            {
                traceQUEUE_RECEIVE( pxQueue );
                pxQueue->uxMessagesWaiting = uxSemaphoreCount - ( UBaseType_t ) 1;

                #if ( configUSE_MUTEXES == 1 )
                {
                    if( pxQueue->ucQueueTypeInternal ==
                        queueQUEUE_TYPE_MUTEX )
                    {
                        pxQueue->xMutexHolder =
                            ( TaskHandle_t ) pxCurrentTCB;
                        pxCurrentTCB->uxMutexesHeld++;
                    }
                }
                #endif

                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                    {
                        queueYIELD_IF_USING_PREEMPTION();
                    }
                }

                taskEXIT_CRITICAL();
                return pdPASS;
            }
            else
            {
                if( xTicksToWait == ( TickType_t ) 0 )
                {
                    taskEXIT_CRITICAL();
                    traceQUEUE_RECEIVE_FAILED( pxQueue );
                    return errQUEUE_EMPTY;
                }
                else if( xEntryTimeSet == pdFALSE )
                {
                    vTaskInternalSetTimeOutState( &xTimeOut );
                    xEntryTimeSet = pdTRUE;
                }
            }
        }
        taskEXIT_CRITICAL();

        vTaskSuspendAll();
        prvUnlockQueue( pxQueue );

        if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == pdFALSE )
        {
            if( prvIsQueueEmpty( pxQueue ) != pdFALSE )
            {
                traceBLOCKING_ON_QUEUE_RECEIVE( pxQueue );
                vTaskPlaceOnEventList(
                    &( pxQueue->xTasksWaitingToReceive ), xTicksToWait );
                prvUnlockQueue( pxQueue );

                if( xTaskResumeAll() == pdFALSE )
                {
                    portYIELD_WITHIN_API();
                }
            }
            else
            {
                prvUnlockQueue( pxQueue );
                ( void ) xTaskResumeAll();
            }
        }
        else
        {
            prvUnlockQueue( pxQueue );
            ( void ) xTaskResumeAll();

            if( prvIsQueueEmpty( pxQueue ) != pdFALSE )
            {
                traceQUEUE_RECEIVE_FAILED( pxQueue );
                return errQUEUE_EMPTY;
            }
        }
    }
}
/*-----------------------------------------------------------*/

BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue,
                                 void * pvBuffer,
                                 BaseType_t * pxHigherPriorityTaskWoken )
{
    BaseType_t xReturn;
    UBaseType_t uxSavedInterruptStatus;
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;

    configASSERT( pxQueue );
    configASSERT( !( ( pvBuffer == NULL ) &&
                     ( pxQueue->uxItemSize != ( UBaseType_t ) 0U ) ) );

    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;

        if( uxMessagesWaiting > ( UBaseType_t ) 0 )
        {
            const int8_t cRxLock = pxQueue->cRxLock;

            traceQUEUE_RECEIVE_FROM_ISR( pxQueue );
            prvCopyDataFromQueue( pxQueue, pvBuffer );
            pxQueue->uxMessagesWaiting = uxMessagesWaiting - ( UBaseType_t ) 1;

            if( cRxLock == queueUNLOCKED )
            {
                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                    {
                        if( pxHigherPriorityTaskWoken != NULL )
                            *pxHigherPriorityTaskWoken = pdTRUE;
                    }
                }
            }
            else
            {
                configASSERT( cRxLock != queueINT8_MAX );
                pxQueue->cRxLock = ( int8_t )( cRxLock + 1 );
            }

            xReturn = pdPASS;
        }
        else
        {
            xReturn = pdFAIL;
            traceQUEUE_RECEIVE_FROM_ISR_FAILED( pxQueue );
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );

    return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueuePeekFromISR( QueueHandle_t xQueue,
                              void * pvBuffer )
{
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;
    BaseType_t xReturn = pdFAIL;
    UBaseType_t uxSavedInterruptStatus;

    configASSERT( pxQueue );

    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        if( pxQueue->uxMessagesWaiting > ( UBaseType_t ) 0 )
        {
            if( pxQueue->uxItemSize != ( UBaseType_t ) 0 )
            {
                ( void ) memcpy( pvBuffer,
                                 ( void * ) pxQueue->u.pcReadFrom,
                                 ( size_t ) pxQueue->uxItemSize );
            }
            xReturn = pdPASS;
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );

    return xReturn;
}
/*-----------------------------------------------------------*/

#if ( configUSE_RECURSIVE_MUTEXES == 1 )

    BaseType_t xQueueTakeMutexRecursive( QueueHandle_t xMutex,
                                         TickType_t xTicksToWait )
    {
        BaseType_t xEntryTimeSet = pdFALSE, xReturn;
        TimeOut_t xTimeOut;
        Queue_t * const pxQueue = ( Queue_t * ) xMutex;

        configASSERT( pxQueue );
        configASSERT( pxQueue->ucQueueTypeInternal ==
                      queueQUEUE_TYPE_RECURSIVE_MUTEX );

        for( ; ; )
        {
            taskENTER_CRITICAL();
            {
                if( pxQueue->xMutexHolder == ( TaskHandle_t ) pxCurrentTCB )
                {
                    pxQueue->u.uxRecursiveCallCount++;
                    xReturn = pdPASS;
                    taskEXIT_CRITICAL();
                    return xReturn;
                }
                else if( pxQueue->uxMessagesWaiting > ( UBaseType_t ) 0 )
                {
                    pxQueue->u.uxRecursiveCallCount = ( UBaseType_t ) 1;
                    pxQueue->xMutexHolder = ( TaskHandle_t ) pxCurrentTCB;
                    pxQueue->uxMessagesWaiting = 0;
                    pxCurrentTCB->uxMutexesHeld++;

                    if( listLIST_IS_EMPTY(
                            &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
                    {
                        if( xTaskRemoveFromEventList(
                                &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                        {
                            queueYIELD_IF_USING_PREEMPTION();
                        }
                    }

                    taskEXIT_CRITICAL();
                    return pdPASS;
                }
                else
                {
                    if( xTicksToWait == ( TickType_t ) 0 )
                    {
                        taskEXIT_CRITICAL();
                        return pdFAIL;
                    }
                    else if( xEntryTimeSet == pdFALSE )
                    {
                        vTaskInternalSetTimeOutState( &xTimeOut );
                        xEntryTimeSet = pdTRUE;
                    }
                }
            }
            taskEXIT_CRITICAL();

            vTaskSuspendAll();
            prvUnlockQueue( pxQueue );

            if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == pdFALSE )
            {
                if( prvIsQueueEmpty( pxQueue ) != pdFALSE )
                {
                    vTaskPlaceOnEventList(
                        &( pxQueue->xTasksWaitingToReceive ), xTicksToWait );
                    prvUnlockQueue( pxQueue );

                    if( xTaskResumeAll() == pdFALSE )
                        portYIELD_WITHIN_API();
                }
                else
                {
                    prvUnlockQueue( pxQueue );
                    ( void ) xTaskResumeAll();
                }
            }
            else
            {
                prvUnlockQueue( pxQueue );
                ( void ) xTaskResumeAll();
                return pdFAIL;
            }
        }
    }

#endif /* configUSE_RECURSIVE_MUTEXES */
/*-----------------------------------------------------------*/

#if ( configUSE_RECURSIVE_MUTEXES == 1 )

    BaseType_t xQueueGiveMutexRecursive( QueueHandle_t xMutex )
    {
        Queue_t * const pxQueue = ( Queue_t * ) xMutex;

        configASSERT( pxQueue );
        configASSERT( pxQueue->ucQueueTypeInternal ==
                      queueQUEUE_TYPE_RECURSIVE_MUTEX );

        if( pxQueue->xMutexHolder != ( TaskHandle_t ) pxCurrentTCB )
        {
            return pdFAIL;
        }

        taskENTER_CRITICAL();
        {
            pxQueue->u.uxRecursiveCallCount--;

            if( pxQueue->u.uxRecursiveCallCount == ( UBaseType_t ) 0 )
            {
                pxQueue->xMutexHolder = NULL;
                pxQueue->uxMessagesWaiting = ( UBaseType_t ) 1;
                pxCurrentTCB->uxMutexesHeld--;

                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                    {
                        queueYIELD_IF_USING_PREEMPTION();
                    }
                }
            }
        }
        taskEXIT_CRITICAL();

        return pdPASS;
    }

#endif /* configUSE_RECURSIVE_MUTEXES */
/*-----------------------------------------------------------*/

void vQueueDelete( QueueHandle_t xQueue )
{
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;
    configASSERT( pxQueue );
    traceQUEUE_DELETE( pxQueue );

    #if ( configQUEUE_REGISTRY_SIZE > 0 )
    {
        vQueueUnregisterQueue( pxQueue );
    }
    #endif

    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
    {
        if( pxQueue->ucStaticallyAllocated == pdFALSE )
        {
            vPortFree( pxQueue );
        }
    }
    #else
    {
        vPortFree( pxQueue );
    }
    #endif
}
/*-----------------------------------------------------------*/

UBaseType_t uxQueueMessagesWaiting( const QueueHandle_t xQueue )
{
    UBaseType_t uxReturn;
    configASSERT( xQueue );
    taskENTER_CRITICAL();
    {
        uxReturn = ( ( Queue_t * ) xQueue )->uxMessagesWaiting;
    }
    taskEXIT_CRITICAL();
    return uxReturn;
}
/*-----------------------------------------------------------*/

UBaseType_t uxQueueSpacesAvailable( const QueueHandle_t xQueue )
{
    UBaseType_t uxReturn;
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;
    configASSERT( pxQueue );
    taskENTER_CRITICAL();
    {
        uxReturn = pxQueue->uxLength - pxQueue->uxMessagesWaiting;
    }
    taskEXIT_CRITICAL();
    return uxReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueIsQueueEmptyFromISR( const QueueHandle_t xQueue )
{
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;
    configASSERT( pxQueue );
    return ( pxQueue->uxMessagesWaiting == ( UBaseType_t ) 0 ) ?
           pdTRUE : pdFALSE;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueIsQueueFullFromISR( const QueueHandle_t xQueue )
{
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;
    configASSERT( pxQueue );
    return ( pxQueue->uxMessagesWaiting == pxQueue->uxLength ) ?
           pdTRUE : pdFALSE;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueReset( QueueHandle_t xQueue )
{
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;

    configASSERT( pxQueue );

    taskENTER_CRITICAL();
    {
        pxQueue->u.pcReadFrom = pxQueue->pcHead +
            ( int8_t )( ( pxQueue->uxLength - ( UBaseType_t ) 1U ) *
                         pxQueue->uxItemSize );
        pxQueue->pcWriteTo    = pxQueue->pcHead;
        pxQueue->uxMessagesWaiting = ( UBaseType_t ) 0U;
        pxQueue->cTxLock      = queueUNLOCKED;
        pxQueue->cRxLock      = queueUNLOCKED;

        while( listLIST_IS_EMPTY(
                &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
        {
            if( xTaskRemoveFromEventList(
                    &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
            {
                queueYIELD_IF_USING_PREEMPTION();
            }
        }
    }
    taskEXIT_CRITICAL();

    return pdPASS;
}
/*-----------------------------------------------------------*/

#if ( configQUEUE_REGISTRY_SIZE > 0 )

    void vQueueAddToRegistry( QueueHandle_t xQueue,
                              const char * pcQueueName )
    {
        UBaseType_t ux;
        for( ux = ( UBaseType_t ) 0U;
             ux < ( UBaseType_t ) configQUEUE_REGISTRY_SIZE;
             ux++ )
        {
            if( xQueueRegistry[ ux ].pcQueueName == NULL )
            {
                xQueueRegistry[ ux ].pcQueueName = pcQueueName;
                xQueueRegistry[ ux ].xHandle     = xQueue;
                traceQUEUE_REGISTRY_ADD( xQueue, pcQueueName );
                break;
            }
        }
    }

    void vQueueUnregisterQueue( QueueHandle_t xQueue )
    {
        UBaseType_t ux;
        for( ux = ( UBaseType_t ) 0U;
             ux < ( UBaseType_t ) configQUEUE_REGISTRY_SIZE;
             ux++ )
        {
            if( xQueueRegistry[ ux ].xHandle == xQueue )
            {
                xQueueRegistry[ ux ].pcQueueName = NULL;
                break;
            }
        }
    }

    const char * pcQueueGetName( QueueHandle_t xQueue )
    {
        UBaseType_t ux;
        const char * pcReturn = NULL;
        for( ux = ( UBaseType_t ) 0U;
             ux < ( UBaseType_t ) configQUEUE_REGISTRY_SIZE;
             ux++ )
        {
            if( xQueueRegistry[ ux ].xHandle == xQueue )
            {
                pcReturn = xQueueRegistry[ ux ].pcQueueName;
                break;
            }
        }
        return pcReturn;
    }

#endif /* configQUEUE_REGISTRY_SIZE */
/*-----------------------------------------------------------*/

/* Queue sets — simplified stubs. */
QueueSetHandle_t xQueueCreateSet( const UBaseType_t uxEventQueueLength )
{
    ( void ) uxEventQueueLength;
    return NULL;
}

BaseType_t xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                           QueueSetHandle_t xQueueSet )
{
    ( void ) xQueueOrSemaphore;
    ( void ) xQueueSet;
    return pdFAIL;
}

BaseType_t xQueueRemoveFromSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                                QueueSetHandle_t xQueueSet )
{
    ( void ) xQueueOrSemaphore;
    ( void ) xQueueSet;
    return pdFAIL;
}

QueueSetMemberHandle_t xQueueSelectFromSet( QueueSetHandle_t xQueueSet,
                                            const TickType_t xTicksToWait )
{
    ( void ) xQueueSet;
    ( void ) xTicksToWait;
    return NULL;
}
/*-----------------------------------------------------------*/
