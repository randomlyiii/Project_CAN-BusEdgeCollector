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
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/*-----------------------------------------------------------*/

/* Semaphores do not actually store or copy data, so there is one item of
 * queue storage space available in a semaphore. */
#define semSEMAPHORE_QUEUE_ITEM_LENGTH    ( ( uint8_t ) 0U )

/* The queue registry is just a means of kernel aware debuggers locating
 * queue structures. */
#if ( configQUEUE_REGISTRY_SIZE > 0 )
    typedef struct QUEUE_REGISTRY_ITEM
    {
        const char *                pcQueueName;
        QueueHandle_t               xHandle;
    } xQueueRegistryItem;

    static xQueueRegistryItem xQueueRegistry[ configQUEUE_REGISTRY_SIZE ];
#endif

/*-----------------------------------------------------------*/

/*
 * Unlocks a queue locked by a call to prvLockQueue.  pdTRUE is returned if
 * the queue was just unlocked (and the queue is now accessible).
 */
static BaseType_t prvUnlockQueue( Queue_t * const pxQueue );

/*
 * Uses a critical section to determine if there is any data in a queue.
 *
 * @return pdTRUE if the queue contains no items, otherwise pdFALSE.
 */
static BaseType_t prvIsQueueEmpty( const Queue_t * pxQueue );

/*
 * Copies an item into the queue, either at the front of the queue or at
 * the back of the queue.
 */
static BaseType_t prvCopyDataToQueue( Queue_t * const pxQueue,
                                      const void * pvItemToQueue,
                                      const BaseType_t xPosition );

/*
 * Copies an item out of a queue.
 */
static void prvCopyDataFromQueue( Queue_t * const pxQueue,
                                  void * const pvBuffer );

/*-----------------------------------------------------------*/

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )

    QueueHandle_t xQueueGenericCreateStatic( const UBaseType_t uxQueueLength,
                                             const UBaseType_t uxItemSize,
                                             uint8_t * pucQueueStorage,
                                             StaticQueue_t * pxStaticQueue,
                                             const uint8_t ucQueueType )
    {
        Queue_t * pxNewQueue;

        configASSERT( uxQueueLength > ( UBaseType_t ) 0 );

        if( ( pxStaticQueue != NULL ) && ( pucQueueStorage != NULL ) )
        {
            /* The address of a statically allocated queue was passed in, use it. */
            pxNewQueue = ( Queue_t * ) pxStaticQueue;

            #if( configASSERT_DEFINED == 1 )
            {
                /* Sanity check that the size of the structure used to declare a
                 * variable of type StaticQueue_t equals the size of the real
                 * queue structure. */
                volatile size_t xSize = sizeof( StaticQueue_t );
                configASSERT( xSize == sizeof( Queue_t ) );
                ( void ) xSize;
            }
            #endif

            if( pucQueueStorage != NULL )
            {
                /* The address of the statically allocated queue storage was
                 * passed in. */
                pxNewQueue->pcHead = ( int8_t * ) pucQueueStorage;
            }
        }
        else
        {
            /* Static allocation not available. */
            pxNewQueue = NULL;
        }

        if( pxNewQueue != NULL )
        {
            /* Initialise the members of the queue structure. */
            if( uxItemSize == ( UBaseType_t ) 0 )
            {
                /* No RAM was allocated for the queue storage area, but PC head
                 * cannot be set to NULL because NULL is used as a key to say
                 * that a queue is used as a mutex.  Therefore, just set pcHead
                 * to point to the queue itself so that pxQueue->pcHead is not
                 * NULL. */
                pxNewQueue->pcHead = ( int8_t * ) pxNewQueue;
            }
            else
            {
                /* Set the head to the start of the queue storage area. */
                pxNewQueue->pcHead = ( int8_t * ) pucQueueStorage;
            }

            pxNewQueue->uxLength = uxQueueLength;
            pxNewQueue->uxItemSize = uxItemSize;
            ( void ) xQueueGenericReset( pxNewQueue, pdTRUE );

            /* Initialise the queue lists. */
            #if ( configUSE_QUEUE_SETS == 1 )
            {
                pxNewQueue->pxQueueSetContainer = NULL;
            }
            #endif

            pxNewQueue->ucQueueType = ucQueueType;
            pxNewQueue->ucStaticallyAllocated = pdTRUE;

            traceQUEUE_CREATE( pxNewQueue );

            return pxNewQueue;
        }
        else
        {
            traceQUEUE_CREATE_FAILED( ucQueueType );
            return NULL;
        }
    }

#endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

QueueHandle_t xQueueGenericCreate( const UBaseType_t uxQueueLength,
                                   const UBaseType_t uxItemSize,
                                   const uint8_t ucQueueType )
{
    Queue_t * pxNewQueue;
    size_t xQueueSizeInBytes;

    configASSERT( uxQueueLength > ( UBaseType_t ) 0 );

    /* Allocate the queue and storage area.  Justification for MISRA
     * deviation as follows:  pvPortMalloc() always ensures returned memory
     * blocks are aligned per the requirements of the MCU stack.  In this case
     * pvPortMalloc() must return a pointer that is guaranteed to meet the
     * alignment requirements of a Queue_t - which in this case is an int8_t *
     * and a void *.  Therefore, whenever the stack alignment requirements are
     * greater than or equal to the pointer type then the memory block will be
     * suitable. */
    if( uxItemSize == ( UBaseType_t ) 0 )
    {
        /* There is no queue storage area. */
        xQueueSizeInBytes = sizeof( Queue_t );
    }
    else
    {
        /* Allocate enough space to hold the maximum number of items that
         * can be in the queue at any time. */
        xQueueSizeInBytes = sizeof( Queue_t ) +
            ( ( size_t ) uxQueueLength * ( size_t ) uxItemSize );
    }

    pxNewQueue = ( Queue_t * ) pvPortMalloc( xQueueSizeInBytes );

    if( pxNewQueue != NULL )
    {
        /* Initialise the queue members.  Jump past the queue structure to
         * find the location of the queue storage area. */
        if( uxItemSize == ( UBaseType_t ) 0 )
        {
            /* No data is being stored. */
            pxNewQueue->pcHead = ( int8_t * ) pxNewQueue;
        }
        else
        {
            /* The queue storage area follows the queue structure. */
            pxNewQueue->pcHead = ( int8_t * ) pxNewQueue +
                sizeof( Queue_t );
        }

        pxNewQueue->uxLength = uxQueueLength;
        pxNewQueue->uxItemSize = uxItemSize;
        ( void ) xQueueGenericReset( pxNewQueue, pdTRUE );

        /* Initialise the queue lists. */
        #if ( configUSE_QUEUE_SETS == 1 )
        {
            pxNewQueue->pxQueueSetContainer = NULL;
        }
        #endif

        pxNewQueue->ucQueueType = ucQueueType;
        #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        {
            pxNewQueue->ucStaticallyAllocated = pdFALSE;
        }
        #endif

        traceQUEUE_CREATE( pxNewQueue );

        return pxNewQueue;
    }
    else
    {
        traceQUEUE_CREATE_FAILED( ucQueueType );
        return NULL;
    }
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericReset( QueueHandle_t xQueue, BaseType_t xNewQueue )
{
    Queue_t * const pxQueue = xQueue;

    configASSERT( pxQueue );

    taskENTER_CRITICAL();
    {
        /* Initialise the storage area and the pointers. */
        pxQueue->u.pcReadFrom = pxQueue->pcHead +
            ( ( pxQueue->uxLength - 1U ) * pxQueue->uxItemSize );
        pxQueue->u.pcWriteTo = pxQueue->pcHead;
        pxQueue->uxMessagesWaiting = ( UBaseType_t ) 0U;
        pxQueue->cRxLock = ( int8_t ) queueUNLOCKED;
        pxQueue->cTxLock = ( int8_t ) queueUNLOCKED;

        if( xNewQueue == pdFALSE )
        {
            /* If there are tasks blocked waiting to read from the queue, then
             * the tasks will remain blocked as after this function exits the
             * queue will still be empty.  If there are tasks blocked waiting
             * to write to the queue, then one should be unblocked as after
             * this function exits it will be possible to write to it. */
            if( listLIST_IS_EMPTY( &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
            {
                if( xTaskRemoveFromEventList(
                        &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                {
                    queueYIELD_IF_USING_PREEMPTION();
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            /* Ensure the event queues start in the correct state. */
            vListInitialise( &( pxQueue->xTasksWaitingToSend ) );
            vListInitialise( &( pxQueue->xTasksWaitingToReceive ) );
        }
    }
    taskEXIT_CRITICAL();

    /* A value is returned for calling semantic purposes, but the value is not
     * expected to ever be used. */
    return pdPASS;
}
/*-----------------------------------------------------------*/

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )

    QueueHandle_t xQueueGenericCreateStatic( const UBaseType_t uxQueueLength,
                                             const UBaseType_t uxItemSize,
                                             uint8_t * pucQueueStorage,
                                             StaticQueue_t * pxStaticQueue,
                                             const uint8_t ucQueueType );

#endif

static BaseType_t prvIsQueueEmpty( const Queue_t * pxQueue )
{
    BaseType_t xReturn;

    taskENTER_CRITICAL();
    {
        if( pxQueue->uxMessagesWaiting == ( UBaseType_t ) 0 )
        {
            xReturn = pdTRUE;
        }
        else
        {
            xReturn = pdFALSE;
        }
    }
    taskEXIT_CRITICAL();

    return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericSend( QueueHandle_t xQueue,
                              const void * const pvItemToQueue,
                              TickType_t xTicksToWait,
                              const BaseType_t xCopyPosition )
{
    BaseType_t xEntryTimeSet = pdFALSE, xYieldRequired;
    TimeOut_t xTimeOut;
    Queue_t * const pxQueue = xQueue;

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

    /* This function relaxes the coding standard somewhat to allow return
     * statements within the function body.  This is done in the interest
     * of execution time efficiency. */
    for( ; ; )
    {
        taskENTER_CRITICAL();
        {
            /* Is there room on the queue?  If running ISR then it must be
             * ok to write. */
            if( ( pxQueue->uxMessagesWaiting < pxQueue->uxLength ) ||
                ( xCopyPosition == queueOVERWRITE ) )
            {
                traceQUEUE_SEND( pxQueue );

                #if ( configUSE_QUEUE_SETS == 1 )
                {
                    UBaseType_t uxPreviousMessagesWaiting =
                        pxQueue->uxMessagesWaiting;
                }
                #endif

                xYieldRequired = prvCopyDataToQueue( pxQueue, pvItemToQueue,
                    xCopyPosition );

                #if ( configUSE_QUEUE_SETS == 1 )
                {
                    if( pxQueue->pxQueueSetContainer != NULL )
                    {
                        if( ( xCopyPosition == queueOVERWRITE ) &&
                            ( uxPreviousMessagesWaiting != ( UBaseType_t ) 0 ) )
                        {
                            /* Do nothing. */
                        }
                        else if( prvNotifyQueueSetContainer( pxQueue,
                            xCopyPosition ) != pdFALSE )
                        {
                            /* The queue is a member of a queue set, and
                             * posting to the queue set caused a higher
                             * priority task to unblock. */
                            queueYIELD_IF_USING_PREEMPTION();
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                    else
                    {
                        /* If there was a task waiting for data to arrive on the
                         * queue then unblock it now. */
                        if( listLIST_IS_EMPTY(
                                &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                        {
                            if( xTaskRemoveFromEventList(
                                    &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                            {
                                /* The unblocked task has a priority higher than
                                 * our own so yield immediately.  Yes it is ok to
                                 * do this from within the critical section. */
                                queueYIELD_IF_USING_PREEMPTION();
                            }
                            else
                            {
                                mtCOVERAGE_TEST_MARKER();
                            }
                        }
                        else if( xYieldRequired != pdFALSE )
                        {
                            /* This path is a special case that will only get
                             * executed if the task was holding multiple mutexes
                             * and the mutexes were given back in an order that
                             * is different to that in which they were taken. */
                            queueYIELD_IF_USING_PREEMPTION();
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                }
                #else /* configUSE_QUEUE_SETS */
                {
                    /* If there was a task waiting for data to arrive on the
                     * queue then unblock it now. */
                    if( listLIST_IS_EMPTY(
                            &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                    {
                        if( xTaskRemoveFromEventList(
                                &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                        {
                            queueYIELD_IF_USING_PREEMPTION();
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                    else if( xYieldRequired != pdFALSE )
                    {
                        queueYIELD_IF_USING_PREEMPTION();
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }
                #endif /* configUSE_QUEUE_SETS */

                taskEXIT_CRITICAL();
                return pdPASS;
            }
            else
            {
                if( xTicksToWait == ( TickType_t ) 0 )
                {
                    /* The queue was full and no block time is specified
                     * (or the block time has expired) so leave now. */
                    taskEXIT_CRITICAL();

                    /* Return to the original privilege level before exiting
                     * the function. */
                    traceQUEUE_SEND_FAILED( pxQueue );
                    return errQUEUE_FULL;
                }
                else if( xEntryTimeSet == pdFALSE )
                {
                    /* The queue was full and a block time was specified so
                     * configure the timeout structure. */
                    vTaskInternalSetTimeOutState( &xTimeOut );
                    xEntryTimeSet = pdTRUE;
                }
                else
                {
                    /* Entry time was already set. */
                    mtCOVERAGE_TEST_MARKER();
                }
            }
        }
        taskEXIT_CRITICAL();

        /* Interrupts and other tasks can send to and receive from the queue
         * now the critical section has been exited. */

        vTaskSuspendAll();
        prvLockQueue( pxQueue );

        /* Update the timeout state to see if it has expired yet. */
        if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == pdFALSE )
        {
            if( prvIsQueueFull( pxQueue ) != pdFALSE )
            {
                traceBLOCKING_ON_QUEUE_SEND( pxQueue );
                vTaskPlaceOnEventList(
                    &( pxQueue->xTasksWaitingToSend ), xTicksToWait );

                /* Unlocking the queue means queue events can affect the
                 * event list.  It is possible that interrupts occurring now
                 * remove this task from the event list again. */
                prvUnlockQueue( pxQueue );

                /* Resuming the scheduler will move the task from the pending
                 * state to the blocked state if it was not unblocked by an
                 * interrupt while the queue was unlocked.  If it was unblocked
                 * then it will be in the pending ready state, so yielding is
                 * necessary if the task that was unblocked has a priority
                 * above the currently running task. */
                if( xTaskResumeAll() == pdFALSE )
                {
                    portYIELD_WITHIN_API();
                }
            }
            else
            {
                /* Try again. */
                prvUnlockQueue( pxQueue );
                ( void ) xTaskResumeAll();
            }
        }
        else
        {
            /* The timeout has expired. */
            prvUnlockQueue( pxQueue );
            ( void ) xTaskResumeAll();

            traceQUEUE_SEND_FAILED( pxQueue );
            return errQUEUE_FULL;
        }
    } /* for( ; ; ) */
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericSendFromISR( QueueHandle_t xQueue,
                                     const void * const pvItemToQueue,
                                     BaseType_t * const pxHigherPriorityTaskWoken,
                                     const BaseType_t xCopyPosition )
{
    BaseType_t xReturn;
    UBaseType_t uxSavedInterruptStatus;
    Queue_t * const pxQueue = xQueue;

    configASSERT( pxQueue );
    configASSERT( !( ( pvItemToQueue == NULL ) &&
        ( pxQueue->uxItemSize != ( UBaseType_t ) 0U ) ) );
    configASSERT( !( ( xCopyPosition == queueOVERWRITE ) &&
        ( pxQueue->uxLength != 1 ) ) );

    /* RTOS ports that support interrupt nesting have the concept of a maximum
     * system call (or maximum API call) interrupt priority.  Interrupts that
     * are above the maximum system call priority are kept permanently enabled,
     * even when the RTOS kernel is in a critical section, but cannot make any
     * calls to FreeRTOS API functions.  If configASSERT() is defined in
     * FreeRTOSConfig.h then portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will
     * result in an assertion failure if a FreeRTOS API function is called from
     * an interrupt that has been assigned a priority above the configured
     * maximum system call interrupt priority.  FreeRTOS maintains a separate
     * interrupt safe API to ensure interrupt entry is as fast and as simple
     * as possible. */
    portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

    /* Similar to xQueueGenericSend, except we don't block if there is no room
     * in the queue.  Also we don't directly wake a task that was blocked on
     * queue empty — we simply return pdTRUE to indicate that a context
     * switch is required. */
    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        if( ( pxQueue->uxMessagesWaiting < pxQueue->uxLength ) ||
            ( xCopyPosition == queueOVERWRITE ) )
        {
            const int8_t cTxLock = pxQueue->cTxLock;
            const UBaseType_t uxPreviousMessagesWaiting = pxQueue->uxMessagesWaiting;

            traceQUEUE_SEND_FROM_ISR( pxQueue );

            /* Semaphores use xQueueGiveFromISR(), so pxQueue will not be a
             * semaphore or mutex.  That means prvCopyDataToQueue() cannot
             * result in a task disinheriting a priority and
             * prvCopyDataToQueue() can be called even though the queue is
             * locked. */
            ( void ) prvCopyDataToQueue( pxQueue, pvItemToQueue,
                xCopyPosition );

            /* The event list is not altered if the queue is locked.  This will
             * be done when the queue is unlocked later. */
            if( cTxLock == queueUNLOCKED )
            {
                #if ( configUSE_QUEUE_SETS == 1 )
                {
                    if( pxQueue->pxQueueSetContainer != NULL )
                    {
                        if( ( xCopyPosition == queueOVERWRITE ) &&
                            ( uxPreviousMessagesWaiting != ( UBaseType_t ) 0 ) )
                        {
                            /* Do nothing. */
                        }
                        else if( prvNotifyQueueSetContainer( pxQueue,
                            xCopyPosition ) != pdFALSE )
                        {
                            /* The queue is a member of a queue set, and
                             * posting to the queue set caused a higher
                             * priority task to unblock. */
                            if( pxHigherPriorityTaskWoken != NULL )
                            {
                                *pxHigherPriorityTaskWoken = pdTRUE;
                            }
                            else
                            {
                                mtCOVERAGE_TEST_MARKER();
                            }
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                    else
                    {
                        if( listLIST_IS_EMPTY(
                                &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                        {
                            if( xTaskRemoveFromEventList(
                                    &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                            {
                                /* The task waiting has a higher priority so record
                                 * that a context switch is required. */
                                if( pxHigherPriorityTaskWoken != NULL )
                                {
                                    *pxHigherPriorityTaskWoken = pdTRUE;
                                }
                                else
                                {
                                    mtCOVERAGE_TEST_MARKER();
                                }
                            }
                            else
                            {
                                mtCOVERAGE_TEST_MARKER();
                            }
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                }
                #else /* configUSE_QUEUE_SETS */
                {
                    if( listLIST_IS_EMPTY(
                            &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                    {
                        if( xTaskRemoveFromEventList(
                                &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                        {
                            if( pxHigherPriorityTaskWoken != NULL )
                            {
                                *pxHigherPriorityTaskWoken = pdTRUE;
                            }
                            else
                            {
                                mtCOVERAGE_TEST_MARKER();
                            }
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }
                #endif /* configUSE_QUEUE_SETS */
            }
            else
            {
                /* Increment the lock count so the task that unlocks the queue
                 * knows that data was posted while it was locked. */
                configASSERT( cTxLock != queueINT8_MAX );
                pxQueue->cTxLock = ( int8_t ) ( cTxLock + 1 );
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

BaseType_t xQueueGiveFromISR( QueueHandle_t xQueue,
                              BaseType_t * const pxHigherPriorityTaskWoken )
{
    BaseType_t xReturn;
    UBaseType_t uxSavedInterruptStatus;
    Queue_t * const pxQueue = xQueue;

    /* Similar to xQueueGenericSendFromISR() but used with semaphores where the
     * item size is 0.  Don't directly wake a task that was blocked on queue
     * empty — we simply return pdTRUE to indicate that a context switch is
     * required. */

    configASSERT( pxQueue );

    /* xQueueGenericSendFromISR() should be used instead of xQueueGiveFromISR()
     * if the item size is not 0. */
    configASSERT( pxQueue->uxItemSize == 0 );

    /* If the queue is locked we do not alter the event list.  This will be done
     * when the queue is unlocked later. */
    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        if( pxQueue->uxMessagesWaiting < pxQueue->uxLength )
        {
            const int8_t cTxLock = pxQueue->cTxLock;

            traceQUEUE_SEND_FROM_ISR( pxQueue );

            /* A task can only have an inherited priority if it is a mutex
             * holder - and if there is a mutex holder then the mutex cannot be
             * given from an ISR.  Therefore, the disinheritance function does
             * not need to be called. */
            ( void ) prvCopyDataToQueue( pxQueue, NULL, queueSEND_TO_BACK );

            /* The event list is not altered if the queue is locked. */
            if( cTxLock == queueUNLOCKED )
            {
                #if ( configUSE_QUEUE_SETS == 1 )
                {
                    if( pxQueue->pxQueueSetContainer != NULL )
                    {
                        if( prvNotifyQueueSetContainer( pxQueue,
                            queueSEND_TO_BACK ) != pdFALSE )
                        {
                            if( pxHigherPriorityTaskWoken != NULL )
                            {
                                *pxHigherPriorityTaskWoken = pdTRUE;
                            }
                            else
                            {
                                mtCOVERAGE_TEST_MARKER();
                            }
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                    else
                    {
                        if( listLIST_IS_EMPTY(
                                &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                        {
                            if( xTaskRemoveFromEventList(
                                    &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                            {
                                if( pxHigherPriorityTaskWoken != NULL )
                                {
                                    *pxHigherPriorityTaskWoken = pdTRUE;
                                }
                                else
                                {
                                    mtCOVERAGE_TEST_MARKER();
                                }
                            }
                            else
                            {
                                mtCOVERAGE_TEST_MARKER();
                            }
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                }
                #else
                {
                    if( listLIST_IS_EMPTY(
                            &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                    {
                        if( xTaskRemoveFromEventList(
                                &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                        {
                            if( pxHigherPriorityTaskWoken != NULL )
                            {
                                *pxHigherPriorityTaskWoken = pdTRUE;
                            }
                            else
                            {
                                mtCOVERAGE_TEST_MARKER();
                            }
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }
                #endif
            }
            else
            {
                configASSERT( cTxLock != queueINT8_MAX );
                pxQueue->cTxLock = ( int8_t ) ( cTxLock + 1 );
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

BaseType_t xQueueReceive( QueueHandle_t xQueue,
                          void * const pvBuffer,
                          TickType_t xTicksToWait )
{
    BaseType_t xEntryTimeSet = pdFALSE;
    TimeOut_t xTimeOut;
    Queue_t * const pxQueue = xQueue;

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

            /* Is there data in the queue now?  To be running the calling task
             * must be the highest priority task wanting to access the queue. */
            if( uxMessagesWaiting > ( UBaseType_t ) 0 )
            {
                /* Data available, remove one item. */
                prvCopyDataFromQueue( pxQueue, pvBuffer );
                traceQUEUE_RECEIVE( pxQueue );
                pxQueue->uxMessagesWaiting = uxMessagesWaiting - ( UBaseType_t ) 1;

                /* There is now space in the queue, were any tasks waiting to
                 * post to the queue?  If so, unblock the highest priority
                 * waiting task. */
                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                    {
                        queueYIELD_IF_USING_PREEMPTION();
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

                taskEXIT_CRITICAL();
                return pdPASS;
            }
            else
            {
                if( xTicksToWait == ( TickType_t ) 0 )
                {
                    /* The queue was empty and no block time is specified
                     * (or the block time has expired) so leave now. */
                    taskEXIT_CRITICAL();
                    traceQUEUE_RECEIVE_FAILED( pxQueue );
                    return errQUEUE_EMPTY;
                }
                else if( xEntryTimeSet == pdFALSE )
                {
                    /* The queue was empty and a block time was specified so
                     * configure the timeout structure. */
                    vTaskInternalSetTimeOutState( &xTimeOut );
                    xEntryTimeSet = pdTRUE;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
        }
        taskEXIT_CRITICAL();

        /* Interrupts and other tasks can send to and receive from the queue
         * now the critical section has been exited. */

        vTaskSuspendAll();
        prvLockQueue( pxQueue );

        /* Update the timeout state to see if it has expired yet. */
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
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                /* The queue was not empty, so try again. */
                prvUnlockQueue( pxQueue );
                ( void ) xTaskResumeAll();
            }
        }
        else
        {
            /* Timed out. */
            prvUnlockQueue( pxQueue );
            ( void ) xTaskResumeAll();

            if( prvIsQueueEmpty( pxQueue ) != pdFALSE )
            {
                traceQUEUE_RECEIVE_FAILED( pxQueue );
                return errQUEUE_EMPTY;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
    } /* for( ; ; ) */
}
/*-----------------------------------------------------------*/

BaseType_t xQueueSemaphoreTake( QueueHandle_t xQueue,
                                TickType_t xTicksToWait )
{
    BaseType_t xEntryTimeSet = pdFALSE;
    TimeOut_t xTimeOut;
    Queue_t * const pxQueue = xQueue;

    /* This function should not be used with queues. */
    configASSERT( pxQueue->uxItemSize == 0 );
    configASSERT( pxQueue != NULL );

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

                /* There is now space in the queue, were any tasks waiting to
                 * post to the queue? */
                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                    {
                        queueYIELD_IF_USING_PREEMPTION();
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
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
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
        }
        taskEXIT_CRITICAL();

        vTaskSuspendAll();
        prvLockQueue( pxQueue );

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
                       void * const pvBuffer,
                       TickType_t xTicksToWait )
{
    BaseType_t xEntryTimeSet = pdFALSE;
    TimeOut_t xTimeOut;
    int8_t * pcOriginalReadPosition;
    Queue_t * const pxQueue = xQueue;

    configASSERT( pxQueue );
    configASSERT( !( ( pvBuffer == NULL ) &&
        ( pxQueue->uxItemSize != ( UBaseType_t ) 0U ) ) );

    for( ; ; )
    {
        taskENTER_CRITICAL();
        {
            if( pxQueue->uxMessagesWaiting > ( UBaseType_t ) 0 )
            {
                /* Remember the read position so it can be reset after the
                 * data is read from the queue as this function is only peeking
                 * at the data, not removing it. */
                pcOriginalReadPosition = pxQueue->u.pcReadFrom;

                prvCopyDataFromQueue( pxQueue, pvBuffer );
                traceQUEUE_PEEK( pxQueue );

                /* The data is not being removed, so reset the read pointer. */
                pxQueue->u.pcReadFrom = pcOriginalReadPosition;

                /* The data is being left in the queue, so see if there are
                 * any other tasks waiting for the data. */
                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                    {
                        queueYIELD_IF_USING_PREEMPTION();
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

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
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
        }
        taskEXIT_CRITICAL();

        vTaskSuspendAll();
        prvLockQueue( pxQueue );

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

BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue,
                                 void * const pvBuffer,
                                 BaseType_t * const pxHigherPriorityTaskWoken )
{
    BaseType_t xReturn;
    UBaseType_t uxSavedInterruptStatus;
    Queue_t * const pxQueue = xQueue;

    configASSERT( pxQueue );
    configASSERT( !( ( pvBuffer == NULL ) &&
        ( pxQueue->uxItemSize != ( UBaseType_t ) 0U ) ) );

    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;

        /* Is there data in the queue now? */
        if( uxMessagesWaiting > ( UBaseType_t ) 0 )
        {
            const int8_t cRxLock = pxQueue->cRxLock;

            traceQUEUE_RECEIVE_FROM_ISR( pxQueue );

            prvCopyDataFromQueue( pxQueue, pvBuffer );
            pxQueue->uxMessagesWaiting = uxMessagesWaiting - ( UBaseType_t ) 1;

            /* If the queue is locked we do not alter the event list. */
            if( cRxLock == queueUNLOCKED )
            {
                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
                {
                    if( xTaskRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                    {
                        if( pxHigherPriorityTaskWoken != NULL )
                        {
                            *pxHigherPriorityTaskWoken = pdTRUE;
                        }
                        else
                        {
                            mtCOVERAGE_TEST_MARKER();
                        }
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                configASSERT( cRxLock != queueINT8_MAX );
                pxQueue->cRxLock = ( int8_t ) ( cRxLock + 1 );
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
    Queue_t * const pxQueue = xQueue;

    configASSERT( pxQueue );

    taskENTER_CRITICAL();
    {
        uxReturn = pxQueue->uxLength - pxQueue->uxMessagesWaiting;
    }
    taskEXIT_CRITICAL();

    return uxReturn;
}
/*-----------------------------------------------------------*/

UBaseType_t uxQueueMessagesWaitingFromISR( const QueueHandle_t xQueue )
{
    UBaseType_t uxReturn;
    Queue_t * const pxQueue = xQueue;

    configASSERT( pxQueue );
    uxReturn = pxQueue->uxMessagesWaiting;

    return uxReturn;
}
/*-----------------------------------------------------------*/

void vQueueDelete( QueueHandle_t xQueue )
{
    Queue_t * const pxQueue = xQueue;

    configASSERT( pxQueue );
    traceQUEUE_DELETE( pxQueue );

    #if ( configQUEUE_REGISTRY_SIZE > 0 )
    {
        vQueueUnregisterQueue( pxQueue );
    }
    #endif

    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
    {
        if( pxQueue->ucStaticallyAllocated == ( uint8_t ) pdFALSE )
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

#if ( configQUEUE_REGISTRY_SIZE > 0 )

    void vQueueAddToRegistry( QueueHandle_t xQueue, const char * pcQueueName )
    {
        UBaseType_t ux;

        /* See if there is an empty space in the registry.  A NULL name denotes
         * a free slot. */
        for( ux = ( UBaseType_t ) 0U; ux < ( UBaseType_t ) configQUEUE_REGISTRY_SIZE; ux++ )
        {
            if( xQueueRegistry[ ux ].pcQueueName == NULL )
            {
                /* Store the information on this queue. */
                xQueueRegistry[ ux ].pcQueueName = pcQueueName;
                xQueueRegistry[ ux ].xHandle = xQueue;

                traceQUEUE_REGISTRY_ADD( xQueue, pcQueueName );
                break;
            }
        }
    }

#endif
/*-----------------------------------------------------------*/

#if ( configQUEUE_REGISTRY_SIZE > 0 )

    const char * pcQueueGetName( QueueHandle_t xQueue )
    {
        UBaseType_t ux;
        const char * pcReturn = NULL;

        for( ux = ( UBaseType_t ) 0U; ux < ( UBaseType_t ) configQUEUE_REGISTRY_SIZE; ux++ )
        {
            if( xQueueRegistry[ ux ].xHandle == xQueue )
            {
                pcReturn = xQueueRegistry[ ux ].pcQueueName;
                break;
            }
        }

        return pcReturn;
    }

#endif
/*-----------------------------------------------------------*/

#if ( configQUEUE_REGISTRY_SIZE > 0 )

    void vQueueUnregisterQueue( QueueHandle_t xQueue )
    {
        UBaseType_t ux;

        for( ux = ( UBaseType_t ) 0U; ux < ( UBaseType_t ) configQUEUE_REGISTRY_SIZE; ux++ )
        {
            if( xQueueRegistry[ ux ].xHandle == xQueue )
            {
                xQueueRegistry[ ux ].pcQueueName = NULL;
                break;
            }
        }
    }

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )

    QueueSetHandle_t xQueueCreateSet( const UBaseType_t uxEventQueueLength )
    {
        QueueSetHandle_t pxQueue;

        pxQueue = xQueueGenericCreate( uxEventQueueLength,
            ( UBaseType_t ) sizeof( Queue_t * ), queueQUEUE_TYPE_SET );

        return pxQueue;
    }

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )

    BaseType_t prvNotifyQueueSetContainer( const Queue_t * const pxQueue,
                                           const BaseType_t xCopyPosition );

    BaseType_t xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                               QueueSetHandle_t xQueueSet )
    {
        BaseType_t xReturn;

        taskENTER_CRITICAL();
        {
            if( ( ( Queue_t * ) xQueueOrSemaphore )->pxQueueSetContainer != NULL )
            {
                xReturn = pdFAIL;
            }
            else
            {
                ( ( Queue_t * ) xQueueOrSemaphore )->pxQueueSetContainer = xQueueSet;
                xReturn = pdPASS;
            }
        }
        taskEXIT_CRITICAL();

        return xReturn;
    }

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )

    BaseType_t xQueueRemoveFromSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                                    QueueSetHandle_t xQueueSet )
    {
        BaseType_t xReturn;
        Queue_t * const pxQueueOrSemaphore = ( Queue_t * ) xQueueOrSemaphore;

        ( void ) xQueueSet;

        if( pxQueueOrSemaphore->pxQueueSetContainer != NULL )
        {
            if( pxQueueOrSemaphore->pxQueueSetContainer == xQueueSet )
            {
                pxQueueOrSemaphore->pxQueueSetContainer = NULL;
                xReturn = pdPASS;
            }
            else
            {
                xReturn = pdFAIL;
            }
        }
        else
        {
            xReturn = pdFAIL;
        }

        return xReturn;
    }

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )

    QueueSetMemberHandle_t xQueueSelectFromSet( QueueSetHandle_t xQueueSet,
                                                TickType_t const xTicksToWait )
    {
        QueueSetMemberHandle_t xReturn = NULL;

        ( void ) xQueueReceive( ( QueueHandle_t ) xQueueSet,
            &xReturn, xTicksToWait );

        return xReturn;
    }

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )

    QueueSetMemberHandle_t xQueueSelectFromSetFromISR(
        QueueSetHandle_t xQueueSet )
    {
        QueueSetMemberHandle_t xReturn = NULL;

        ( void ) xQueueReceiveFromISR( ( QueueHandle_t ) xQueueSet,
            &xReturn, NULL );

        return xReturn;
    }

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

static BaseType_t prvCopyDataToQueue( Queue_t * const pxQueue,
                                      const void * pvItemToQueue,
                                      const BaseType_t xPosition )
{
    BaseType_t xReturn = pdFALSE;
    UBaseType_t uxMessagesWaiting;

    /* This function is called from a critical section. */

    uxMessagesWaiting = pxQueue->uxMessagesWaiting;

    if( pxQueue->uxItemSize == ( UBaseType_t ) 0 )
    {
        /* This is a mutex or semaphore. */
        #if ( configUSE_MUTEXES == 1 )
        {
            if( pxQueue->uxQueueType == queueQUEUE_IS_MUTEX )
            {
                /* The mutex is no longer being held. */
                xReturn = xTaskPriorityDisinherit(
                    pxQueue->u.pxMutexHolder );
                pxQueue->u.pxMutexHolder = NULL;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        #endif
    }
    else if( xPosition == queueSEND_TO_BACK )
    {
        ( void ) memcpy( ( void * ) pxQueue->pcWriteTo, pvItemToQueue,
            ( size_t ) pxQueue->uxItemSize );
        pxQueue->pcWriteTo += pxQueue->uxItemSize;

        if( pxQueue->pcWriteTo >= pxQueue->pcHead +
            ( pxQueue->uxLength * pxQueue->uxItemSize ) )
        {
            pxQueue->pcWriteTo = pxQueue->pcHead;
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    else if( xPosition == queueSEND_TO_FRONT )
    {
        ( void ) memcpy( ( void * ) pxQueue->u.pcReadFrom,
            pvItemToQueue, ( size_t ) pxQueue->uxItemSize );
        pxQueue->u.pcReadFrom -= pxQueue->uxItemSize;

        if( pxQueue->u.pcReadFrom < pxQueue->pcHead )
        {
            pxQueue->u.pcReadFrom =
                ( pxQueue->pcHead +
                  ( ( pxQueue->uxLength - ( UBaseType_t ) 1U ) *
                    pxQueue->uxItemSize ) );
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    else
    {
        /* xPosition == queueOVERWRITE */
        configASSERT( pxQueue->uxLength == 1 );

        /* Overwrite an existing item. */
        ( void ) memcpy( ( void * ) pxQueue->pcHead, pvItemToQueue,
            ( size_t ) pxQueue->uxItemSize );
        pxQueue->u.pcReadFrom = pxQueue->pcHead;
        pxQueue->pcWriteTo = pxQueue->pcHead + pxQueue->uxItemSize;
    }

    pxQueue->uxMessagesWaiting = uxMessagesWaiting + ( UBaseType_t ) 1;

    return xReturn;
}
/*-----------------------------------------------------------*/

static void prvCopyDataFromQueue( Queue_t * const pxQueue,
                                  void * const pvBuffer )
{
    if( pxQueue->uxItemSize != ( UBaseType_t ) 0 )
    {
        pxQueue->u.pcReadFrom += pxQueue->uxItemSize;

        if( pxQueue->u.pcReadFrom >= pxQueue->pcHead +
            ( pxQueue->uxLength * pxQueue->uxItemSize ) )
        {
            pxQueue->u.pcReadFrom = pxQueue->pcHead;
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        ( void ) memcpy( ( void * ) pvBuffer,
            ( void * ) pxQueue->u.pcReadFrom,
            ( size_t ) pxQueue->uxItemSize );
    }
}
/*-----------------------------------------------------------*/

static void prvLockQueue( Queue_t * const pxQueue )
{
    /* This function must only be called within a critical section. */

    /* The cTxLock and cRxLock counts track the number of times data has been
     * posted or received while the queue was locked.  When the queue is
     * unlocked, the count values are used to re-enable the event lists. */
    taskENTER_CRITICAL();
    {
        /* If cTxLock overflows then the calling task is using the queue
         * recursively (queue locking is not designed to be used recursively). */
        configASSERT( pxQueue->cTxLock != queueINT8_MAX );
        configASSERT( pxQueue->cRxLock != queueINT8_MAX );

        /* Data can only be posted to the queue if the queue is not locked.
         * The queue is locked by setting cTxLock to queueLOCKED_UNMODIFIED. */
        if( pxQueue->cTxLock == queueUNLOCKED )
        {
            pxQueue->cTxLock = queueLOCKED_UNMODIFIED;
        }

        if( pxQueue->cRxLock == queueUNLOCKED )
        {
            pxQueue->cRxLock = queueLOCKED_UNMODIFIED;
        }
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

static BaseType_t prvUnlockQueue( Queue_t * const pxQueue )
{
    BaseType_t xReturn = pdFALSE;

    /* This function must only be called within a critical section. */

    taskENTER_CRITICAL();
    {
        /* Is there a task waiting to receive data?  If so, and if the cRxLock
         * count is not 0, unblock it now. */
        if( pxQueue->cRxLock != queueUNLOCKED )
        {
            int8_t cRxLock = pxQueue->cRxLock;

            /* If cRxLock is queueLOCKED_UNMODIFIED then the lock was taken
             * but no data was received while it was locked.  The event list
             * does not need updating. */
            if( cRxLock > queueLOCKED_UNMODIFIED )
            {
                /* Data was posted while the queue was locked. */
                BaseType_t xYieldRequired = pdFALSE;
                TaskHandle_t xTaskToNotify = NULL;

                while( cRxLock > queueLOCKED_UNMODIFIED )
                {
                    #if ( configUSE_QUEUE_SETS == 1 )
                    {
                        if( pxQueue->pxQueueSetContainer != NULL )
                        {
                            if( prvNotifyQueueSetContainer( pxQueue,
                                queueSEND_TO_BACK ) != pdFALSE )
                            {
                                /* The queue is a member of a queue set, and
                                 * posting to the queue set caused a higher
                                 * priority task to unblock. */
                                xYieldRequired = pdTRUE;
                            }
                        }
                    }
                    #else
                    {
                        if( listLIST_IS_EMPTY(
                                &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                        {
                            /* Remove the task from the event list. */
                            xTaskToNotify = xTaskRemoveFromEventList(
                                &( pxQueue->xTasksWaitingToReceive ) );

                            if( xTaskToNotify != NULL )
                            {
                                /* The task removed from the event list has a
                                 * higher priority than the calling task. */
                                xYieldRequired = pdTRUE;
                            }
                        }
                    }
                    #endif

                    cRxLock--;
                }

                pxQueue->cRxLock = queueUNLOCKED;

                if( xYieldRequired == pdTRUE )
                {
                    /* At least one task was removed from the event list.
                     * Indicate that a context switch is required. */
                    xReturn = pdTRUE;
                }
            }
            else
            {
                /* The lock was taken, but no data was received.  Just restore
                 * the queue to the unlocked state. */
                pxQueue->cRxLock = queueUNLOCKED;
            }
        }

        /* Is there a task waiting to send data?  If so, and if the cTxLock
         * count is not 0, unblock it now. */
        if( pxQueue->cTxLock != queueUNLOCKED )
        {
            int8_t cTxLock = pxQueue->cTxLock;

            if( cTxLock > queueLOCKED_UNMODIFIED )
            {
                while( cTxLock > queueLOCKED_UNMODIFIED )
                {
                    if( listLIST_IS_EMPTY(
                            &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
                    {
                        if( xTaskRemoveFromEventList(
                                &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                        {
                            /* The task removed from the event list has a
                             * higher priority than the calling task. */
                            xReturn = pdTRUE;
                        }
                    }
                    else
                    {
                        /* No tasks are waiting to send. */
                        break;
                    }

                    cTxLock--;
                }

                pxQueue->cTxLock = queueUNLOCKED;
            }
            else
            {
                pxQueue->cTxLock = queueUNLOCKED;
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    taskEXIT_CRITICAL();

    return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueIsQueueEmptyFromISR( const QueueHandle_t xQueue )
{
    BaseType_t xReturn;
    Queue_t * const pxQueue = xQueue;

    configASSERT( pxQueue );

    if( pxQueue->uxMessagesWaiting == ( UBaseType_t ) 0 )
    {
        xReturn = pdTRUE;
    }
    else
    {
        xReturn = pdFALSE;
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueIsQueueFullFromISR( const QueueHandle_t xQueue )
{
    BaseType_t xReturn;
    Queue_t * const pxQueue = xQueue;

    configASSERT( pxQueue );

    if( pxQueue->uxMessagesWaiting == pxQueue->uxLength )
    {
        xReturn = pdTRUE;
    }
    else
    {
        xReturn = pdFALSE;
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

#if ( configUSE_CO_ROUTINES == 1 )

    BaseType_t xQueueCRSendFromISR( QueueHandle_t xQueue,
                                    const void * pvItemToQueue,
                                    BaseType_t xCoRoutinePreviouslyWoken )
    {
        Queue_t * const pxQueue = xQueue;

        /* Cannot block within ISR even for co-routines. */
        if( pxQueue->uxMessagesWaiting < pxQueue->uxLength )
        {
            prvCopyDataToQueue( pxQueue, pvItemToQueue, queueSEND_TO_BACK );

            if( pxQueue->cTxLock == queueUNLOCKED )
            {
                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
                {
                    if( xCoRoutineRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                    {
                        return pdTRUE;
                    }
                }
            }
        }

        return xCoRoutinePreviouslyWoken;
    }

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_CO_ROUTINES == 1 )

    BaseType_t xQueueCRReceiveFromISR( QueueHandle_t xQueue,
                                       void * pvBuffer,
                                       BaseType_t * pxTaskWoken )
    {
        BaseType_t xReturn;
        Queue_t * const pxQueue = xQueue;

        if( pxQueue->uxMessagesWaiting > ( UBaseType_t ) 0 )
        {
            prvCopyDataFromQueue( pxQueue, pvBuffer );
            pxQueue->uxMessagesWaiting--;

            if( pxQueue->cRxLock == queueUNLOCKED )
            {
                if( listLIST_IS_EMPTY(
                        &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
                {
                    if( xCoRoutineRemoveFromEventList(
                            &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                    {
                        if( pxTaskWoken != NULL )
                        {
                            *pxTaskWoken = pdTRUE;
                        }
                    }
                }
            }

            xReturn = pdPASS;
        }
        else
        {
            xReturn = pdFAIL;
        }

        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_CO_ROUTINES == 1 )

    BaseType_t xQueueCRSend( QueueHandle_t xQueue,
                             const void * pvItemToQueue,
                             TickType_t xTicksToWait )
    {
        BaseType_t xReturn = pdFAIL;
        Queue_t * const pxQueue = xQueue;

        /* If the queue is already full we may have to block. */
        if( pxQueue->uxMessagesWaiting < pxQueue->uxLength )
        {
            prvCopyDataToQueue( pxQueue, pvItemToQueue, queueSEND_TO_BACK );

            if( listLIST_IS_EMPTY(
                    &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
            {
                if( xCoRoutineRemoveFromEventList(
                        &( pxQueue->xTasksWaitingToReceive ) ) != pdFALSE )
                {
                    /* The task waiting has a higher priority. */
                    xReturn = pdPASS;
                }
            }
        }
        else
        {
            xReturn = errQUEUE_FULL;
        }

        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_CO_ROUTINES == 1 )

    BaseType_t xQueueCRReceive( QueueHandle_t xQueue,
                                void * pvBuffer,
                                TickType_t xTicksToWait )
    {
        BaseType_t xReturn;
        Queue_t * const pxQueue = xQueue;

        if( pxQueue->uxMessagesWaiting > ( UBaseType_t ) 0 )
        {
            prvCopyDataFromQueue( pxQueue, pvBuffer );
            pxQueue->uxMessagesWaiting--;

            if( listLIST_IS_EMPTY(
                    &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
            {
                if( xCoRoutineRemoveFromEventList(
                        &( pxQueue->xTasksWaitingToSend ) ) != pdFALSE )
                {
                    xReturn = pdPASS;
                }
            }
        }
        else
        {
            xReturn = pdFAIL;
        }

        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )

    QueueHandle_t xQueueGenericCreateStatic( const UBaseType_t uxQueueLength,
                                             const UBaseType_t uxItemSize,
                                             uint8_t * pucQueueStorage,
                                             StaticQueue_t * pxStaticQueue,
                                             const uint8_t ucQueueType );

    BaseType_t xQueueGetStaticBuffers( QueueHandle_t xQueue,
                                       uint8_t ** ppucQueueStorage,
                                       StaticQueue_t ** ppxStaticQueue )
    {
        BaseType_t xReturn = pdFAIL;
        Queue_t * const pxQueue = xQueue;

        #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        {
            if( pxQueue->ucStaticallyAllocated == pdTRUE )
            {
                if( ppucQueueStorage != NULL )
                {
                    *ppucQueueStorage = ( uint8_t * ) pxQueue->pcHead;
                }

                if( ppxStaticQueue != NULL )
                {
                    *ppxStaticQueue = ( StaticQueue_t * ) pxQueue;
                }

                xReturn = pdTRUE;
            }
        }
        #else
        {
            ( void ) ppucQueueStorage;
            ( void ) ppxStaticQueue;
        }
        #endif

        ( void ) pxQueue;
        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/
