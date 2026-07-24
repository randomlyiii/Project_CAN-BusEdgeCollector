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

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "list.h"

/*-----------------------------------------------------------*/

/* The following bit fields convey control information in a task's event list
 * item value.  It is important they do not clash with the task notification
 * state. */
#if configUSE_16_BIT_TICKS == 1
    #define eventEVENT_BITS_CONTROL_BYTES    ( ( uint8_t ) 0x00U )
#else
    #define eventEVENT_BITS_CONTROL_BYTES    ( ( uint8_t ) 0x00U )
#endif

typedef struct EventGroupDef_t
{
    EventBits_t     uxEventBits;
    List_t          xTasksWaitingForBits;

    #if( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t uxEventGroupNumber;
    #endif

    #if( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
        uint8_t     ucStaticallyAllocated;
    #endif
} EventGroup_t;

/*-----------------------------------------------------------*/

static BaseType_t prvTestWaitCondition( const EventBits_t uxCurrentEventBits,
                                        const EventBits_t uxBitsToWaitFor,
                                        const BaseType_t xWaitForAllBits )
{
    BaseType_t xWaitConditionMet = pdFALSE;

    if( xWaitForAllBits == pdFALSE )
    {
        /* Task only has to be woken if any of the bits it is waiting on
         * are set. */
        if( ( uxCurrentEventBits & uxBitsToWaitFor ) != ( EventBits_t ) 0 )
        {
            xWaitConditionMet = pdTRUE;
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    else
    {
        /* Task only has to be woken if all the bits it is waiting on
         * are set. */
        if( ( uxCurrentEventBits & uxBitsToWaitFor ) == uxBitsToWaitFor )
        {
            xWaitConditionMet = pdTRUE;
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }

    return xWaitConditionMet;
}
/*-----------------------------------------------------------*/

EventGroupHandle_t xEventGroupCreate( void )
{
    EventGroup_t * pxEventBits;

    pxEventBits = ( EventGroup_t * ) pvPortMalloc( sizeof( EventGroup_t ) );

    if( pxEventBits != NULL )
    {
        pxEventBits->uxEventBits = 0;
        vListInitialise( &( pxEventBits->xTasksWaitingForBits ) );

        #if( configSUPPORT_STATIC_ALLOCATION == 1 )
        {
            pxEventBits->ucStaticallyAllocated = pdFALSE;
        }
        #endif

        traceEVENT_GROUP_CREATE( pxEventBits );
    }
    else
    {
        traceEVENT_GROUP_CREATE_FAILED();
    }

    return pxEventBits;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupSync( EventGroupHandle_t xEventGroup,
                             const EventBits_t uxBitsToSet,
                             const EventBits_t uxBitsToWaitFor,
                             TickType_t xTicksToWait )
{
    EventBits_t uxOriginalBitValue, uxReturn;
    EventGroup_t * pxEventBits = xEventGroup;
    BaseType_t xAlreadyYielded;
    BaseType_t xTimeoutOccurred = pdFALSE;

    configASSERT( ( uxBitsToWaitFor & eventEVENT_BITS_CONTROL_BYTES ) == 0 );
    configASSERT( uxBitsToWaitFor != 0 );

    /* Only in the following code, with wait for bit pattern, then set bit
     * pattern is allowed. */
    #if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
    {
        configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) &&
                         ( xTicksToWait != 0 ) ) );
    }
    #endif

    vTaskSuspendAll();
    {
        uxOriginalBitValue = pxEventBits->uxEventBits;

        ( void ) xEventGroupSetBits( xEventGroup, uxBitsToSet );

        if( ( ( uxOriginalBitValue | uxBitsToSet ) &
              uxBitsToWaitFor ) == uxBitsToWaitFor )
        {
            /* All the rendezvous bits are now set - no need to block. */
            uxReturn = ( uxOriginalBitValue | uxBitsToSet );

            /* Rendezvous always clear the bits.  They will have been cleared
             * already unless this is the only task in the rendezvous. */
            pxEventBits->uxEventBits &= ~uxBitsToWaitFor;

            xTicksToWait = 0;
        }
        else
        {
            if( xTicksToWait != ( TickType_t ) 0 )
            {
                traceEVENT_GROUP_SYNC_BLOCK( xEventGroup, uxBitsToSet, uxBitsToWaitFor );

                /* Store the bits that the calling task is waiting for in the
                 * task's event list item so the kernel knows when a match is
                 * found.  Then enter the blocked state. */
                vTaskPlaceOnUnorderedEventList(
                    &( pxEventBits->xTasksWaitingForBits ),
                    ( uxBitsToWaitFor | eventCLEAR_EVENTS_ON_EXIT_BIT |
                      eventWAIT_FOR_ALL_BITS | eventSYNC_POINT_BIT ),
                    xTicksToWait );

                /* This assignment is obsolete as uxReturn will get set after
                 * the task unblocks, but some compilers mistakenly generate a
                 * warning about uxReturn being returned without being set
                 * if it is not done. */
                uxReturn = 0;
            }
            else
            {
                /* The rendezvous bits were not set, but no block time was
                 * specified - just return the current event bit value. */
                uxReturn = pxEventBits->uxEventBits;
                xTimeoutOccurred = pdTRUE;
            }
        }
    }
    xAlreadyYielded = xTaskResumeAll();

    if( xTicksToWait != ( TickType_t ) 0 )
    {
        if( xAlreadyYielded == pdFALSE )
        {
            portYIELD_WITHIN_API();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* The task blocked to wait for all the rendezvous bits to be set.
         * Return the event bit value when the task finally unblocks. */
        uxReturn = uxTaskResetEventItemValue();

        if( ( uxReturn & eventUNBLOCKED_DUE_TO_BIT_SET ) ==
            ( EventBits_t ) 0 )
        {
            /* The task timed out, just return the current event bit value. */
            taskENTER_CRITICAL();
            {
                uxReturn = pxEventBits->uxEventBits;

                /* Although the task got here because it timed out before the
                 * bits it was waiting for were set, it is possible that since
                 * it unblocked another task has set the bits.  If this is the
                 * case then it needs to clear the bits before exiting. */
                if( ( uxReturn & uxBitsToWaitFor ) == uxBitsToWaitFor )
                {
                    pxEventBits->uxEventBits &= ~uxBitsToWaitFor;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            taskEXIT_CRITICAL();

            xTimeoutOccurred = pdTRUE;
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* Control bits might be set as the task had blocked should not be
         * returned. */
        uxReturn &= ~eventEVENT_BITS_CONTROL_BYTES;
    }

    traceEVENT_GROUP_SYNC_END( xEventGroup, uxBitsToSet, uxBitsToWaitFor, xTimeoutOccurred );

    /* Prevent compiler warnings when trace macros are not used. */
    ( void ) xTimeoutOccurred;

    return uxReturn;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupWaitBits( EventGroupHandle_t xEventGroup,
                                 const EventBits_t uxBitsToWaitFor,
                                 const BaseType_t xClearOnExit,
                                 const BaseType_t xWaitForAllBits,
                                 TickType_t xTicksToWait )
{
    EventGroup_t * pxEventBits = xEventGroup;
    EventBits_t uxReturn, uxControlBits = 0;
    BaseType_t xWaitConditionMet, xAlreadyYielded;
    BaseType_t xTimeoutOccurred = pdFALSE;

    /* Check the user is not attempting to wait on the bits used by the kernel
     * itself, and that at least one bit is being requested. */
    configASSERT( xEventGroup );
    configASSERT( ( uxBitsToWaitFor & eventEVENT_BITS_CONTROL_BYTES ) == 0 );
    configASSERT( uxBitsToWaitFor != 0 );

    #if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
    {
        configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) &&
                         ( xTicksToWait != 0 ) ) );
    }
    #endif

    vTaskSuspendAll();
    {
        const EventBits_t uxCurrentEventBits = pxEventBits->uxEventBits;

        /* Check to see if the wait condition is already met or not. */
        xWaitConditionMet = prvTestWaitCondition( uxCurrentEventBits,
                                                  uxBitsToWaitFor,
                                                  xWaitForAllBits );

        if( xWaitConditionMet != pdFALSE )
        {
            /* The wait condition has already been met so there is no need to
             * block. */
            uxReturn = uxCurrentEventBits;
            xTicksToWait = ( TickType_t ) 0;

            /* Clear the wait bits if requested to do so. */
            if( xClearOnExit != pdFALSE )
            {
                pxEventBits->uxEventBits &= ~uxBitsToWaitFor;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else if( xTicksToWait == ( TickType_t ) 0 )
        {
            /* The wait condition has not been met, but no block time was
             * specified, so just return the current value. */
            uxReturn = uxCurrentEventBits;
            xTimeoutOccurred = pdTRUE;
        }
        else
        {
            /* The task is going to block to wait for the bits to be set.
             * The uxControlBits parameter is used to pass information into the
             * task's event list item. */
            if( xClearOnExit != pdFALSE )
            {
                uxControlBits |= eventCLEAR_EVENTS_ON_EXIT_BIT;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }

            if( xWaitForAllBits != pdFALSE )
            {
                uxControlBits |= eventWAIT_FOR_ALL_BITS;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }

            /* Store the bits that the calling task is waiting for in the
             * task's event list item so the kernel knows when a match is
             * found.  Then enter the blocked state. */
            vTaskPlaceOnUnorderedEventList(
                &( pxEventBits->xTasksWaitingForBits ),
                ( uxBitsToWaitFor | uxControlBits ),
                xTicksToWait );

            uxReturn = 0;
            traceEVENT_GROUP_WAIT_BITS_BLOCK( xEventGroup, uxBitsToWaitFor );
        }
    }
    xAlreadyYielded = xTaskResumeAll();

    if( xTicksToWait != ( TickType_t ) 0 )
    {
        if( xAlreadyYielded == pdFALSE )
        {
            portYIELD_WITHIN_API();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* The task blocked to wait for the required bits to be set.  uxReturn
         * now holds the event item value.  If the task unblocked because the
         * bits were set, uxReturn will have the bits that caused the task to
         * unblock ANDed with the control bits.  If it timed out then uxReturn
         * will still be zero. */
        uxReturn = uxTaskResetEventItemValue();

        if( ( uxReturn & eventUNBLOCKED_DUE_TO_BIT_SET ) ==
            ( EventBits_t ) 0 )
        {
            taskENTER_CRITICAL();
            {
                /* The task timed out, just return the current event bit value. */
                uxReturn = pxEventBits->uxEventBits;

                /* It is possible that the event bits were updated between this
                 * task leaving the Blocked state and running this function. */
                if( prvTestWaitCondition( uxReturn, uxBitsToWaitFor,
                                          xWaitForAllBits ) != pdFALSE )
                {
                    if( xClearOnExit != pdFALSE )
                    {
                        pxEventBits->uxEventBits &= ~uxBitsToWaitFor;
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

                xTimeoutOccurred = pdTRUE;
            }
            taskEXIT_CRITICAL();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* The task blocked so control bits may have been set. */
        uxReturn &= ~eventEVENT_BITS_CONTROL_BYTES;
    }

    traceEVENT_GROUP_WAIT_BITS_END( xEventGroup, uxBitsToWaitFor, xTimeoutOccurred );

    /* Prevent compiler warnings when trace macros are not used. */
    ( void ) xTimeoutOccurred;

    return uxReturn;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupClearBits( EventGroupHandle_t xEventGroup,
                                  const EventBits_t uxBitsToClear )
{
    EventGroup_t * pxEventBits = xEventGroup;
    EventBits_t uxReturn;

    /* Check the user is not attempting to clear the bits used by the kernel
     * itself. */
    configASSERT( xEventGroup );
    configASSERT( ( uxBitsToClear & eventEVENT_BITS_CONTROL_BYTES ) == 0 );

    taskENTER_CRITICAL();
    {
        traceEVENT_GROUP_CLEAR_BITS( xEventGroup, uxBitsToClear );

        /* The value returned is the event group value prior to the bits being
         * cleared. */
        uxReturn = pxEventBits->uxEventBits;

        /* Clear the bits. */
        pxEventBits->uxEventBits &= ~uxBitsToClear;
    }
    taskEXIT_CRITICAL();

    return uxReturn;
}
/*-----------------------------------------------------------*/

#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( INCLUDE_xTimerPendFunctionCall == 1 ) && \
      ( configUSE_TIMERS == 1 ) )

    BaseType_t xEventGroupClearBitsFromISR( EventGroupHandle_t xEventGroup,
                                            const EventBits_t uxBitsToClear )
    {
        BaseType_t xReturn;

        traceEVENT_GROUP_CLEAR_BITS_FROM_ISR( xEventGroup, uxBitsToClear );
        xReturn = xTimerPendFunctionCallFromISR(
            vEventGroupClearBitsCallback,
            ( void * ) xEventGroup,
            ( uint32_t ) uxBitsToClear,
            NULL );

        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/

EventBits_t xEventGroupGetBitsFromISR( EventGroupHandle_t xEventGroup )
{
    UBaseType_t uxSavedInterruptStatus;
    EventGroup_t const * const pxEventBits = xEventGroup;
    EventBits_t uxReturn;

    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        uxReturn = pxEventBits->uxEventBits;
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );

    return uxReturn;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupSetBits( EventGroupHandle_t xEventGroup,
                                const EventBits_t uxBitsToSet )
{
    ListItem_t * pxListItem, * pxNext;
    ListItem_t const * pxListEnd;
    List_t const * pxList;
    EventBits_t uxBitsToClear = 0, uxBitsWaitedFor, uxControlBits;
    EventGroup_t * pxEventBits = xEventGroup;
    BaseType_t xMatchFound = pdFALSE;

    /* Check the user is not attempting to set the bits used by the kernel
     * itself. */
    configASSERT( xEventGroup );
    configASSERT( ( uxBitsToSet & eventEVENT_BITS_CONTROL_BYTES ) == 0 );

    pxList = &( pxEventBits->xTasksWaitingForBits );
    pxListEnd = listGET_END_MARKER( pxList );
    vTaskSuspendAll();
    {
        traceEVENT_GROUP_SET_BITS( xEventGroup, uxBitsToSet );

        pxListItem = listGET_HEAD_ENTRY( pxList );

        /* Set the bits. */
        pxEventBits->uxEventBits |= uxBitsToSet;

        /* See if the new bit value should unblock any tasks. */
        while( pxListItem != pxListEnd )
        {
            pxNext = listGET_NEXT( pxListItem );
            uxBitsWaitedFor = listGET_LIST_ITEM_VALUE( pxListItem );
            xMatchFound = pdFALSE;

            /* Split the bits waited for from the control bits. */
            uxControlBits = uxBitsWaitedFor & eventEVENT_BITS_CONTROL_BYTES;
            uxBitsWaitedFor &= ~eventEVENT_BITS_CONTROL_BYTES;

            if( ( uxControlBits & eventWAIT_FOR_ALL_BITS ) ==
                ( EventBits_t ) 0 )
            {
                /* Just looking for single bit being set. */
                if( ( uxBitsWaitedFor & pxEventBits->uxEventBits ) !=
                    ( EventBits_t ) 0 )
                {
                    xMatchFound = pdTRUE;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else if( ( uxBitsWaitedFor & pxEventBits->uxEventBits ) ==
                     uxBitsWaitedFor )
            {
                /* All bits are set. */
                xMatchFound = pdTRUE;
            }
            else
            {
                /* Need all bits to be set, but not all the bits were set. */
            }

            if( xMatchFound != pdFALSE )
            {
                /* The bits match.  Should the bits be cleared on exit? */
                if( ( uxControlBits & eventCLEAR_EVENTS_ON_EXIT_BIT ) !=
                    ( EventBits_t ) 0 )
                {
                    uxBitsToClear |= uxBitsWaitedFor;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

                if( ( uxControlBits & eventSYNC_POINT_BIT ) ==
                    ( EventBits_t ) 0 )
                {
                    /* Store the actual event flag value in the task's event
                     * list item before removing the task from the event list. */
                    vTaskRemoveFromUnorderedEventList( pxListItem,
                        pxEventBits->uxEventBits | eventUNBLOCKED_DUE_TO_BIT_SET );
                }
                else
                {
                    /* The sync point implementation is a rendezvous.  This
                     * also checks if the task is the first (last, or only)
                     * task in the rendezvous. */
                    if( listCURRENT_LIST_LENGTH( pxList ) >
                        ( UBaseType_t ) 1 )
                    {
                        vTaskRemoveFromUnorderedEventList( pxListItem,
                            pxEventBits->uxEventBits | eventUNBLOCKED_DUE_TO_BIT_SET );
                    }
                    else
                    {
                        uxBitsToClear |= uxBitsWaitedFor;
                        vTaskRemoveFromUnorderedEventList( pxListItem,
                            ( EventBits_t ) 0 );
                    }
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }

            pxListItem = pxNext;
        }

        /* Clear bits if any tasks were unblocked. */
        pxEventBits->uxEventBits &= ~uxBitsToClear;
    }
    ( void ) xTaskResumeAll();

    return pxEventBits->uxEventBits;
}
/*-----------------------------------------------------------*/

void vEventGroupDelete( EventGroupHandle_t xEventGroup )
{
    EventGroup_t * pxEventBits = xEventGroup;
    const List_t * pxTasksWaitingForBits = &( pxEventBits->xTasksWaitingForBits );

    vTaskSuspendAll();
    {
        traceEVENT_GROUP_DELETE( xEventGroup );

        while( listCURRENT_LIST_LENGTH( pxTasksWaitingForBits ) >
               ( UBaseType_t ) 0 )
        {
            /* Unblock the task, returning 0 as the event list is being deleted
             * and cannot therefore have any bits set. */
            configASSERT( pxTasksWaitingForBits->xListEnd.pxNext !=
                ( const ListItem_t * ) &( pxTasksWaitingForBits->xListEnd ) );
            vTaskRemoveFromUnorderedEventList(
                pxTasksWaitingForBits->xListEnd.pxNext,
                eventUNBLOCKED_DUE_TO_BIT_SET );
        }

        vPortFree( pxEventBits );

        #if( configSUPPORT_STATIC_ALLOCATION == 1 )
        {
            pxEventBits->ucStaticallyAllocated = pdFALSE;
        }
        #endif
    }
    ( void ) xTaskResumeAll();
}
/*-----------------------------------------------------------*/

/* For backward compatibility. */
void vEventGroupSetBitsCallback( void * pvEventGroup,
                                 const uint32_t ulBitsToSet )
{
    ( void ) xEventGroupSetBits( pvEventGroup, ( EventBits_t ) ulBitsToSet );
}
/*-----------------------------------------------------------*/

void vEventGroupClearBitsCallback( void * pvEventGroup,
                                   const uint32_t ulBitsToClear )
{
    ( void ) xEventGroupClearBits( pvEventGroup, ( EventBits_t ) ulBitsToClear );
}
/*-----------------------------------------------------------*/

#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( INCLUDE_xTimerPendFunctionCall == 1 ) && \
      ( configUSE_TIMERS == 1 ) )

    BaseType_t xEventGroupSetBitsFromISR( EventGroupHandle_t xEventGroup,
                                          const EventBits_t uxBitsToSet,
                                          BaseType_t * pxHigherPriorityTaskWoken )
    {
        BaseType_t xReturn;

        traceEVENT_GROUP_SET_BITS_FROM_ISR( xEventGroup, uxBitsToSet );
        xReturn = xTimerPendFunctionCallFromISR(
            vEventGroupSetBitsCallback,
            ( void * ) xEventGroup,
            ( uint32_t ) uxBitsToSet,
            pxHigherPriorityTaskWoken );

        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

    UBaseType_t uxEventGroupGetNumber( void * xEventGroup )
    {
        UBaseType_t xReturn;
        EventGroup_t const * pxEventBits = ( EventGroup_t * ) xEventGroup;

        if( xEventGroup == NULL )
        {
            xReturn = 0;
        }
        else
        {
            xReturn = pxEventBits->uxEventGroupNumber;
        }

        return xReturn;
    }

#endif
/*-----------------------------------------------------------*/

#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )

    void vEventGroupSetNumber( void * xEventGroup, UBaseType_t uxEventGroupNumber )
    {
        ( ( EventGroup_t * ) xEventGroup )->uxEventGroupNumber = uxEventGroupNumber;
    }

#endif
/*-----------------------------------------------------------*/
