/*
 * FreeRTOS V10.4.6 - Main header
 */

#ifndef FREERTOS_H
#define FREERTOS_H

/* Pull in project config — must be before everything else */
#include "FreeRTOSConfig.h"

#include <stdint.h>
#include <stddef.h>

/* Include all public headers */
#include "projdefs.h"
#include "portmacro.h"

/* configASSERT */
#ifndef configASSERT
    #define configASSERT(x)
#endif

/* Defaults for optional config macros not defined in FreeRTOSConfig.h */
#ifndef configUSE_PREEMPTION
    #error "configUSE_PREEMPTION must be defined in FreeRTOSConfig.h"
#endif
#ifndef configUSE_TIME_SLICING
    #define configUSE_TIME_SLICING          1
#endif
#ifndef configUSE_IDLE_HOOK
    #define configUSE_IDLE_HOOK             0
#endif
#ifndef configUSE_TICK_HOOK
    #define configUSE_TICK_HOOK             0
#endif
#ifndef configUSE_MALLOC_FAILED_HOOK
    #define configUSE_MALLOC_FAILED_HOOK    0
#endif
#ifndef configSUPPORT_STATIC_ALLOCATION
    #define configSUPPORT_STATIC_ALLOCATION  0
#endif
#ifndef configSUPPORT_DYNAMIC_ALLOCATION
    #define configSUPPORT_DYNAMIC_ALLOCATION  1
#endif
#ifndef configGENERATE_RUN_TIME_STATS
    #define configGENERATE_RUN_TIME_STATS    0
#endif
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
    #define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#endif
#ifndef configNUM_THREAD_LOCAL_STORAGE_POINTERS
    #define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0
#endif
#ifndef configUSE_POSIX_ERRNO
    #define configUSE_POSIX_ERRNO            0
#endif
#ifndef configRECORD_STACK_HIGH_ADDRESS
    #define configRECORD_STACK_HIGH_ADDRESS  0
#endif
#ifndef portCRITICAL_NESTING_IN_TCB
    #define portCRITICAL_NESTING_IN_TCB      0
#endif
#ifndef configSTACK_DEPTH_TYPE
    #define configSTACK_DEPTH_TYPE          uint16_t
#endif
#ifndef configIDLE_TASK_NAME
    #define configIDLE_TASK_NAME            "IDLE"
#endif
#ifndef portTICK_TYPE_ENTER_CRITICAL
    #define portTICK_TYPE_ENTER_CRITICAL()  portENTER_CRITICAL()
#endif
#ifndef portTICK_TYPE_EXIT_CRITICAL
    #define portTICK_TYPE_EXIT_CRITICAL()   portEXIT_CRITICAL()
#endif
#ifndef portTICK_TYPE_SET_INTERRUPT_MASK_FROM_ISR
    #define portTICK_TYPE_SET_INTERRUPT_MASK_FROM_ISR()   0
#endif
#ifndef portTICK_TYPE_CLEAR_INTERRUPT_MASK_FROM_ISR
    #define portTICK_TYPE_CLEAR_INTERRUPT_MASK_FROM_ISR(x)  (void)(x)
#endif
#ifndef portASSERT_IF_INTERRUPT_PRIORITY_INVALID
    #define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()
#endif
#ifndef portSOFTWARE_BARRIER
    #define portSOFTWARE_BARRIER()
#endif
#ifndef portMEMORY_BARRIER
    #define portMEMORY_BARRIER()
#endif
#ifndef portCLEAN_UP_TCB
    #define portCLEAN_UP_TCB(pxTCB)         (void)(pxTCB)
#endif
#ifndef portCONFIGURE_TIMER_FOR_RUN_TIME_STATS
    #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#endif
#ifndef configASSERT_DEFINED
    #define configASSERT_DEFINED            0
#endif

/* Privileged data macro — empty for non-MPU ports */
#ifndef PRIVILEGED_DATA
    #define PRIVILEGED_DATA
#endif

/* Idle task priority */
#define tskIDLE_PRIORITY    ( ( UBaseType_t ) 0U )

/* Static + dynamic allocation possibility */
#if ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
    #define tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE  1
#else
    #define tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE  0
#endif

/* Yield if using preemption */
#if ( configUSE_PREEMPTION == 1 )
    #define taskYIELD_IF_USING_PREEMPTION()    portYIELD()
#else
    #define taskYIELD_IF_USING_PREEMPTION()
#endif

/* Port hooks — no-op by default */
#ifndef portSETUP_TCB
    #define portSETUP_TCB(pxTCB)
#endif
#ifndef portPRE_TASK_DELETE_HOOK
    #define portPRE_TASK_DELETE_HOOK(pxTCB, pxYield)
#endif

/* Opaque types for MPU / tag */
typedef void TaskHookFunction_t;
typedef void xMPU_SETTINGS;
typedef void MemoryRegion_t;

/* Return values */
#define errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY  (-1)
#define errQUEUE_FULL                          (0)
#define errQUEUE_EMPTY                         (0)

/* Queue yield macro */
#if ( configUSE_PREEMPTION == 1 )
    #define queueYIELD_IF_USING_PREEMPTION()    portYIELD()
#else
    #define queueYIELD_IF_USING_PREEMPTION()
#endif

/* Kernel version (for trace facility assert) */
#define tskKERNEL_VERSION_NUMBER    ( ( ( uint32_t ) 10 << 24 ) | \
                                      ( ( uint32_t ) 4  << 16 ) | \
                                      ( ( uint32_t ) 6  << 8  ) )
#define tskKERNEL_VERSION_BUILD     0

/* Hook function typedefs */
typedef void (*PendedFunction_t)(void *, uint32_t);

/* Required functions */
void vApplicationMallocFailedHook(void);
void *pvPortMalloc(size_t xSize);
void vPortFree(void *pv);

/* Assert semantics */
#ifndef portFORCE_INLINE
    #if defined(__ARMCC_VERSION)
        #define portFORCE_INLINE  __forceinline static
    #endif
#endif

/* Coverage test macros — no-op by default */
#ifndef mtCOVERAGE_TEST_MARKER
    #define mtCOVERAGE_TEST_MARKER()
#endif
#ifndef mtCOVERAGE_TEST_DELAY
    #define mtCOVERAGE_TEST_DELAY()
#endif

/* Task trace macros */
#ifndef traceTASK_CREATE
    #define traceTASK_CREATE(pxNewTCB)
#endif
#ifndef traceTASK_DELETE
    #define traceTASK_DELETE(pxTCB)
#endif
#ifndef traceTASK_DELAY
    #define traceTASK_DELAY()
#endif
#ifndef traceTASK_DELAY_UNTIL
    #define traceTASK_DELAY_UNTIL(xTimeToWake)
#endif
#ifndef traceTASK_SUSPEND
    #define traceTASK_SUSPEND(pxTCB)
#endif
#ifndef traceTASK_RESUME
    #define traceTASK_RESUME(pxTCB)
#endif
#ifndef traceTASK_RESUME_FROM_ISR
    #define traceTASK_RESUME_FROM_ISR(pxTCB)
#endif
#ifndef traceTASK_INCREMENT_TICK
    #define traceTASK_INCREMENT_TICK(xTickCount)
#endif
#ifndef traceTASK_SWITCHED_IN
    #define traceTASK_SWITCHED_IN()
#endif
#ifndef traceTASK_SWITCHED_OUT
    #define traceTASK_SWITCHED_OUT()
#endif
#ifndef traceTASK_PRIORITY_SET
    #define traceTASK_PRIORITY_SET(pxTCB, uxNewPriority)
#endif
#ifndef traceTASK_PRIORITY_INHERIT
    #define traceTASK_PRIORITY_INHERIT(pxMutexHolderTCB, uxPriority)
#endif
#ifndef traceTASK_PRIORITY_DISINHERIT
    #define traceTASK_PRIORITY_DISINHERIT(pxTCB, uxBasePriority)
#endif
#ifndef traceTASK_START_SCHEDULER
    #define traceTASK_START_SCHEDULER()
#endif
#ifndef traceTIMER_CREATE
    #define traceTIMER_CREATE(pxTimer)
#endif
#ifndef traceTIMER_EXPIRED
    #define traceTIMER_EXPIRED(pxTimer)
#endif
#ifndef traceTIMER_DELETE
    #define traceTIMER_DELETE(pxTimer)
#endif
#ifndef traceTIMER_COMMAND_SEND
    #define traceTIMER_COMMAND_SEND(xTimer, xCmdID, xOptVal, xRet)
#endif
#ifndef traceMOVED_TASK_TO_READY_STATE
    #define traceMOVED_TASK_TO_READY_STATE(pxTCB)
#endif

/* Trace macros — empty by default (application can override). */
#ifndef traceMALLOC
    #define traceMALLOC(pvAddress, uiSize)
#endif
#ifndef traceFREE
    #define traceFREE(pvAddress, uiSize)
#endif
#ifndef traceQUEUE_CREATE
    #define traceQUEUE_CREATE(pxNewQueue)
#endif
#ifndef traceQUEUE_CREATE_FAILED
    #define traceQUEUE_CREATE_FAILED(ucQueueType)
#endif
#ifndef traceQUEUE_SEND
    #define traceQUEUE_SEND(pxQueue)
#endif
#ifndef traceQUEUE_SEND_FAILED
    #define traceQUEUE_SEND_FAILED(pxQueue)
#endif
#ifndef traceQUEUE_RECEIVE
    #define traceQUEUE_RECEIVE(pxQueue)
#endif
#ifndef traceQUEUE_RECEIVE_FAILED
    #define traceQUEUE_RECEIVE_FAILED(pxQueue)
#endif
#ifndef traceQUEUE_PEEK
    #define traceQUEUE_PEEK(pxQueue)
#endif
#ifndef traceQUEUE_SEND_FROM_ISR
    #define traceQUEUE_SEND_FROM_ISR(pxQueue)
#endif
#ifndef traceQUEUE_RECEIVE_FROM_ISR
    #define traceQUEUE_RECEIVE_FROM_ISR(pxQueue)
#endif
#ifndef traceEVENT_GROUP_CREATE
    #define traceEVENT_GROUP_CREATE(pxEventGroup)
#endif
#ifndef traceEVENT_GROUP_CREATE_FAILED
    #define traceEVENT_GROUP_CREATE_FAILED()
#endif
#ifndef traceEVENT_GROUP_SET_BITS
    #define traceEVENT_GROUP_SET_BITS(pxEventGroup, uxBitsToSet)
#endif
#ifndef traceEVENT_GROUP_CLEAR_BITS
    #define traceEVENT_GROUP_CLEAR_BITS(pxEventGroup, uxBitsToClear)
#endif
#ifndef traceEVENT_GROUP_WAIT_BITS_BLOCK
    #define traceEVENT_GROUP_WAIT_BITS_BLOCK(pxEventGroup, uxBitsToWaitFor)
#endif
#ifndef traceEVENT_GROUP_WAIT_BITS_END
    #define traceEVENT_GROUP_WAIT_BITS_END(pxEventGroup, uxBitsToWaitFor, xTimeoutOccurred)
#endif
#ifndef traceEVENT_GROUP_DELETE
    #define traceEVENT_GROUP_DELETE(pxEventGroup)
#endif
#ifndef traceEVENT_GROUP_SYNC_BLOCK
    #define traceEVENT_GROUP_SYNC_BLOCK(pxEventGroup, uxBitsToSet, uxBitsToWaitFor)
#endif
#ifndef traceEVENT_GROUP_SYNC_END
    #define traceEVENT_GROUP_SYNC_END(pxEventGroup, uxBitsToSet, uxBitsToWaitFor, xTimeoutOccurred)
#endif
#ifndef traceBLOCKING_ON_QUEUE_SEND
    #define traceBLOCKING_ON_QUEUE_SEND(pxQueue)
#endif
#ifndef traceBLOCKING_ON_QUEUE_RECEIVE
    #define traceBLOCKING_ON_QUEUE_RECEIVE(pxQueue)
#endif
#ifndef traceQUEUE_REGISTRY_ADD
    #define traceQUEUE_REGISTRY_ADD(pxQueue, pcName)
#endif
#ifndef traceQUEUE_REGISTRY_REMOVE
    #define traceQUEUE_REGISTRY_REMOVE(pxQueue)
#endif
#ifndef traceEVENT_GROUP_SET_BITS_FROM_ISR
    #define traceEVENT_GROUP_SET_BITS_FROM_ISR(pxEventGroup, uxBitsToSet)
#endif
#ifndef traceEVENT_GROUP_CLEAR_BITS_FROM_ISR
    #define traceEVENT_GROUP_CLEAR_BITS_FROM_ISR(pxEventGroup, uxBitsToClear)
#endif
#ifndef traceEVENT_GROUP_SET_BITS_FROM_ISR_FAILED
    #define traceEVENT_GROUP_SET_BITS_FROM_ISR_FAILED(pxEventGroup, uxBitsToSet)
#endif
#ifndef traceQUEUE_RECEIVE_FROM_ISR_FAILED
    #define traceQUEUE_RECEIVE_FROM_ISR_FAILED(pxQueue)
#endif
#ifndef traceQUEUE_SEND_FROM_ISR_FAILED
    #define traceQUEUE_SEND_FROM_ISR_FAILED(pxQueue)
#endif
#ifndef traceQUEUE_PEEK_FAILED
    #define traceQUEUE_PEEK_FAILED(pxQueue)
#endif
#ifndef traceBLOCKING_ON_QUEUE_PEEK
    #define traceBLOCKING_ON_QUEUE_PEEK(pxQueue)
#endif
#ifndef traceQUEUE_DELETE
    #define traceQUEUE_DELETE(pxQueue)
#endif

/* List test macros — no-op by default. */
#ifndef listTEST_LIST_INTEGRITY
    #define listTEST_LIST_INTEGRITY(list)
#endif
#ifndef listTEST_LIST_ITEM_INTEGRITY
    #define listTEST_LIST_ITEM_INTEGRITY(item)
#endif

/* heapMINIMUM_BLOCK_SIZE — minimum heap block byte-alignment for splitting */
#ifndef heapMINIMUM_BLOCK_SIZE
    #define heapMINIMUM_BLOCK_SIZE  ((size_t)(portBYTE_ALIGNMENT * 2))
#endif

#include "list.h"
#include "task.h"

/* Stack overflow hook — must be after task.h (needs TaskHandle_t) */
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
    void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
#endif

#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "mpu_wrappers.h"

#endif /* FREERTOS_H */
