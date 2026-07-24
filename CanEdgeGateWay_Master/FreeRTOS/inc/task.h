/*
 * FreeRTOS V10.4.6 - Task API
 */

#ifndef TASK_H
#define TASK_H

#include "portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Task states */
typedef enum {
    eRunning   = 0,
    eReady     = 1,
    eBlocked   = 2,
    eSuspended = 3,
    eDeleted   = 4,
    eInvalid   = 5
} eTaskState;

/* Scheduler running states */
#define taskSCHEDULER_NOT_STARTED    0
#define taskSCHEDULER_RUNNING        1
#define taskSCHEDULER_SUSPENDED      2

/* Sleep mode status */
#define eAbortSleep                  ((BaseType_t)-1)

/* Forward types used by task API */
typedef void * TaskHandle_t;

#define queueSEND_TO_BACK    ((BaseType_t)0)

/* Forward declaration — full struct defined below under MPU_WRAPPERS_INCLUDED_FROM_API_FILE */
typedef struct tskTaskControlBlock TCB_t;

extern TCB_t * volatile pxCurrentTCB;

/* Notification actions */
typedef enum {
    eNoAction,
    eSetBits,
    eIncrement,
    eSetValueWithOverwrite,
    eSetValueWithoutOverwrite
} eNotifyAction;

/* Internal functions referenced by kernel source */
void vTaskSwitchContext(void);
BaseType_t xTaskIncrementTick(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void vTaskPlaceOnEventList(List_t * const pxEventList, const TickType_t xTicksToWait);
void vTaskPlaceOnUnorderedEventList(List_t *pxEventList,
                                     const TickType_t xItemValue,
                                     const TickType_t xTicksToWait);
BaseType_t xTaskRemoveFromEventList(const List_t * const pxEventList);
void vTaskRemoveFromUnorderedEventList(ListItem_t *pxEventListItem,
                                        const TickType_t xItemValue);
TickType_t uxTaskResetEventItemValue(void);

/* Task status structure */
typedef struct xTASK_STATUS {
    TaskHandle_t   xHandle;
    const char *   pcTaskName;
    UBaseType_t    xTaskNumber;
    eTaskState     eCurrentState;
    UBaseType_t    uxCurrentPriority;
    UBaseType_t    uxBasePriority;
    uint32_t       ulRunTimeCounter;
    StackType_t *  pxStackBase;
    uint32_t       usStackHighWaterMark;
} TaskStatus_t;

/* Task creation */
BaseType_t xTaskCreate(TaskFunction_t pxTaskCode,
                       const char * const pcName,
                       const uint16_t usStackDepth,
                       void * const pvParameters,
                       UBaseType_t uxPriority,
                       TaskHandle_t * const pxCreatedTask);

/* Task control */
void vTaskDelete(TaskHandle_t xTaskToDelete);
void vTaskDelay(const TickType_t xTicksToDelay);
void vTaskDelayUntil(TickType_t * const pxPreviousWakeTime,
                     const TickType_t xTimeIncrement);
void vTaskSuspend(TaskHandle_t xTaskToSuspend);
void vTaskResume(TaskHandle_t xTaskToResume);
BaseType_t xTaskResumeFromISR(TaskHandle_t xTaskToResume);
void vTaskSuspendAll(void);
BaseType_t xTaskResumeAll(void);

/* Priority */
void         vTaskPrioritySet(TaskHandle_t xTask, UBaseType_t uxNewPriority);
UBaseType_t  uxTaskPriorityGet(const TaskHandle_t xTask);
UBaseType_t  uxTaskPriorityGetFromISR(const TaskHandle_t xTask);

/* Scheduler */
void         vTaskStartScheduler(void);
void         vTaskEndScheduler(void);
TickType_t   xTaskGetTickCount(void);
TickType_t   xTaskGetTickCountFromISR(void);
UBaseType_t  uxTaskGetNumberOfTasks(void);
char *       pcTaskGetName(TaskHandle_t xTaskToQuery);
TaskHandle_t xTaskGetIdleTaskHandle(void);
UBaseType_t  uxTaskGetSystemState(TaskStatus_t * const pxTaskStatusArray,
                                  const UBaseType_t uxArraySize,
                                  uint32_t * const pulTotalRunTime);
void         vTaskGetInfo(TaskHandle_t xTask, TaskStatus_t *pxTaskStatus,
                          BaseType_t xGetFreeStackSpace, eTaskState eState);

/* Stack monitoring */
UBaseType_t  uxTaskGetStackHighWaterMark(TaskHandle_t xTask);

/* Critical section — aliased to port layer */
#define taskENTER_CRITICAL()            portENTER_CRITICAL()
#define taskEXIT_CRITICAL()             portEXIT_CRITICAL()
#define taskDISABLE_INTERRUPTS()        portDISABLE_INTERRUPTS()
#define taskENABLE_INTERRUPTS()         portENABLE_INTERRUPTS()
#define taskYIELD()                     portYIELD()

/* Task notifications */
BaseType_t xTaskNotifyWait(uint32_t ulBitsToClearOnEntry,
                           uint32_t ulBitsToClearOnExit,
                           uint32_t *pulNotificationValue,
                           TickType_t xTicksToWait);
BaseType_t xTaskGenericNotify(TaskHandle_t xTaskToNotify,
                              uint32_t ulValue,
                              eNotifyAction eAction,
                              uint32_t *pulPreviousNotificationValue);
#define xTaskNotify(xTaskToNotify, ulValue, eAction) \
    xTaskGenericNotify((xTaskToNotify), (ulValue), (eAction), NULL)
#define xTaskNotifyGive(xTaskToNotify) \
    xTaskGenericNotify((xTaskToNotify), (0), 1, NULL)

/* Utility */
uint32_t ulTaskNotifyTake(BaseType_t xClearCountOnExit,
                          TickType_t xTicksToWait);


BaseType_t xTaskGetSchedulerState(void);
BaseType_t eTaskConfirmSleepModeStatus(void);
void vTaskStepTick(const TickType_t xTicksToJump);

/* Event list helpers (used by event_groups.c) */
void vTaskPlaceOnUnorderedEventList(List_t *pxEventList,
                                    const TickType_t xEventListItemValue,
                                    const TickType_t xTicksToWait);
void vTaskRemoveFromUnorderedEventList(ListItem_t *pxEventListItem,
                                       const TickType_t xItemValue);
TickType_t uxTaskResetEventItemValue(void);

/* Event list helpers — ordered/priority-based (used by queue.c, timers.c) */
void vTaskPlaceOnEventList(List_t * const pxEventList,
                           const TickType_t xTicksToWait);
BaseType_t xTaskRemoveFromEventList(const List_t * const pxEventList);

/* Timeout management (used by queue.c, timers.c) */
typedef struct xTIME_OUT
{
    BaseType_t xOverflowCount;
    TickType_t xTimeOnEntering;
} TimeOut_t;

void vTaskSetTimeOutState(TimeOut_t * const pxTimeOut);
void vTaskInternalSetTimeOutState(TimeOut_t * const pxTimeOut);
BaseType_t xTaskCheckForTimeOut(TimeOut_t * const pxTimeOut,
                                TickType_t * const pxTicksToWait);

void vTaskMissedYield(void);

/*
 * Internal kernel structures — visible only when MPU_WRAPPERS_INCLUDED_FROM_API_FILE
 * is defined before including this header.  Used by tasks.c, queue.c, timers.c.
 */
#if defined( MPU_WRAPPERS_INCLUDED_FROM_API_FILE )

    typedef struct tskTaskControlBlock
    {
        volatile StackType_t * pxTopOfStack;
        #if ( portUSING_MPU_WRAPPERS == 1 )
            xMPU_SETTINGS       xMPUSettings;
        #endif
        ListItem_t              xStateListItem;
        ListItem_t              xEventListItem;
        UBaseType_t             uxPriority;
        StackType_t           * pxStack;
        char                    pcTaskName[ configMAX_TASK_NAME_LEN ];
        configSTACK_DEPTH_TYPE  uxStackDepth;
        #if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
            StackType_t       * pxEndOfStack;
        #endif
        #if ( portCRITICAL_NESTING_IN_TCB == 1 )
            UBaseType_t         uxCriticalNesting;
        #endif
        #if ( configUSE_TRACE_FACILITY == 1 )
            UBaseType_t         uxTCBNumber;
            UBaseType_t         uxTaskNumber;
        #endif
        #if ( configUSE_MUTEXES == 1 )
            UBaseType_t         uxBasePriority;
            UBaseType_t         uxMutexesHeld;
        #endif
        #if ( configUSE_APPLICATION_TASK_TAG == 1 )
            TaskHookFunction_t  pxTaskTag;
        #endif
        #if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
            void *              pvThreadLocalStoragePointers[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
        #endif
        #if ( configGENERATE_RUN_TIME_STATS == 1 )
            uint32_t            ulRunTimeCounter;
        #endif
        #if ( configUSE_TASK_NOTIFICATIONS == 1 )
            volatile uint32_t   ulNotifiedValue[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
            volatile uint8_t    ucNotifyState[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
        #endif
        #if ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
            uint8_t             ucStaticallyAllocated;
        #endif
        #if ( INCLUDE_xTaskAbortDelay == 1 )
            uint8_t             ucDelayAborted;
        #endif
        #if ( configUSE_POSIX_ERRNO == 1 )
            int                 iTaskErrno;
        #endif
    } tskTCB;

    typedef tskTCB TCB_t;

    /* Kernel globals */
    extern TCB_t * volatile pxCurrentTCB;

#endif /* MPU_WRAPPERS_INCLUDED_FROM_API_FILE */

/* Always-visible externs */
extern volatile UBaseType_t uxSchedulerSuspended;

#ifdef __cplusplus
}
#endif

#endif /* TASK_H */
