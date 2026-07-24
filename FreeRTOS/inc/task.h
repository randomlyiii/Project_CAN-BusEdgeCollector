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

/* Task control block (opaque handle) */
typedef void * TaskHandle_t;

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
                              uint32_t eAction,
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

#ifdef __cplusplus
}
#endif

#endif /* TASK_H */
