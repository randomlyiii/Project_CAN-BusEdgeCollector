/*
 * FreeRTOS V10.4.6 - Software Timer API
 */

#ifndef TIMERS_H
#define TIMERS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void * TimerHandle_t;
typedef void (*TimerCallbackFunction_t)(TimerHandle_t xTimer);

TimerHandle_t xTimerCreate(const char * const pcTimerName,
                           const TickType_t xTimerPeriodInTicks,
                           const UBaseType_t uxAutoReload,
                           void * const pvTimerID,
                           TimerCallbackFunction_t pxCallbackFunction);

#define xTimerStart(xTimer, xTicksToWait) \
    xTimerGenericCommand((xTimer), 1, xTaskGetTickCount(), NULL, (xTicksToWait))
#define xTimerStop(xTimer, xTicksToWait) \
    xTimerGenericCommand((xTimer), 2, 0, NULL, (xTicksToWait))
#define xTimerChangePeriod(xTimer, xNewPeriod, xTicksToWait) \
    xTimerGenericCommand((xTimer), 3, (xNewPeriod), NULL, (xTicksToWait))
#define xTimerDelete(xTimer, xTicksToWait) \
    xTimerGenericCommand((xTimer), 4, 0, NULL, (xTicksToWait))
#define xTimerReset(xTimer, xTicksToWait) \
    xTimerGenericCommand((xTimer), 5, xTaskGetTickCount(), NULL, (xTicksToWait))

#define xTimerStartFromISR(xTimer, pxHigherPriorityTaskWoken) \
    xTimerGenericCommand((xTimer), 1, xTaskGetTickCountFromISR(), \
                         (pxHigherPriorityTaskWoken), 0)
#define xTimerStopFromISR(xTimer, pxHigherPriorityTaskWoken) \
    xTimerGenericCommand((xTimer), 2, 0, (pxHigherPriorityTaskWoken), 0)
#define xTimerResetFromISR(xTimer, pxHigherPriorityTaskWoken) \
    xTimerGenericCommand((xTimer), 5, xTaskGetTickCountFromISR(), \
                         (pxHigherPriorityTaskWoken), 0)
#define xTimerChangePeriodFromISR(xTimer, xNewPeriod, pxHigherPriorityTaskWoken) \
    xTimerGenericCommand((xTimer), 3, (xNewPeriod), (pxHigherPriorityTaskWoken), 0)

/* Internal command message type (must match timers.c layout) */
typedef struct tmrTimerQueueMessage {
    BaseType_t xMessageID;
    union {
        void *xTimerHandle;
        TickType_t xOptionalValue;
        struct {
            void (*pxCallbackFunction)(void *, uint32_t);
            void *pvParameter1;
            uint32_t ulParameter2;
        } xCallbackParameters;
    } u;
} TimerQueueMessage_t;

#define tmrCOMMAND_EXECUTE_CALLBACK_FROM_ISR  ((BaseType_t)-2)
#define tmrCOMMAND_EXECUTE_CALLBACK           ((BaseType_t)-1)
#define tmrCOMMAND_START_DONT_TRACE           ((BaseType_t)0)
#define tmrCOMMAND_START                      ((BaseType_t)1)
#define tmrCOMMAND_START_FROM_ISR             ((BaseType_t)2)
#define tmrCOMMAND_RESET                      ((BaseType_t)3)
#define tmrCOMMAND_RESET_FROM_ISR             ((BaseType_t)4)
#define tmrCOMMAND_STOP                       ((BaseType_t)5)
#define tmrCOMMAND_STOP_FROM_ISR              ((BaseType_t)6)
#define tmrCOMMAND_CHANGE_PERIOD              ((BaseType_t)7)
#define tmrCOMMAND_CHANGE_PERIOD_FROM_ISR     ((BaseType_t)8)
#define tmrCOMMAND_DELETE                     ((BaseType_t)9)
#define tmrFIRST_FROM_ISR_COMMAND             ((BaseType_t)2)
#define tmrNO_DELAY                           ((TickType_t)0)

void *pvTimerGetTimerID(const TimerHandle_t xTimer);
void  vTimerSetTimerID(TimerHandle_t xTimer, void *pvNewID);

BaseType_t xTimerIsTimerActive(TimerHandle_t xTimer);
const char *pcTimerGetName(TimerHandle_t xTimer);
TickType_t  xTimerGetPeriod(TimerHandle_t xTimer);
TickType_t  xTimerGetExpiryTime(TimerHandle_t xTimer);

BaseType_t xTimerPendFunctionCall(void (*pvFunction)(void *, uint32_t),
                                  void *pvParameter1, uint32_t ulParameter2,
                                  TickType_t xTicksToWait);

/* Internal */
BaseType_t xTimerGenericCommand(TimerHandle_t xTimer,
                                const BaseType_t xCommandID,
                                const TickType_t xOptionalValue,
                                BaseType_t * const pxHigherPriorityTaskWoken,
                                const TickType_t xTicksToWait);

/* Called by vTaskStartScheduler() to create the timer service task. */
BaseType_t xTimerCreateTimerTask(void);

#ifdef __cplusplus
}
#endif

#endif /* TIMERS_H */
