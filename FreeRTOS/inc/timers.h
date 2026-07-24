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

#ifdef __cplusplus
}
#endif

#endif /* TIMERS_H */
