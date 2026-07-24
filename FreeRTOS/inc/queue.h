/*
 * FreeRTOS V10.4.6 - Queue API
 */

#ifndef QUEUE_H
#define QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void * QueueHandle_t;
typedef void * QueueSetHandle_t;
typedef void * QueueSetMemberHandle_t;

/* Queue creation */
QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize);
QueueHandle_t xQueueCreateStatic(UBaseType_t uxQueueLength,
                                 UBaseType_t uxItemSize,
                                 uint8_t *pucQueueStorage,
                                 void *pxQueueBuffer);

/* Send to queue */
BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue,
                      TickType_t xTicksToWait);
BaseType_t xQueueSendToBack(QueueHandle_t xQueue, const void *pvItemToQueue,
                            TickType_t xTicksToWait);
BaseType_t xQueueSendToFront(QueueHandle_t xQueue, const void *pvItemToQueue,
                             TickType_t xTicksToWait);
BaseType_t xQueueSendFromISR(QueueHandle_t xQueue, const void *pvItemToQueue,
                             BaseType_t *pxHigherPriorityTaskWoken);
BaseType_t xQueueOverwrite(QueueHandle_t xQueue, const void *pvItemToQueue);

/* Generic send (used by semaphore macros) */
BaseType_t xQueueGenericSend(QueueHandle_t xQueue, const void * const pvItemToQueue,
                             TickType_t xTicksToWait, const BaseType_t xCopyPosition);

/* Receive from queue */
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer,
                         TickType_t xTicksToWait);
BaseType_t xQueuePeek(QueueHandle_t xQueue, void *pvBuffer,
                      TickType_t xTicksToWait);
BaseType_t xQueueReceiveFromISR(QueueHandle_t xQueue, void *pvBuffer,
                                BaseType_t *pxHigherPriorityTaskWoken);
BaseType_t xQueuePeekFromISR(QueueHandle_t xQueue, void *pvBuffer);

/* Queue management */
UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue);
UBaseType_t uxQueueSpacesAvailable(const QueueHandle_t xQueue);
void        vQueueDelete(QueueHandle_t xQueue);
BaseType_t  xQueueIsQueueEmptyFromISR(const QueueHandle_t xQueue);
BaseType_t  xQueueIsQueueFullFromISR(const QueueHandle_t xQueue);
BaseType_t  xQueueReset(QueueHandle_t xQueue);

/* Queue registry (for debug) */
void        vQueueAddToRegistry(QueueHandle_t xQueue, const char *pcQueueName);
void        vQueueUnregisterQueue(QueueHandle_t xQueue);
const char *pcQueueGetName(QueueHandle_t xQueue);

/* Queue sets */
QueueSetHandle_t         xQueueCreateSet(const UBaseType_t uxEventQueueLength);
BaseType_t               xQueueAddToSet(QueueSetMemberHandle_t xQueueOrSemaphore,
                                        QueueSetHandle_t xQueueSet);
BaseType_t               xQueueRemoveFromSet(QueueSetMemberHandle_t xQueueOrSemaphore,
                                             QueueSetHandle_t xQueueSet);
QueueSetMemberHandle_t   xQueueSelectFromSet(QueueSetHandle_t xQueueSet,
                                             const TickType_t xTicksToWait);

#ifdef __cplusplus
}
#endif

#endif /* QUEUE_H */
