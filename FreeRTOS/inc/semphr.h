/*
 * FreeRTOS V10.4.6 - Semaphore API (wraps queue primitives)
 */

#ifndef SEMPHR_H
#define SEMPHR_H

typedef QueueHandle_t SemaphoreHandle_t;

/* Binary semaphore */
#define xSemaphoreCreateBinary() \
    xQueueGenericCreate((UBaseType_t)1, 0, 3)

/* Counting semaphore */
#define xSemaphoreCreateCounting(uxMaxCount, uxInitialCount) \
    xQueueGenericCreate((uxMaxCount), 0, 4)

/* Mutex (priority inheritance) */
#define xSemaphoreCreateMutex() \
    xQueueGenericCreate((UBaseType_t)1, 0, 1)

/* Recursive mutex */
#define xSemaphoreCreateRecursiveMutex() \
    xQueueGenericCreate((UBaseType_t)1, 0, 2)

/* Take */
#define xSemaphoreTake(xSemaphore, xBlockTime) \
    xQueueSemaphoreTake((xSemaphore), (xBlockTime))

#define xSemaphoreTakeRecursive(xMutex, xBlockTime) \
    xQueueTakeMutexRecursive((xMutex), (xBlockTime))

/* Give */
#define xSemaphoreGive(xSemaphore) \
    xQueueGenericSend((QueueHandle_t)(xSemaphore), NULL, 0, 0)

#define xSemaphoreGiveRecursive(xMutex) \
    xQueueGiveMutexRecursive((xMutex))

/* ISR versions */
#define xSemaphoreGiveFromISR(xSemaphore, pxHigherPriorityTaskWoken) \
    xQueueGiveFromISR((xSemaphore), (pxHigherPriorityTaskWoken))

#define xSemaphoreTakeFromISR(xSemaphore, pxHigherPriorityTaskWoken) \
    xQueueReceiveFromISR((xSemaphore), NULL, (pxHigherPriorityTaskWoken))

/* Get count */
#define uxSemaphoreGetCount(xSemaphore) \
    uxQueueMessagesWaiting((xSemaphore))

/* Internal functions (declared in queue.c) */
QueueHandle_t xQueueGenericCreate(const UBaseType_t uxQueueLength,
                                  const UBaseType_t uxItemSize,
                                  const uint8_t ucQueueType);
BaseType_t xQueueSemaphoreTake(QueueHandle_t xQueue, TickType_t xTicksToWait);
BaseType_t xQueueGiveFromISR(QueueHandle_t xQueue,
                             BaseType_t *pxHigherPriorityTaskWoken);
BaseType_t xQueueTakeMutexRecursive(QueueHandle_t xMutex,
                                    TickType_t xTicksToWait);
BaseType_t xQueueGiveMutexRecursive(QueueHandle_t xMutex);

#endif /* SEMPHR_H */
