/*
 * FreeRTOS V10.4.6 - Project Definitions
 */

#ifndef PROJDEFS_H
#define PROJDEFS_H

typedef void (*TaskFunction_t)(void *);

#define pdFALSE                ((BaseType_t)0)
#define pdTRUE                 ((BaseType_t)1)
#define pdPASS                 (pdTRUE)
#define pdFAIL                 (pdFALSE)
#define pdMS_TO_TICKS(xTimeInMs)  ((TickType_t)(((TickType_t)(xTimeInMs) * (TickType_t)configTICK_RATE_HZ) / (TickType_t)1000))

#define pdLITTLE_ENDIAN        1

#endif /* PROJDEFS_H */
