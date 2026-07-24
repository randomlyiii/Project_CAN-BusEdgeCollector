/*
 * FreeRTOS V10.4.6 - Main header
 */

#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>
#include <stddef.h>

/* Check that FreeRTOSConfig.h is included */
#ifndef configCPU_CLOCK_HZ
    #error "FreeRTOSConfig.h must be included before FreeRTOS.h. Add FreeRTOSConfig.h to your include path."
#endif

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
    #define configUSE_TIME_SLICING    1
#endif
#ifndef configUSE_IDLE_HOOK
    #define configUSE_IDLE_HOOK       0
#endif
#ifndef configUSE_TICK_HOOK
    #define configUSE_TICK_HOOK       0
#endif
#ifndef configUSE_MALLOC_FAILED_HOOK
    #define configUSE_MALLOC_FAILED_HOOK   0
#endif

/* Hook function typedefs */
typedef void (*PendedFunction_t)(void *, uint32_t);

/* Required functions */
void vApplicationMallocFailedHook(void);

/* Stack overflow hook */
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
    void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
#endif

/* Assert semantics */
#if defined(__ARMCC_VERSION)
    #define portFORCE_INLINE  __forceinline static
#endif

/* Coverage test macros — no-op by default */
#ifndef mtCOVERAGE_TEST_MARKER
    #define mtCOVERAGE_TEST_MARKER()
#endif
#ifndef mtCOVERAGE_TEST_DELAY
    #define mtCOVERAGE_TEST_DELAY()
#endif

#include "list.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "mpu_wrappers.h"

#endif /* FREERTOS_H */
