/*
 * Stack overflow detection macros — standard FreeRTOS implementation
 */

#ifndef STACK_MACROS_H
#define STACK_MACROS_H

extern void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

#if (configCHECK_FOR_STACK_OVERFLOW == 1)

    /* Method 1: check if stack pointer went below stack base */
    #define taskCHECK_FOR_STACK_OVERFLOW()                              \
    {                                                                   \
        if (pxCurrentTCB->pxTopOfStack <= pxCurrentTCB->pxStack)      \
        {                                                               \
            vApplicationStackOverflowHook(                              \
                (TaskHandle_t)pxCurrentTCB,                            \
                pxCurrentTCB->pcTaskName);                             \
        }                                                               \
    }

#elif (configCHECK_FOR_STACK_OVERFLOW == 2)

    /* Method 2: check canary (word) at stack base + SP overflow
     * Stack filled byte-wise with 0xa5 → each uint32_t word = 0xa5a5a5a5 */
    #define taskCHECK_FOR_STACK_OVERFLOW()                              \
    {                                                                   \
        if (*(pxCurrentTCB->pxStack) != 0xa5a5a5a5UL                 \
            || pxCurrentTCB->pxTopOfStack <= pxCurrentTCB->pxStack)  \
        {                                                               \
            vApplicationStackOverflowHook(                              \
                (TaskHandle_t)pxCurrentTCB,                            \
                pxCurrentTCB->pcTaskName);                             \
        }                                                               \
    }

#else

    #define taskCHECK_FOR_STACK_OVERFLOW()

#endif

#endif /* STACK_MACROS_H */
