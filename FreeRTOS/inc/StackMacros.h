/*
 * FreeRTOS V10.4.6 - Stack overflow detection macros
 */

#ifndef STACK_MACROS_H
#define STACK_MACROS_H

/*
 * Method 1: check pxCurrentTCB->pxTopOfStack on every context switch.
 * Method 2: fill stack with known pattern (0xA5) on task create,
 *           then check on task delete or explicit check.
 */
#if (configCHECK_FOR_STACK_OVERFLOW == 1)
    extern void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
    #define taskCHECK_FOR_STACK_OVERFLOW()                                    \
        {                                                                     \
            extern volatile uint32_t *pxCurrentTCB_pxTopOfStack;             \
            if (*pxCurrentTCB_pxTopOfStack <= (uint32_t)0x00000010)          \
                vApplicationStackOverflowHook(NULL, NULL);                    \
        }
#elif (configCHECK_FOR_STACK_OVERFLOW == 2)
    #define taskCHECK_FOR_STACK_OVERFLOW()                                    \
        {                                                                     \
            const uint32_t * const pxStack = pxCurrentTCB->pxStack;          \
            const uint32_t ulStackLimit = (uint32_t)pxCurrentTCB->pxStack +  \
                                          (uint32_t)20;                       \
            if (*pxStack != 0xA5A5A5A5UL ||                                  \
                (uint32_t)pxCurrentTCB->pxTopOfStack <= ulStackLimit)         \
                vApplicationStackOverflowHook((TaskHandle_t)pxCurrentTCB,    \
                                              pxCurrentTCB->pcTaskName);      \
        }
#endif

/* Fill stack with known value at task creation */
#define taskSTACK_FILL_BYTE    (0xA5U)

#endif /* STACK_MACROS_H */
