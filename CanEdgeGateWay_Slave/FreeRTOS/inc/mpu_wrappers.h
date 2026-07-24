/*
 * FreeRTOS V10.4.6 - MPU wrappers (empty for STM32F103 — no MPU)
 */

#ifndef MPU_WRAPPERS_H
#define MPU_WRAPPERS_H

/* MPU is not used on STM32F103C8T6 */
#define portUSING_MPU_WRAPPERS   0

/* These are empty; real definitions only when MPU is enabled */
#if portUSING_MPU_WRAPPERS
    #error "MPU wrappers not supported for this port"
#endif

#endif /* MPU_WRAPPERS_H */
