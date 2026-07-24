/*
 * FreeRTOS V10.4.6 - Event Group API
 */

#ifndef EVENT_GROUPS_H
#define EVENT_GROUPS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void            * EventGroupHandle_t;
typedef TickType_t        EventBits_t;

/* Event group control bits — stored in event list item value */
#define eventEVENT_BITS_CONTROL_BYTES    0xFF000000UL
#define eventCLEAR_EVENTS_ON_EXIT_BIT    0x01000000UL
#define eventUNBLOCKED_DUE_TO_BIT_SET    0x02000000UL
#define eventWAIT_FOR_ALL_BITS           0x04000000UL
#define eventSYNC_POINT_BIT              0x08000000UL

EventGroupHandle_t xEventGroupCreate(void);
EventBits_t xEventGroupSetBits(EventGroupHandle_t xEventGroup,
                               const EventBits_t uxBitsToSet);
EventBits_t xEventGroupSetBitsFromISR(EventGroupHandle_t xEventGroup,
                                      const EventBits_t uxBitsToSet,
                                      BaseType_t *pxHigherPriorityTaskWoken);
EventBits_t xEventGroupClearBits(EventGroupHandle_t xEventGroup,
                                 const EventBits_t uxBitsToClear);
EventBits_t xEventGroupGetBits(EventGroupHandle_t xEventGroup);
EventBits_t xEventGroupGetBitsFromISR(EventGroupHandle_t xEventGroup);

EventBits_t xEventGroupWaitBits(EventGroupHandle_t xEventGroup,
                                const EventBits_t uxBitsToWaitFor,
                                const BaseType_t xClearOnExit,
                                const BaseType_t xWaitForAllBits,
                                TickType_t xTicksToWait);

EventBits_t xEventGroupSync(EventGroupHandle_t xEventGroup,
                            const EventBits_t uxBitsToSet,
                            const EventBits_t uxBitsToWaitFor,
                            TickType_t xTicksToWait);

void vEventGroupDelete(EventGroupHandle_t xEventGroup);
void vEventGroupSetBitsCallback(void *pvEventGroup,
                                const uint32_t ulBitsToSet);
void vEventGroupClearBitsCallback(void *pvEventGroup,
                                  const uint32_t ulBitsToClear);

#ifdef __cplusplus
}
#endif

#endif /* EVENT_GROUPS_H */
