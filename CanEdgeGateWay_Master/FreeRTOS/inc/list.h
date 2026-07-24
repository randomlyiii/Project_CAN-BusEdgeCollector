/*
 * FreeRTOS V10.4.6 - Internal linked list
 */

#ifndef LIST_H
#define LIST_H

#ifdef __cplusplus
extern "C" {
#endif

struct xLIST;

struct xLIST_ITEM {
    TickType_t  xItemValue;
    struct xLIST_ITEM *pxNext;
    struct xLIST_ITEM *pxPrevious;
    void *pvOwner;
    struct xLIST *pvContainer;
};
typedef struct xLIST_ITEM ListItem_t;

struct xMINI_LIST_ITEM {
    TickType_t  xItemValue;
    struct xLIST_ITEM *pxNext;
    struct xLIST_ITEM *pxPrevious;
};
typedef struct xMINI_LIST_ITEM MiniListItem_t;

struct xLIST {
    UBaseType_t    uxNumberOfItems;
    ListItem_t    *pxIndex;
    MiniListItem_t xListEnd;
};
typedef struct xLIST List_t;

void vListInitialise(List_t * const pxList);
void vListInitialiseItem(ListItem_t * const pxItem);
void vListInsert(List_t * const pxList, ListItem_t * const pxNewListItem);
void vListInsertEnd(List_t * const pxList, ListItem_t * const pxNewListItem);
UBaseType_t uxListRemove(ListItem_t * const pxItemToRemove);

#define listSET_LIST_ITEM_OWNER(pxListItem, pxOwner)     ((pxListItem)->pvOwner = (void *)(pxOwner))
#define listGET_LIST_ITEM_OWNER(pxListItem)              ((pxListItem)->pvOwner)
#define listSET_LIST_ITEM_VALUE(pxListItem, xValue)      ((pxListItem)->xItemValue = (xValue))
#define listGET_LIST_ITEM_VALUE(pxListItem)              ((pxListItem)->xItemValue)
#define listGET_ITEM_VALUE_OF_HEAD_ENTRY(pxList)         ((((pxList)->xListEnd).pxNext)->xItemValue)
#define listLIST_IS_EMPTY(pxList)                        (((pxList)->uxNumberOfItems == 0U) ? pdTRUE : pdFALSE)
#define listCURRENT_LIST_LENGTH(pxList)                  ((pxList)->uxNumberOfItems)
#define listGET_OWNER_OF_NEXT_ENTRY(pxTCB, pxList)       { \
    List_t * const pxConstList = (pxList);                   \
    (pxConstList)->pxIndex = (pxConstList)->pxIndex->pxNext; \
    if ((void *)(pxConstList)->pxIndex == (void *)&((pxConstList)->xListEnd)) \
        (pxConstList)->pxIndex = (pxConstList)->pxIndex->pxNext; \
    (pxTCB) = (pxConstList)->pxIndex->pvOwner;               \
}
#define listGET_OWNER_OF_HEAD_ENTRY(pxList)              ((pxList)->xListEnd.pxNext->pvOwner)
#define listGET_END_MARKER(pxList)                       ((ListItem_t const *)(&((pxList)->xListEnd)))
#define listGET_HEAD_ENTRY(pxList)                       ((pxList)->xListEnd.pxNext)
#define listGET_NEXT(pxEntry)                            ((pxEntry)->pxNext)
#define listLIST_ITEM_CONTAINER(pxListItem)              ((pxListItem)->pvContainer)
#define listIS_CONTAINED_WITHIN(pxList, pxListItem)      (((pxListItem)->pvContainer) == (void *)(pxList))

#ifdef __cplusplus
}
#endif

#endif /* LIST_H */
