#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY} HAL_StatusTypeDef;
typedef struct {uint32_t TSR, ESR;} CAN_TypeDef;
typedef struct {CAN_TypeDef *Instance;} CAN_HandleTypeDef;
typedef struct {unsigned unused;} UART_HandleTypeDef;
typedef struct {unsigned unused;} GPIO_TypeDef;
typedef struct {uint32_t StdId, IDE, RTR, DLC;} CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
typedef struct {
  uint32_t FilterBank, FilterMode, FilterScale, FilterIdHigh, FilterIdLow;
  uint32_t FilterMaskIdHigh, FilterMaskIdLow, FilterFIFOAssignment;
  uint32_t FilterActivation, SlaveStartFilterBank;
} CAN_FilterTypeDef;
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart7;
extern GPIO_TypeDef test_gpioe, test_gpiof;
#define GPIOE (&test_gpioe)
#define GPIOF (&test_gpiof)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_14 (1U << 14)
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_TSR_TXOK0 2U
#define CAN_TSR_TXOK1 512U
#define CAN_TSR_TXOK2 131072U
#define CAN_FLAG_RQCP0 0x500U
#define CAN_FLAG_RQCP1 0x508U
#define CAN_FLAG_RQCP2 0x510U
#define CAN_FLAG_MASK 0x1FU
#define CAN_ESR_BOFF 4U
#define CAN_ID_STD 0U
#define CAN_RTR_DATA 0U
#define CAN_RX_FIFO0 0U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 2U
#define ENABLE 1U
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define __HAL_UART_CLEAR_OREFLAG(x) ((void)(x))
#define __HAL_CAN_CLEAR_FLAG(h, f) ((h)->Instance->TSR &= ~((1U << ((f) & CAN_FLAG_MASK)) | (2U << ((f) & CAN_FLAG_MASK))))
uint32_t HAL_GetTick(void);
void Error_Handler(void);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint8_t *, uint32_t *);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, CAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
void HAL_GPIO_TogglePin(GPIO_TypeDef *, uint16_t);
#endif
