#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/* Compile the actual adapter, including its IRQ callbacks and UART parser. */
#include "../Core/Src/base_app.c"

static CAN_TypeDef registers;
CAN_HandleTypeDef hcan1 = {&registers};
UART_HandleTypeDef huart7;
GPIO_TypeDef test_gpioe, test_gpiof;
static uint32_t tick, mailbox_pending;
static uint8_t sent[8], incoming[8];
static CAN_RxHeaderTypeDef incoming_header;
static unsigned incoming_ready, abort_count, rx_rearms, uart_fail;
static unsigned uart_tx_count;
static uint16_t gpioe_state, gpiof_state;
static char last_uart[RESPONSE_SIZE];

uint32_t HAL_GetTick(void) {return tick;}
void Error_Handler(void) {assert(0);}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *h, uint32_t m)
{(void)h; (void)m; return mailbox_pending;}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t m)
{(void)h; (void)m; mailbox_pending = 0; registers.TSR = 0; ++abort_count; return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *hdr, uint8_t *d, uint32_t *m)
{
  (void)h;
  assert(hdr->StdId == 0x200 && hdr->DLC == 8 && hdr->IDE == CAN_ID_STD);
  memcpy(sent, d, 8); *m = CAN_TX_MAILBOX0; mailbox_pending = 1;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f)
{(void)h; assert(f->FilterIdHigh == (0x202U << 5)); return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) {(void)h; return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t n)
{(void)h; (void)n; return HAL_OK;}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo)
{(void)h; (void)fifo; return incoming_ready;}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo, CAN_RxHeaderTypeDef *hdr, uint8_t *d)
{(void)h; (void)fifo; *hdr = incoming_header; memcpy(d, incoming, 8); incoming_ready = 0; return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) {(void)h; return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *d, uint16_t n)
{(void)h; (void)d; assert(n == 1); ++rx_rearms; return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *d, uint16_t n)
{
  (void)h; assert(n > 0 && n < RESPONSE_SIZE);
  if (uart_fail) return HAL_BUSY;
  memcpy(last_uart, d, n);
  last_uart[n] = '\0';
  ++uart_tx_count;
  /* Exercise even the completion-before-return case. */
  HAL_UART_TxCpltCallback(&huart7);
  return HAL_OK;
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
  if (port == GPIOE) gpioe_state ^= pin;
  else if (port == GPIOF) gpiof_state ^= pin;
  else assert(0);
}

static void reset_app(void)
{
  BaseFeedback empty = {0};
  feedback = empty;
  tick = 100;
  motion_busy = start_pending = rx_restart = command_overflow = 0;
  rx_byte = rx_first = rx_previous = 0;
  rx_length = 0;
  command_head = command_tail = response_head = response_tail = 0;
  tx_busy = can_pending = 0;
  reported_state = BASE_IDLE;
  active_motion_command = '1';
  mailbox_pending = abort_count = incoming_ready = uart_fail = uart_tx_count = 0;
  gpioe_state = gpiof_state = 0;
  last_uart[0] = '\0';
  registers.TSR = registers.ESR = 0;
  memset(sent, 0, sizeof(sent));
  base_app_init();
}

static void receive_text(const char *s)
{
  while (*s) {rx_byte = (uint8_t)*s++; HAL_UART_RxCpltCallback(&huart7);}
}

static void receive_can(uint32_t id, uint32_t dlc, uint32_t rtr)
{
  incoming_header.StdId = id;
  incoming_header.DLC = dlc;
  incoming_header.RTR = rtr;
  incoming_header.IDE = CAN_ID_STD;
  incoming[0] = 0x1F; incoming[1] = 0x40; /* 8000 */
  incoming[2] = incoming[3] = 0;
  incoming[4] = 0xF0; incoming[5] = 0x60; /* -4000 */
  incoming_ready = 1;
  HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}

static void test_uart_and_can(void)
{
  unsigned i;
  reset_app();
  receive_can(0x203, 8, 0);
  receive_can(0x202, 7, 0);
  receive_can(0x202, 8, 2);
  assert(!feedback.seen);
  receive_can(0x202, 8, 0);
  assert(feedback.seen && feedback.current == -4000 && feedback.position == 8000);
  receive_text("1\r");
  handle_commands(); assert(control.state == BASE_IDLE);
  receive_text("\n1\r\n");
  assert(commands[0] == '1' && commands[1] == 'B');
  handle_commands();
  assert(control.state == BASE_SEEK_POSITIVE && motion_busy && !start_pending);
  receive_text("1\r\n");
  /* The command was BUSY when received, even if motor completes first. */
  control.state = BASE_DONE; motion_busy = 0;
  handle_commands(); assert(control.state == BASE_DONE);
  receive_text("1\r\n"); handle_commands();
  assert(control.state == BASE_SEEK_POSITIVE);
  reset_app();
  for (i = 0; i < 1024; ++i) receive_text("x");
  receive_text("1\r\n"); handle_commands();
  assert(control.state == BASE_IDLE && !start_pending);
  receive_text("11\r\n 1\r\n1 \r\n1\n\r\n"); handle_commands();
  assert(control.state == BASE_IDLE);
  receive_can(0x202, 8, 0);
  receive_text("1\r\n"); handle_commands();
  assert(control.state == BASE_SEEK_POSITIVE);
  reset_app();
  for (i = 0; i < 100; ++i) receive_text("bad\r\n");
  assert(command_overflow);
  handle_commands(); assert(!command_overflow && !start_pending);
  receive_text("1\r\n"); handle_commands();
  assert(control.state == BASE_FAULT && control.error == BASE_ERROR_FEEDBACK);
  receive_can(0x202, 8, 0);
  receive_text("1\r\n"); handle_commands();
  assert(control.state == BASE_SEEK_POSITIVE);
}

static void test_led_toggle_command(void)
{
  reset_app();
  receive_text("0\r\n");
  handle_commands();
  assert(gpioe_state == GPIO_PIN_11);
  assert(gpiof_state == GPIO_PIN_14);
  assert(strcmp(responses[1], "CMD 0 LED TOGGLED\r\n") == 0);

  receive_text("0\r\n");
  handle_commands();
  assert(gpioe_state == 0 && gpiof_state == 0);
  assert(strcmp(responses[2], "CMD 0 LED TOGGLED\r\n") == 0);
}

static void test_debug_command_and_rate(void)
{
  reset_app();
  assert(!debug_enabled);
  uart_service(tick);
  assert(uart_tx_count == 1 && strcmp(last_uart, "READY\r\n") == 0);

  tick += 1000U;
  uart_service(tick);
  assert(uart_tx_count == 1); /* Debug output is off by default. */

  receive_text("9\r\n");
  handle_commands();
  assert(debug_enabled);
  uart_service(tick);
  assert(uart_tx_count == 2 && strcmp(last_uart, "DEBUG ON 5HZ\r\n") == 0);

  tick += BASE_DEBUG_PERIOD_MS - 1U;
  uart_service(tick);
  assert(uart_tx_count == 2);
  ++tick;
  uart_service(tick);
  assert(uart_tx_count == 3 && strncmp(last_uart, "DBG ", 4) == 0);
  tick += BASE_DEBUG_PERIOD_MS;
  uart_service(tick);
  assert(uart_tx_count == 4 && strncmp(last_uart, "DBG ", 4) == 0);

  receive_text("9\r\n");
  handle_commands();
  assert(!debug_enabled);
  uart_service(tick);
  assert(uart_tx_count == 5 && strcmp(last_uart, "DEBUG OFF\r\n") == 0);
  tick += 1000U;
  uart_service(tick);
  assert(uart_tx_count == 5);
}

static void test_traverse_command(void)
{
  unsigned i;
  reset_app();
  feedback.seen = 1;
  feedback.last_ms = tick;
  feedback.position = 4500;
  control.state = BASE_DONE;
  control.negative_end = 0;
  control.positive_end = 10000;
  control.midpoint = 5000;
  control.range_valid = 1;

  receive_text("2\r\n");
  handle_commands();
  assert(control.state == BASE_TRAVERSE_NEAR && motion_busy && !start_pending);
  assert(control.near_target == 100 && control.far_target == 9900);
  assert(strcmp(responses[1], "CMD 2 START NEAR=100 FAR=9900\r\n") == 0);
  receive_text("2\r\n");
  handle_commands();
  assert(strcmp(responses[2], "BUSY\r\n") == 0);
  for (i = 0; i < BASE_TRAVERSE_SAMPLE_COUNT; ++i) {
    control.traverse_sample_positions[i] = (int32_t)(100 + i * 1000);
    control.traverse_speed_samples[i] = (int16_t)(i * 100);
  }
  control.traverse_sample_count = BASE_TRAVERSE_SAMPLE_COUNT;
  control.traverse_run_elapsed_ms = 1234U;
  control.state = BASE_TRAVERSE_DONE;
  report_state();
  assert(strncmp(responses[3], "CMD 2 DATA P1=100,0", 19) == 0);
  assert(strcmp(responses[6], "CMD 2 DONE TIME=1234MS RELEASED\r\n") == 0);

  reset_app();
  receive_text("2\r\n");
  handle_commands();
  assert(!motion_busy && !start_pending);
  assert(strcmp(responses[1], "CMD 2 ERROR NOT_HOMED\r\n") == 0);
}

static void test_fault_diagnostics(void)
{
  reset_app();
  feedback.seen = 1U;
  feedback.last_ms = tick - 5U;
  feedback.max_gap_ms = 25U;
  registers.ESR = CAN_ESR_BOFF | (3U << 4) | (9U << 16) | (7U << 24);
  active_motion_command = '2';
  base_fault(&control, BASE_ERROR_CAN_TX);
  reported_state = BASE_IDLE;
  report_state();
  assert(strcmp(responses[1],
    "CMD 2 ERROR CAN_TX TEC=9 REC=7 LEC=3 BOFF=1 MAXGAP=25MS\r\n") == 0);
}

static void test_can_failure_and_stop(void)
{
  unsigned i;
  reset_app();
  receive_can(0x202, 8, 0);
  receive_text("1\r\n"); handle_commands();
  control.output = 3000;
  send_current(tick);
  assert(sent[2] == 0x0B && sent[3] == 0xB8);
  base_fault(&control, BASE_ERROR_FEEDBACK);
  send_current(++tick);
  assert(abort_count == 1);
  send_current(++tick);
  for (i = 0; i < 8; ++i) assert(sent[i] == 0);
  reset_app();
  receive_can(0x202, 8, 0);
  receive_text("1\r\n"); handle_commands();
  /* Mailbox acceptance is not proof that a frame reached the bus. */
  for (i = 1; i <= 3; ++i) {
    mailbox_pending = 0;
    registers.TSR |= CAN_TSR_TXOK0 | CAN_FLAG_RQCP0;
    send_current(++tick);
  }
  assert(control.state == BASE_SEEK_POSITIVE);
  for (i = 1; i <= BASE_CAN_TX_TIMEOUT_MS; ++i) {
    mailbox_pending = 0; registers.TSR |= 1U | CAN_FLAG_RQCP0; /* Completed without TXOK. */
    send_current(++tick);
  }
  assert(control.state == BASE_FAULT && control.error == BASE_ERROR_CAN_TX);
  reset_app();
  receive_can(0x202, 8, 0);
  receive_text("1\r\n"); handle_commands();
  send_current(tick);
  for (i = 1; i <= 100; ++i) {
    mailbox_pending = 0; registers.TSR |= CAN_TSR_TXOK0 | CAN_FLAG_RQCP0;
    send_current(++tick);
  }
  assert(control.state == BASE_SEEK_POSITIVE);
  registers.ESR = CAN_ESR_BOFF;
  send_current(++tick);
  assert(control.state == BASE_FAULT && control.output == 0);
}

static void test_uart_recovery(void)
{
  uint8_t old_tail;
  reset_app();
  old_tail = response_tail;
  uart_fail = 1;
  uart_service(tick);
  assert(response_tail == old_tail && !tx_busy);
  uart_fail = 0;
  uart_service(tick);
  assert(response_tail != old_tail && !tx_busy);
  receive_text("1");
  tx_busy = 1U;
  HAL_UART_ErrorCallback(&huart7);
  assert(rx_restart && !tx_busy);
  uart_service(tick);
  assert(!rx_restart);
  receive_text("\r\n"); handle_commands();
  assert(control.state == BASE_IDLE);
  receive_can(0x202, 8, 0);
  receive_text("1\r\n"); handle_commands();
  assert(control.state == BASE_SEEK_POSITIVE);
}

int main(void)
{
  test_uart_and_can();
  test_led_toggle_command();
  test_debug_command_and_rate();
  test_traverse_command();
  test_fault_diagnostics();
  test_can_failure_and_stop();
  test_uart_recovery();
  puts("PASS: actual UART/CAN callbacks, framing, overlength/queue overflow,");
  puts("      arrival-time BUSY, LED/debug commands and rate, retry, TX completion/failure, pending stop, UART recovery");
  return 0;
}
