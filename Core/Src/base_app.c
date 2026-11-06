#include "base_app.h"
#include "base_control.h"
#include "main.h"
#include "can.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* UART 收到的命令先放入小环形队列，再由主循环执行。
 * 中断回调只做收字节、组帧和 CAN 反馈采样，不在中断里跑 PID 或 printf。 */
#define COMMAND_QUEUE_SIZE 32U
/* UART 回复可能来自状态切换、故障或测速结果，使用第二个队列串行发送。 */
#define RESPONSE_QUEUE_SIZE 16U
#define RESPONSE_SIZE 160U

static BaseControl control;             /* 核心算法维护的运行状态和当前位置目标 */
static volatile BaseFeedback feedback;  /* CAN ISR 写、主循环快照读取的电机反馈 */
static volatile uint8_t motion_busy, start_pending; /* UART ISR 用于及时拒绝重复运动命令 */
static uint8_t rx_byte, rx_first, rx_previous;
static uint16_t rx_length;
static volatile uint8_t rx_restart, command_overflow;
static volatile uint8_t commands[COMMAND_QUEUE_SIZE];
static volatile uint8_t command_head, command_tail;
static char responses[RESPONSE_QUEUE_SIZE][RESPONSE_SIZE];
static uint8_t response_head, response_tail;
static uint8_t tx_buffer[RESPONSE_SIZE];
static volatile uint8_t tx_busy;
static uint8_t debug_enabled = BASE_DEBUG_DEFAULT;
static uint32_t last_control_ms, last_debug_ms;
static uint32_t can_mailbox, last_can_ok_ms;
static uint8_t can_pending;
static int16_t can_pending_current;
static BaseState reported_state = BASE_IDLE;
static uint8_t active_motion_command = '1';

/* 复制 ISR 与主循环共享的数据时短暂关中断。
 * 返回进入函数前的 PRIMASK，保证原先已关闭中断时不会误开中断。 */
static uint32_t lock_interrupts(void)
{
  uint32_t previous = __get_PRIMASK();
  __disable_irq();
  return previous;
}

/* 反馈结构体不是单个机器字，复制一半时 CAN 中断可能更新剩余字段。
 * 通过临界区取得同一帧的一致快照，随后主循环在开中断状态下计算。 */
static BaseFeedback snapshot(void)
{
  BaseFeedback copy;
  uint32_t previous = lock_interrupts();
  copy = feedback;
  __set_PRIMASK(previous);
  return copy;
}

/* 所有回复都由主循环产生；发送完成中断只清除 tx_busy。
 * 队列满时丢弃新回复，避免覆盖仍在发送的数据。 */
static void respond(const char *text)
{
  uint8_t next = (uint8_t)((response_head + 1U) % RESPONSE_QUEUE_SIZE);
  if (next == response_tail) return;
  (void)snprintf(responses[response_head], RESPONSE_SIZE, "%s", text);
  response_head = next;
}

/* 把跨越阶段采到的 10 个转速分成每行最多 4 点的短消息。
 * 完成用时是从“开始跨越”到“远侧第一次稳定到位”，不包括近侧等待。 */
static void report_traverse_results(void)
{
  char text[RESPONSE_SIZE];
  uint8_t first, i;
  for (first = 0U; first < BASE_TRAVERSE_SAMPLE_COUNT; first = (uint8_t)(first + 4U)) {
    int length = snprintf(text, sizeof(text), "CMD 2 DATA");
    uint8_t end = (uint8_t)(first + 4U);
    if (end > BASE_TRAVERSE_SAMPLE_COUNT) end = BASE_TRAVERSE_SAMPLE_COUNT;
    for (i = first; i < end && length > 0 && length < (int)sizeof(text); ++i) {
      length += snprintf(text + length, sizeof(text) - (size_t)length,
                         " P%u=%ld,%d", (unsigned)(i + 1U),
                         (long)control.traverse_sample_positions[i],
                         (int)control.traverse_speed_samples[i]);
    }
    if (length > 0 && length < (int)(sizeof(text) - 2U)) {
      text[length++] = '\r';
      text[length++] = '\n';
      text[length] = '\0';
      respond(text);
    }
  }
  (void)snprintf(text, sizeof(text), "CMD 2 DONE TIME=%luMS RELEASED\r\n",
                 (unsigned long)control.traverse_run_elapsed_ms);
  respond(text);
}

/* 根据状态转换生成易读的蓝牙消息。
 * reported_state 防止主循环每 1 ms 重复报告同一个状态；测速结果只在
 * BASE_TRAVERSE_DONE 到来时一次性排队。 */
static void report_state(void)
{
  char text[RESPONSE_SIZE];
  if (control.state == reported_state) return;
  reported_state = control.state;
  switch (control.state) {
    case BASE_SEEK_POSITIVE:
      respond("CMD 1 START\r\n");
      break;
    case BASE_REVERSE_PAUSE:
      (void)snprintf(text, sizeof(text), "END POS=%ld\r\n", (long)control.positive_end);
      respond(text);
      break;
    case BASE_SEEK_NEGATIVE:
      respond("SEEK NEG\r\n");
      break;
    case BASE_CENTERING:
      (void)snprintf(text, sizeof(text), "RANGE NEG=%ld POS=%ld MID=%ld\r\n",
                     (long)control.negative_end, (long)control.positive_end,
                     (long)control.midpoint);
      respond(text);
      break;
    case BASE_TRAVERSE_NEAR:
      (void)snprintf(text, sizeof(text), "CMD 2 START NEAR=%ld FAR=%ld\r\n",
                     (long)control.near_target, (long)control.far_target);
      respond(text);
      break;
    case BASE_TRAVERSE_WAIT:
      respond("CMD 2 NEAR DONE WAIT 1S\r\n");
      break;
    case BASE_TRAVERSE_FAR:
      (void)snprintf(text, sizeof(text),
                     "CMD 2 GO FAR=%ld RECORD START POINTS=%u\r\n",
                     (long)control.far_target, (unsigned)BASE_TRAVERSE_SAMPLE_COUNT);
      respond(text);
      break;
    case BASE_DONE:
      respond("CMD 1 DONE RELEASED\r\n");
      break;
    case BASE_TRAVERSE_DONE:
      report_traverse_results();
      break;
    case BASE_FAULT:
      {
        BaseFeedback f = snapshot();
        uint32_t esr = hcan1.Instance->ESR;
        uint32_t gap = (uint32_t)(HAL_GetTick() - f.last_ms);
        if (f.max_gap_ms > gap) gap = f.max_gap_ms;
        /* bxCAN ESR 中包含收发错误计数、最近错误类型和 bus-off 标志。
         * 与最大反馈间隔一起上报，便于区分电机没上电、CAN 物理层异常，
         * 以及编码器位置无法可靠展开。 */
        (void)snprintf(text, sizeof(text),
                       "CMD %c ERROR %s TEC=%lu REC=%lu LEC=%lu BOFF=%u MAXGAP=%luMS\r\n",
                       active_motion_command, base_error_name(control.error),
                       (unsigned long)((esr >> 16) & 0xFFU),
                       (unsigned long)((esr >> 24) & 0xFFU),
                       (unsigned long)((esr >> 4) & 0x07U),
                       (unsigned)((esr & CAN_ESR_BOFF) != 0U),
                       (unsigned long)gap);
      }
      respond(text);
      break;
    default: break;
  }
}

static void handle_commands(void)
{
  while (command_tail != command_head) {
    uint8_t cmd = commands[command_tail];
    command_tail = (uint8_t)((command_tail + 1U) % COMMAND_QUEUE_SIZE);
    if (cmd == '1') {
      /* 命令 1：开始新一轮寻零。先禁止 ISR 接受第二个运动命令，
       * 原子读取反馈，并将本轮的多圈坐标从当前单圈角度重新起算。 */
      BaseFeedback f;
      uint32_t now, previous = lock_interrupts();
      motion_busy = 1U;
      start_pending = 0U;
      active_motion_command = '1';
      now = HAL_GetTick();
      f = feedback;
      if (base_feedback_fresh(&f, now)) {
        base_feedback_rebase(&f);
        feedback = f;
      }
      __set_PRIMASK(previous);
      /* ISR 在命令到达时就标记 BUSY，不能等主循环处理时才判断；
       * 否则快速结束或排队时可能把运动中的重复命令误当成新动作。 */
      if (base_start(&control, &f, now) == 1) last_can_ok_ms = now;
      motion_busy = base_busy(&control);
      reported_state = BASE_IDLE; /* Also report repeated offline failures. */
      report_state();
    } else if (cmd == '2') {
      /* 命令 2：只有已经测得有效行程时才允许运行测速跨越。
       * base_traverse_start 会从当前的位置选择最近的安全边界。 */
      BaseFeedback f;
      int result;
      uint32_t now, previous = lock_interrupts();
      motion_busy = 1U;
      start_pending = 0U;
      active_motion_command = '2';
      now = HAL_GetTick();
      f = feedback;
      __set_PRIMASK(previous);
      result = base_traverse_start(&control, &f, now);
      if (result == 1) {
        uint32_t reset_previous = lock_interrupts();
        feedback.max_gap_ms = 0U;
        __set_PRIMASK(reset_previous);
        last_can_ok_ms = now;
      }
      motion_busy = base_busy(&control);
      if (result == -1) respond("CMD 2 ERROR NOT_HOMED\r\n");
      else {
        reported_state = BASE_IDLE;
        report_state();
      }
    } else if (cmd == 'B') {
      respond("BUSY\r\n");
    } else if (cmd == '0') {
      /* 板载双 LED 测试，不是电机停止命令。 */
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
      HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
      respond("CMD 0 LED TOGGLED\r\n");
    } else if (cmd == '9') {
      /* 调试文本默认关闭，按 9 切换，降低蓝牙输出占用。 */
      debug_enabled = (uint8_t)!debug_enabled;
      last_debug_ms = HAL_GetTick();
      respond(debug_enabled ? "DEBUG ON 5HZ\r\n" : "DEBUG OFF\r\n");
    } else {
      respond("ERROR COMMAND\r\n");
    }
  }
  if (command_overflow) {
    uint32_t previous = lock_interrupts();
    command_overflow = 0U;
    __set_PRIMASK(previous);
    respond("ERROR RX_QUEUE_FULL\r\n");
  }
}

static void send_current(uint32_t now)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t data[8];
  if (can_pending && !HAL_CAN_IsTxMessagePending(&hcan1, can_mailbox)) {
    uint32_t ok = can_mailbox == CAN_TX_MAILBOX0 ? CAN_TSR_TXOK0 :
                  (can_mailbox == CAN_TX_MAILBOX1 ? CAN_TSR_TXOK1 : CAN_TSR_TXOK2);
    uint32_t complete = can_mailbox == CAN_TX_MAILBOX0 ? CAN_FLAG_RQCP0 :
                        (can_mailbox == CAN_TX_MAILBOX1 ? CAN_FLAG_RQCP1 : CAN_FLAG_RQCP2);
    if ((hcan1.Instance->TSR & ok) != 0U) last_can_ok_ms = now;
    /* RQCP/TXOK 是写 1 清除的粘滞标志。成功、失败或取消后都清本邮箱，
     * 防止上一帧的 TXOK 被误认为当前帧发送成功。 */
    __HAL_CAN_CLEAR_FLAG(&hcan1, complete);
    can_pending = 0U;
  }
  if (base_busy(&control) &&
      (((uint32_t)(now - last_can_ok_ms) >= BASE_CAN_TX_TIMEOUT_MS) ||
       ((hcan1.Instance->ESR & CAN_ESR_BOFF) != 0U))) {
    base_fault(&control, BASE_ERROR_CAN_TX);
  }
  if (can_pending) {
    /* 状态转换或故障需要零电流时，若上一条非零帧还在邮箱里就撤销它，
     * 防止零电流停在尚未发送的驱动命令后面排队。 */
    if ((control.output == 0) && (can_pending_current != 0))
      (void)HAL_CAN_AbortTxRequest(&hcan1, can_mailbox);
    return;
  }
  header.StdId = 0x200U;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;
  /* 同一帧可装入 4 个电调的电流命令；编码器算法负责把 ID 2 的
   * 有符号 16 位电流写入对应槽，其余三个槽发送零。 */
  base_pack_current(control.output, data);
  if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &can_mailbox) == HAL_OK) {
    can_pending = 1U;
    can_pending_current = control.output;
  }
}

static void uart_service(uint32_t now)
{
  BaseFeedback f;
  int length;
  if (rx_restart) {
    /* UART 接收错误后由主循环重启单字节中断接收；错误后的半帧会被丢弃。 */
    uint32_t previous = lock_interrupts();
    rx_restart = 0U;
    (void)HAL_UART_AbortReceive(&huart7);
    __HAL_UART_CLEAR_OREFLAG(&huart7);
    if (HAL_UART_Receive_IT(&huart7, &rx_byte, 1U) != HAL_OK) rx_restart = 1U;
    __set_PRIMASK(previous);
  }
  if (tx_busy) return;
  if (response_tail != response_head) {
    /* 回复优先于周期调试文本，避免高频调试数据挡住 START/DONE/ERROR。 */
    length = (int)strlen(responses[response_tail]);
    memcpy(tx_buffer, responses[response_tail], (size_t)length);
    tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart7, tx_buffer, (uint16_t)length) == HAL_OK)
      response_tail = (uint8_t)((response_tail + 1U) % RESPONSE_QUEUE_SIZE);
    else tx_busy = 0U;
    return;
  }
  if (!debug_enabled || ((uint32_t)(now - last_debug_ms) < BASE_DEBUG_PERIOD_MS)) return;
  last_debug_ms = now;
  f = snapshot();
  length = snprintf((char *)tx_buffer, sizeof(tx_buffer),
                    "DBG S=%s P=%ld V=%d IFB=%d ICMD=%d ONLINE=%u VALID=%u\r\n",
                    base_state_name(control.state), (long)f.position,
                    (int)f.speed, (int)f.current, (int)control.output,
                    (unsigned)base_feedback_fresh(&f, HAL_GetTick()),
                    (unsigned)control.range_valid);
  if (length > 0 && length < (int)sizeof(tx_buffer)) {
    tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart7, tx_buffer, (uint16_t)length) != HAL_OK) tx_busy = 0U;
  }
}

void base_app_init(void)
{
  CAN_FilterTypeDef filter = {0};
  base_init(&control);
  debug_enabled = BASE_DEBUG_DEFAULT;
  /* C620 ID 2 的反馈标准帧为 0x202。硬件过滤器先按 ID、帧格式筛选，
   * 降低无关中断；接收回调仍会复核 ID、DLC 和 DATA 帧类型。 */
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (0x200U + BASE_MOTOR_ID) << 5;
  filter.FilterMaskIdHigh = 0xFFE0U;
  filter.FilterMaskIdLow = 0x0006U; /* 同时匹配标准帧格式和 DATA 数据帧。 */
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
      HAL_CAN_Start(&hcan1) != HAL_OK ||
      HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();
  if (HAL_UART_Receive_IT(&huart7, &rx_byte, 1U) != HAL_OK) Error_Handler();
  last_control_ms = last_debug_ms = last_can_ok_ms = HAL_GetTick();
  respond("READY\r\n");
}

void base_app_poll(void)
{
  uint32_t now;
  handle_commands();
  now = HAL_GetTick();
  /* 主循环不使用 HAL_Delay：tick 到期时才推进一次状态机，
   * 其余时间可持续处理 UART/CAN 中断。 */
  if ((uint32_t)(now - last_control_ms) >= BASE_CONTROL_PERIOD_MS) {
    BaseFeedback f = snapshot();
    /* 先取得反馈快照，再读取当前时刻；防止快照中的新帧时间戳
     * 比超时计算使用的时刻还晚。 */
    now = HAL_GetTick();
    last_control_ms = now;
    base_step(&control, &f, now);
    send_current(now);
    motion_busy = base_busy(&control);
    report_state();
  }
  uart_service(HAL_GetTick());
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  if (hcan != &hcan1) return;
  /* 一次通知可能积累了多帧，循环读空 FIFO 防止接收队列溢出。
   * 反馈写入共享区后，主循环通过 snapshot() 原子读取。 */
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    BaseFeedback f;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
        header.DLC != 8U || header.StdId != (0x200U + BASE_MOTOR_ID)) continue;
    f = feedback;
    base_feedback_decode(&f, data, HAL_GetTick());
    feedback = f;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart7) return;
  /* 每次中断接收一个字符。仅接受精确的 1/2/0/9 加 CRLF；
   * 运动命令在收到时即占位，避免主循环稍后处理造成重复启动竞态。 */
  if (rx_length == 0U) rx_first = rx_byte;
  /* Saturate rather than wrapping: discard an entire overlong frame.
   * An embedded trailing "1" must never turn garbage into a home command. */
  if (rx_length < 256U) ++rx_length;
  if ((rx_previous == '\r') && (rx_byte == '\n')) {
    uint8_t cmd = (rx_length == 3U &&
                   (rx_first == '1' || rx_first == '2' ||
                    rx_first == '0' || rx_first == '9')) ? rx_first : 'E';
    uint8_t next = (uint8_t)((command_head + 1U) % COMMAND_QUEUE_SIZE);
    if (next != command_tail) {
      if (cmd == '1' || cmd == '2') {
        if (motion_busy || start_pending) cmd = 'B';
        else start_pending = 1U;
      }
      commands[command_head] = cmd;
      command_head = next;
    } else command_overflow = 1U;
    rx_length = 0U;
    rx_previous = 0U;
  } else rx_previous = rx_byte;
  if (HAL_UART_Receive_IT(&huart7, &rx_byte, 1U) != HAL_OK) rx_restart = 1U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* UART DMA/中断发送完毕，允许主循环取队列中的下一条回复。 */
  if (huart == &huart7) tx_busy = 0U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart7) return;
  /* UART 配置为每次收 1 字节；命令必须是单个数字加 CRLF。
   * 长帧长度饱和到 256 而不回绕，直到 CRLF 才丢弃并重新同步。 */
  rx_length = 256U;
  rx_previous = 0U;
  tx_busy = 0U;
  rx_restart = 1U;
}
