#ifndef BASE_CONTROL_H
#define BASE_CONTROL_H

#include <stdint.h>

/* 基地单电机控制的参数、状态和接口定义。
 *
 * 数据单位约定：
 * - RPM 都是 C620 回报的电机转子转速；经过减速箱后，机构转速会更低。
 * - 位置是电机转子编码器累计值，每圈 8192 个计数。
 * - HOME_CURRENT_GATE 对比 C620 回传的实际电流原始值；
 *   HOME_CURRENT_LIMIT 和 PID 输出则是发送给 C620 的电流指令原始值。
 *   这两类数值都不是安培，调参前要结合实机遥测。
 *
 * 本文件中的宏是调试入口。速度、阈值、超时、边界余量都在这里调，
 * 状态机代码读取这些宏，避免在多个函数里散落同一参数。 */
/* Commissioning values: tune on the actual mechanism before normal use.
 * Speeds are MOTOR ROTOR rpm; positions are rotor encoder counts (8192/rev).
 * Current gates use signed C620 feedback raw units, NOT amperes.
 * Command current limits use C620 command units, NOT amperes. */
#define BASE_MOTOR_ID                 2U
#define BASE_CONTROL_PERIOD_MS        1U
#define BASE_HOME_SPEED_RPM           500.0f
#define BASE_HOME_CURRENT_LIMIT       4000
#define BASE_HOME_CURRENT_GATE        2500
#define BASE_HOME_STOP_SPEED_RPM      50
#define BASE_HOME_CONFIRM_MS          200U
#define BASE_HOME_BLANK_MS            300U
#define BASE_HOME_TIMEOUT_MS          90000U
#define BASE_REVERSE_PAUSE_MS         100U
#define BASE_CENTER_TIMEOUT_MS        90000U
#define BASE_CENTER_TOLERANCE         500L
#define BASE_CENTER_SPEED_RPM         4000.0f
#define BASE_CENTER_STOP_SPEED_RPM    30
#define BASE_CENTER_STABLE_MS         100U
#define BASE_MIN_TRAVEL_COUNTS        2001L
#define BASE_TRAVERSE_MARGIN_PERCENT  1U
#define BASE_TRAVERSE_TOLERANCE_DIVISOR 4U
#define BASE_TRAVERSE_NEAR_SPEED_RPM  8000.0f
#define BASE_TRAVERSE_FAST_SPEED_RPM  8000.0f
#define BASE_TRAVERSE_ACCEL_RPM_PER_S 40000.0f
#define BASE_TRAVERSE_WAIT_MS         1000U
#define BASE_TRAVERSE_TIMEOUT_MS      30000U
#define BASE_TRAVERSE_SAMPLE_COUNT    10U
/* C620 speed feedback assists encoder unwrapping across short frame gaps. */
#define BASE_FEEDBACK_TIMEOUT_MS      20U
#define BASE_MAX_TRACKED_SPEED_RPM    10000
#define BASE_CAN_TX_TIMEOUT_MS        20U
#define BASE_DEBUG_PERIOD_MS          200U
#define BASE_DEBUG_DEFAULT            0U

#if (BASE_MOTOR_ID < 1U) || (BASE_MOTOR_ID > 4U)
#error "This application uses command frame 0x200 (motor IDs 1..4)."
#endif
#if (BASE_HOME_CURRENT_LIMIT <= 0) || (BASE_HOME_CURRENT_LIMIT > 16384)
#error "Homing command current limit must be in 1..16384."
#endif
#if (BASE_HOME_CURRENT_GATE <= 0) || (BASE_HOME_CURRENT_GATE > 32767)
#error "Homing feedback current gate must be in 1..32767."
#endif
#if (BASE_CENTER_TOLERANCE < 0) || (BASE_MIN_TRAVEL_COUNTS <= (2 * BASE_CENTER_TOLERANCE))
#error "Minimum travel must exceed twice the midpoint tolerance."
#endif
#if BASE_FEEDBACK_TIMEOUT_MS > 50U
#error "Feedback timeout is too long for safe speed-assisted tracking."
#endif
#if (BASE_TRAVERSE_MARGIN_PERCENT < 1U) || (BASE_TRAVERSE_MARGIN_PERCENT >= 50U)
#error "Traverse margin must be between 1 and 49 percent."
#endif
#if BASE_TRAVERSE_TOLERANCE_DIVISOR < 1U
#error "Traverse tolerance divisor must be positive."
#endif
#if BASE_TRAVERSE_SAMPLE_COUNT < 2U
#error "Traverse sampling requires at least two points."
#endif

typedef enum {
  /* 指令 1 寻零和回中流程。 */
  BASE_IDLE, BASE_SEEK_POSITIVE, BASE_REVERSE_PAUSE,
  BASE_SEEK_NEGATIVE, BASE_CENTERING,
  /* 指令 2 测速流程：先去最近安全边界，再等待，然后高速跨越。 */
  BASE_TRAVERSE_NEAR, BASE_TRAVERSE_WAIT, BASE_TRAVERSE_FAR,
  /* 两种正常结束状态各自保留，用来区分串口上报内容。 */
  BASE_DONE,
  BASE_TRAVERSE_DONE, BASE_FAULT
} BaseState;

/* 发生故障时清除本次端点/中点，防止在位置不可信时继续执行运动。 */
typedef enum {
  BASE_ERROR_NONE, BASE_ERROR_FEEDBACK, BASE_ERROR_POSITION,
  BASE_ERROR_HOME_TIMEOUT, BASE_ERROR_RANGE, BASE_ERROR_CENTER_TIMEOUT,
  BASE_ERROR_TRAVERSE_TIMEOUT, BASE_ERROR_CAN_TX
} BaseError;

typedef struct {
  int32_t position;          /* 展开后的多圈累计位置，单位为编码器计数 */
  uint16_t angle;            /* C620 回传的单圈角度，范围 0..8191 */
  int16_t speed;             /* 最新转速，单位 RPM */
  int16_t current;           /* 最新实际电流反馈，C620 原始单位 */
  float filtered_speed;      /* 低通速度，供速度 PID 使用 */
  uint32_t last_ms;          /* 最近一帧有效 CAN 反馈到达时刻 */
  uint32_t max_gap_ms;       /* 当前动作中记录到的最大反馈间隔，便于诊断 */
  uint8_t seen;              /* 至少成功解析过一帧反馈 */
  uint8_t tracking_invalid;  /* 多圈位置可能不准确，禁止继续运动 */
  uint8_t feedback_gap;      /* 位置失效由反馈间隔过长引起 */
} BaseFeedback;

typedef struct {
  /* 离散 PID：每个控制周期更新一次，积分和微分没有乘显式 dt。 */
  float kp, ki, kd;
  float integral, last_error;
  float integral_limit, p_limit, i_limit, output_limit;
} BasePID;

typedef struct {
  BaseState state;
  BaseError error;
  BasePID angle_pid, speed_pid;
  /* 指令 1 寻零所得的正端、负端和两端算术中点。 */
  int32_t positive_end, negative_end, midpoint;
  /* 指令 2 的先后两个安全边界；先后顺序取决于电机当前位置。 */
  int32_t near_target, far_target;
  int32_t traverse_tolerance;
  /* 指令 2 跨越过程中各测速点及其对应的实际转速。 */
  int32_t traverse_sample_positions[BASE_TRAVERSE_SAMPLE_COUNT];
  int16_t traverse_speed_samples[BASE_TRAVERSE_SAMPLE_COUNT];
  float traverse_speed_command;
  int16_t output;            /* 下一帧应发送给 C620 的电流指令 */
  uint32_t state_since;       /* 当前状态进入时刻，用来计时和超时保护 */
  uint32_t condition_since;   /* 到位/寻端条件首次连续成立的时刻 */
  uint32_t traverse_run_start_ms, traverse_run_elapsed_ms;
  uint32_t traverse_speed_command_ms;
  uint8_t condition_active, range_valid, traverse_sample_count;
} BaseControl;

void base_init(BaseControl *control);
/* 解析 C620 反馈的角度、速度、实际电流，并更新多圈位置。 */
void base_feedback_decode(BaseFeedback *feedback, const uint8_t data[8], uint32_t now);
/* 每次新的指令 1 开始前，用当前单圈角度建立一套新的累计坐标。 */
void base_feedback_rebase(BaseFeedback *feedback);
uint8_t base_feedback_fresh(const BaseFeedback *feedback, uint32_t now);
uint8_t base_busy(const BaseControl *control);
/* Returns 0 when busy, 1 when started, -1 on missing/invalid feedback. */
int base_start(BaseControl *control, const BaseFeedback *feedback, uint32_t now);
/* Returns 1 when started, 0 when busy, -1 when no homed range exists,
 * and -2 when feedback/position is invalid (the control enters FAULT). */
int base_traverse_start(BaseControl *control, const BaseFeedback *feedback, uint32_t now);
void base_fault(BaseControl *control, BaseError error);
/* 核心 1 ms 状态机：读取反馈快照，更新状态和 control->output。 */
void base_step(BaseControl *control, const BaseFeedback *feedback, uint32_t now);
/* 把有符号电流命令放入 C620 ID 对应的 0x200 帧槽位。 */
void base_pack_current(int16_t current, uint8_t data[8]);
const char *base_state_name(BaseState state);
const char *base_error_name(BaseError error);

#endif
