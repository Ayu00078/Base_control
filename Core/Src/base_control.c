#include "base_control.h"
#include <limits.h>
#include <string.h>

/* 对对称上下限做截断，PID 和速度斜坡共用。 */
static float clamp(float x, float limit)
{
  return x > limit ? limit : (x < -limit ? -limit : x);
}

/* 先扩展为 32 位再取绝对值，避免 int16_t 的 -32768 溢出。 */
static int32_t magnitude(int16_t x)
{
  return x < 0 ? -(int32_t)x : (int32_t)x;
}

/* 位置错误需要细分原因：长时间没收到帧是反馈故障，
 * 角度/速度无法解释或计数溢出则是位置跟踪故障。 */
static BaseError feedback_tracking_error(const BaseFeedback *f)
{
  return f->feedback_gap ? BASE_ERROR_FEEDBACK : BASE_ERROR_POSITION;
}

/* 切换状态时清空历史误差，防止上一个阶段的积分、微分冲击带入下阶段。 */
static void reset_pid(BasePID *pid)
{
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
}

/* 一个 1 ms 离散 PID 计算。先限各项，再限合成输出，减少积分项过度增长。 */
static float update_pid(BasePID *pid, float error)
{
  float derivative = error - pid->last_error;
  pid->last_error = error;
  pid->integral = clamp(pid->integral + error, pid->integral_limit);
  return clamp(clamp(pid->kp * error, pid->p_limit) +
               clamp(pid->ki * pid->integral, pid->i_limit) +
               pid->kd * derivative, pid->output_limit);
}

/* 集中处理状态转换的通用动作：启动计时、撤掉旧电流输出并重置 PID。 */
static void enter_state(BaseControl *c, BaseState state, uint32_t now)
{
  c->state = state;
  c->state_since = now;
  c->condition_active = 0U;
  c->output = 0;
  c->traverse_speed_command = 0.0f;
  c->traverse_speed_command_ms = now;
  reset_pid(&c->angle_pid);
  reset_pid(&c->speed_pid);
}

/* 指令 2 起步时限制速度变化率，避免突然给出 8000 rpm 的目标。
 * 目标变化到相反方向时先把速度目标拉回零；需要减速时立即响应，
 * 不再按斜坡拖延，以免越过安全边界。 */
static float ramp_traverse_speed(BaseControl *c, float desired, uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - c->traverse_speed_command_ms);
  float change = BASE_TRAVERSE_ACCEL_RPM_PER_S * (float)elapsed / 1000.0f;
  float current = c->traverse_speed_command;
  c->traverse_speed_command_ms = now;
  if ((current > 0.0f && desired < 0.0f) ||
      (current < 0.0f && desired > 0.0f)) {
    current = 0.0f;
  } else if ((desired > current) && (current >= 0.0f)) {
    current = desired - current > change ? current + change : desired;
  } else if ((desired < current) && (current <= 0.0f)) {
    current = current - desired > change ? current - change : desired;
  } else {
    /* 目标要求减速或已经靠近目标时立刻降速，避免因斜坡而冲过边界。 */
    current = desired;
  }
  c->traverse_speed_command = current;
  return current;
}

/* 对某个条件做持续时间确认。条件中途有一次不满足，就重新计时。
 * 寻零用它滤掉短时电流尖峰，定位用它避免刚经过目标就宣布到位。 */
static uint8_t confirmed(BaseControl *c, uint8_t condition,
                         uint32_t now, uint32_t duration)
{
  if (!condition) {
    c->condition_active = 0U;
    return 0U;
  }
  if (!c->condition_active) {
    c->condition_active = 1U;
    c->condition_since = now;
  }
  return (uint8_t)((uint32_t)(now - c->condition_since) >= duration);
}

/* 位置从 near_target 朝 far_target 单调移动时，逐一捕获已经越过的测速点。
 * 一帧可能跨过不止一个点，因此用 while 补记，避免漏样。 */
static void record_traverse_samples(BaseControl *c, const BaseFeedback *f)
{
  while (c->traverse_sample_count < BASE_TRAVERSE_SAMPLE_COUNT) {
    uint8_t index = c->traverse_sample_count;
    int32_t target = c->traverse_sample_positions[index];
    uint8_t crossed = c->far_target >= c->near_target ?
                      (uint8_t)(f->position >= target) :
                      (uint8_t)(f->position <= target);
    if (!crossed) break;
    c->traverse_speed_samples[index] = f->speed;
    c->traverse_sample_count = (uint8_t)(index + 1U);
  }
}

void base_init(BaseControl *c)
{
  /* 保留原镖架工程的 3508 PID 增益，位置环更新周期仍是 1 ms。
   * 指令 2 需要较高速度，位置环上限设为 8000 rpm；指令 1 回中时
   * 还会在 base_step 中再次限到 BASE_CENTER_SPEED_RPM。
   * 字段顺序见 BasePID：kp、ki、kd、当前积分、上次误差、积分限幅、
   * P 项限幅、I 项限幅、最终输出限幅。 */
  const BasePID angle = {0.1f, 0.01f, 0.01f, 0.0f, 0.0f,
                         100.0f, 8000.0f, 100.0f, 8000.0f};
  const BasePID speed = {25.0f, 0.1f, 0.01f, 0.0f, 0.0f,
                         100000.0f, 16384.0f, 6000.0f, 16384.0f};
  memset(c, 0, sizeof(*c));
  c->angle_pid = angle;
  c->speed_pid = speed;
}

void base_feedback_decode(BaseFeedback *f, const uint8_t data[8], uint32_t now)
{
  /* C620 标准反馈帧按大端序排列：角度[0..1]、转速[2..3]、
   * 实际转矩电流[4..5]。位置和速度符号均沿用电调回报方向。 */
  uint16_t angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  int16_t speed = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  int64_t delta, expected, numerator, wraps, residual, next;
  uint32_t gap;
  if (angle > 8191U) {
    f->tracking_invalid = 1U;
    return;
  }
  if (!f->seen) {
    /* 上电后第一帧只知道当前单圈角度，先将其作为累计坐标起点。 */
    f->position = angle;
    f->filtered_speed = (float)speed;
  } else {
    gap = (uint32_t)(now - f->last_ms);
    if (gap > f->max_gap_ms) f->max_gap_ms = gap;
    if (gap > BASE_FEEDBACK_TIMEOUT_MS) {
      /* 间隔太长时可能已经转过不止半圈；不能可靠地猜回绕次数。 */
      f->tracking_invalid = 1U;
      f->feedback_gap = 1U;
    } else {
      /* 用前后两帧速度预测这段时间转过的圈数，再选择最接近预测位移的
       * 编码器角度展开值。短暂漏帧时，这比只看最短角度差更可靠。 */
      delta = (int32_t)angle - (int32_t)f->angle;
      numerator = ((int64_t)f->speed + speed) * gap * 8192;
      /* 两帧速度的平均值乘间隔，换算成编码器计数：
       * 平均 rpm × gap(ms) × 8192 / 60000。
       * 代码用速度之和，因此分母是 120000；正负值都按最近整数取整。 */
      expected = numerator >= 0 ? (numerator + 60000) / 120000 :
                                  (numerator - 60000) / 120000;
      if (expected >= delta) wraps = (expected - delta + 4096) / 8192;
      else wraps = (expected - delta - 4096) / 8192;
      delta += wraps * 8192;
      /* 把单圈角度差加上若干个整圈，选出最接近速度预测值的位移。
       * 若预测刚好处于两种解释的中间（差 4096），方向无法判定。 */
      residual = delta - expected;
      if ((residual == 4096) || (residual == -4096))
        f->tracking_invalid = 1U;
      next = (int64_t)f->position + delta;
      if ((next > INT32_MAX) || (next < INT32_MIN)) f->tracking_invalid = 1U;
      else f->position = (int32_t)next;
    }
    f->filtered_speed = (float)speed * 0.6f + f->filtered_speed * 0.4f;
  }
  if (magnitude(speed) > BASE_MAX_TRACKED_SPEED_RPM) f->tracking_invalid = 1U;
  f->angle = angle;
  f->speed = speed;
  f->current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  f->last_ms = now;
  f->seen = 1U;
}

void base_feedback_rebase(BaseFeedback *f)
{
  /* 只在一轮新的指令 1 开始时调用，并在应用层关中断保护。
   * 旧端点坐标仅对上一轮有效，因此把当前位置从单圈角度重新起算，
   * 同时清除上一轮的反馈间隔诊断值。 */
  f->position = f->angle;
  f->filtered_speed = (float)f->speed;
  f->tracking_invalid = (uint8_t)(magnitude(f->speed) > BASE_MAX_TRACKED_SPEED_RPM);
  f->feedback_gap = 0U;
  f->max_gap_ms = 0U;
}

uint8_t base_feedback_fresh(const BaseFeedback *f, uint32_t now)
{
  return (uint8_t)(f->seen && ((uint32_t)(now - f->last_ms) <= BASE_FEEDBACK_TIMEOUT_MS));
}

uint8_t base_busy(const BaseControl *c)
{
  /* DONE、TRAVERSE_DONE、IDLE 和 FAULT 都不是正在主动运动的状态。 */
  return (uint8_t)((c->state == BASE_SEEK_POSITIVE) ||
                   (c->state == BASE_REVERSE_PAUSE) ||
                   (c->state == BASE_SEEK_NEGATIVE) ||
                   (c->state == BASE_CENTERING) ||
                   (c->state == BASE_TRAVERSE_NEAR) ||
                   (c->state == BASE_TRAVERSE_WAIT) ||
                   (c->state == BASE_TRAVERSE_FAR));
}

void base_fault(BaseControl *c, BaseError error)
{
  /* 发生任何运动错误后，本次测得的范围一律作废；应用层会继续
   * 周期发送零电流。之后需要重新执行完整寻零才能运行指令 2。 */
  c->state = BASE_FAULT;
  c->error = error;
  c->output = 0;
  c->range_valid = 0U;
  c->positive_end = c->negative_end = c->midpoint = 0;
  c->near_target = c->far_target = 0;
  c->traverse_tolerance = 0;
  c->traverse_sample_count = 0U;
  c->traverse_run_start_ms = c->traverse_run_elapsed_ms = 0U;
  c->condition_active = 0U;
  reset_pid(&c->angle_pid);
  reset_pid(&c->speed_pid);
}

int base_start(BaseControl *c, const BaseFeedback *f, uint32_t now)
{
  if (base_busy(c)) return 0;
  base_init(c);
  if (!base_feedback_fresh(f, now)) {
    base_fault(c, BASE_ERROR_FEEDBACK);
    return -1;
  }
  if (f->tracking_invalid) {
    base_fault(c, feedback_tracking_error(f));
    return -1;
  }
  /* 不自动启动：只有串口收到命令 1 且 CAN 反馈正常才进入此状态。 */
  enter_state(c, BASE_SEEK_POSITIVE, now);
  return 1;
}

int base_traverse_start(BaseControl *c, const BaseFeedback *f, uint32_t now)
{
  int64_t span, margin, low_target, high_target;
  int64_t low_distance, high_distance;
  uint8_t i;
  if (base_busy(c)) return 0;
  if (!c->range_valid) return -1;
  if (!base_feedback_fresh(f, now)) {
    base_fault(c, BASE_ERROR_FEEDBACK);
    return -2;
  }
  if (f->tracking_invalid) {
    base_fault(c, feedback_tracking_error(f));
    return -2;
  }
  span = (int64_t)c->positive_end - c->negative_end;
  if (span < BASE_MIN_TRAVEL_COUNTS) {
    base_fault(c, BASE_ERROR_RANGE);
    return -2;
  }
  /* 每侧留出实测行程的一小段余量，不直接冲向曾经碰到的机械限位。
   * 根据当前位置判断先去哪个安全边界，之后再执行一次完整跨越。 */
  margin = span * BASE_TRAVERSE_MARGIN_PERCENT / 100;
  if (margin < 1) margin = 1;
  c->traverse_tolerance = (int32_t)(margin / BASE_TRAVERSE_TOLERANCE_DIVISOR);
  if (c->traverse_tolerance < 1) c->traverse_tolerance = 1;
  low_target = (int64_t)c->negative_end + margin;
  high_target = (int64_t)c->positive_end - margin;
  low_distance = (int64_t)f->position - low_target;
  high_distance = high_target - f->position;
  if (low_distance < 0) low_distance = -low_distance;
  if (high_distance < 0) high_distance = -high_distance;
  if (low_distance <= high_distance) {
    c->near_target = (int32_t)low_target;
    c->far_target = (int32_t)high_target;
  } else {
    c->near_target = (int32_t)high_target;
    c->far_target = (int32_t)low_target;
  }
  for (i = 0U; i < BASE_TRAVERSE_SAMPLE_COUNT; ++i) {
    /* 将测速点平均布在近、远两个安全目标之间，包含两个端点。 */
    c->traverse_sample_positions[i] = (int32_t)((int64_t)c->near_target +
      ((int64_t)c->far_target - c->near_target) * i /
      (BASE_TRAVERSE_SAMPLE_COUNT - 1U));
    c->traverse_speed_samples[i] = 0;
  }
  c->traverse_sample_count = 0U;
  c->traverse_run_start_ms = c->traverse_run_elapsed_ms = 0U;
  c->error = BASE_ERROR_NONE;
  enter_state(c, BASE_TRAVERSE_NEAR, now);
  return 1;
}

void base_step(BaseControl *c, const BaseFeedback *f, uint32_t now)
{
  int64_t span, error;
  uint8_t stopped;
  float target_speed;
  /* 这是每 1 ms 调用的控制状态机。所有运动决策都基于 ISR 更新的
   * 最新反馈快照；空闲/完成/故障时输出保持为零。 */
  if (!base_busy(c)) {
    c->output = 0;
    return;
  }
  if (!base_feedback_fresh(f, now)) {
    base_fault(c, BASE_ERROR_FEEDBACK);
    return;
  }
  if (f->tracking_invalid) {
    base_fault(c, feedback_tracking_error(f));
    return;
  }
  if ((c->state == BASE_REVERSE_PAUSE) || (c->state == BASE_TRAVERSE_WAIT)) {
    /* 两种等待状态都明确输出零电流：一是正端寻到后换向，
     * 二是指令 2 到近侧安全边界后等待 1 秒再开始测速跨越。 */
    c->output = 0;
    if (c->state == BASE_REVERSE_PAUSE) {
      if ((uint32_t)(now - c->state_since) >= BASE_REVERSE_PAUSE_MS)
        enter_state(c, BASE_SEEK_NEGATIVE, now);
    } else if ((uint32_t)(now - c->state_since) >= BASE_TRAVERSE_WAIT_MS) {
      enter_state(c, BASE_TRAVERSE_FAR, now);
      c->traverse_run_start_ms = now;
      record_traverse_samples(c, f);
    }
    return;
  }
  if ((c->state == BASE_SEEK_POSITIVE) || (c->state == BASE_SEEK_NEGATIVE)) {
    if ((uint32_t)(now - c->state_since) >= BASE_HOME_TIMEOUT_MS) {
      base_fault(c, BASE_ERROR_HOME_TIMEOUT);
      return;
    }
    target_speed = c->state == BASE_SEEK_POSITIVE ? BASE_HOME_SPEED_RPM : -BASE_HOME_SPEED_RPM;
    c->output = (int16_t)clamp(update_pid(&c->speed_pid,
                              target_speed - f->filtered_speed), BASE_HOME_CURRENT_LIMIT);
    /* 寻零方向由目标速度符号决定，PID 的电流指令单独限幅。
     * 端点判据使用实际反馈电流，不是控制器自己输出的电流命令。
     * 电流高、转速低必须同时持续满足；启动屏蔽时间两边都适用。 */
    stopped = (uint8_t)(((uint32_t)(now - c->state_since) >= BASE_HOME_BLANK_MS) &&
                       (magnitude(f->current) >= BASE_HOME_CURRENT_GATE) &&
                       (magnitude(f->speed) <= BASE_HOME_STOP_SPEED_RPM));
    if (confirmed(c, stopped, now, BASE_HOME_CONFIRM_MS)) {
      if (c->state == BASE_SEEK_POSITIVE) {
        /* 记录正端后先零电流停顿，再反向寻找另一端。 */
        c->positive_end = f->position;
        enter_state(c, BASE_REVERSE_PAUSE, now);
      } else {
        /* 负端确认后验证端点顺序和最小行程，再求两端位置的中点。 */
        c->negative_end = f->position;
        span = (int64_t)c->positive_end - c->negative_end;
        if (span < BASE_MIN_TRAVEL_COUNTS) {
          base_fault(c, BASE_ERROR_RANGE);
          return;
        }
        c->midpoint = (int32_t)((int64_t)c->negative_end + span / 2);
        c->range_valid = 1U;
        enter_state(c, BASE_CENTERING, now);
      }
    }
    return;
  }
  /* 回中和指令 2 的每段运动都有独立超时；无论 PID 是否仍在给输出，
   * 到时都清范围并退出主动运动。 */
  if ((uint32_t)(now - c->state_since) >=
      (c->state == BASE_CENTERING ? BASE_CENTER_TIMEOUT_MS : BASE_TRAVERSE_TIMEOUT_MS)) {
    base_fault(c, c->state == BASE_CENTERING ?
                     BASE_ERROR_CENTER_TIMEOUT : BASE_ERROR_TRAVERSE_TIMEOUT);
    return;
  }
  /* 位置误差为目标位置减当前累计位置。接近目标且速度已足够低后，
   * 连续确认一段时间才结束，结束态会把输出置零并释放电机。 */
  error = (int64_t)(c->state == BASE_CENTERING ? c->midpoint :
                    (c->state == BASE_TRAVERSE_NEAR ? c->near_target : c->far_target)) -
          f->position;
  if (c->state == BASE_TRAVERSE_FAR) record_traverse_samples(c, f);
  {
    int32_t tolerance = c->state == BASE_CENTERING ?
                        BASE_CENTER_TOLERANCE : c->traverse_tolerance;
    stopped = (uint8_t)((error >= -(int64_t)tolerance) &&
                       (error <= (int64_t)tolerance) &&
                       (magnitude(f->speed) <= BASE_CENTER_STOP_SPEED_RPM));
  }
  if (confirmed(c, stopped, now, BASE_CENTER_STABLE_MS)) {
    if (c->state == BASE_CENTERING) enter_state(c, BASE_DONE, now);
    else if (c->state == BASE_TRAVERSE_NEAR) enter_state(c, BASE_TRAVERSE_WAIT, now);
    else {
      while (c->traverse_sample_count < BASE_TRAVERSE_SAMPLE_COUNT) {
        c->traverse_speed_samples[c->traverse_sample_count] = f->speed;
        ++c->traverse_sample_count;
      }
      c->traverse_run_elapsed_ms =
        (uint32_t)(c->condition_since - c->traverse_run_start_ms);
      enter_state(c, BASE_TRAVERSE_DONE, now);
    }
    return;
  }
  target_speed = update_pid(&c->angle_pid, (float)error);
  /* 外环把位置误差换成目标转速，内环再把转速误差换成 CAN 电流。
   * 指令 1 回中限速较低；指令 2 两段由独立速度上限控制。 */
  if (c->state == BASE_CENTERING)
    target_speed = clamp(target_speed, BASE_CENTER_SPEED_RPM);
  else if (c->state == BASE_TRAVERSE_NEAR)
    target_speed = clamp(target_speed, BASE_TRAVERSE_NEAR_SPEED_RPM);
  else if (c->state == BASE_TRAVERSE_FAR)
    target_speed = clamp(target_speed, BASE_TRAVERSE_FAST_SPEED_RPM);
  if ((c->state == BASE_TRAVERSE_NEAR) || (c->state == BASE_TRAVERSE_FAR))
    target_speed = ramp_traverse_speed(c, target_speed, now);
  c->output = (int16_t)update_pid(&c->speed_pid, target_speed - f->filtered_speed);
}

void base_pack_current(int16_t current, uint8_t data[8])
{
  /* C620 通过 0x200 公共电流命令帧控制 1～4 号电机。
   * 每个电机占两个大端字节，ID 2 位于 data[2..3]，其它槽清零。 */
  uint32_t offset = (BASE_MOTOR_ID - 1U) * 2U;
  memset(data, 0, 8U);
  data[offset] = (uint8_t)((uint16_t)current >> 8);
  data[offset + 1U] = (uint8_t)current;
}

const char *base_state_name(BaseState state)
{
  /* 调试串口状态字符串；修改枚举后应同步维护此映射。 */
  switch (state) {
    case BASE_IDLE: return "IDLE";
    case BASE_SEEK_POSITIVE: return "SEEK_POS";
    case BASE_REVERSE_PAUSE: return "REVERSE_PAUSE";
    case BASE_SEEK_NEGATIVE: return "SEEK_NEG";
    case BASE_CENTERING: return "CENTER";
    case BASE_TRAVERSE_NEAR: return "MOVE_NEAR";
    case BASE_TRAVERSE_WAIT: return "MOVE_WAIT";
    case BASE_TRAVERSE_FAR: return "MOVE_FAR";
    case BASE_DONE: return "DONE";
    case BASE_TRAVERSE_DONE: return "MOVE_DONE";
    case BASE_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

const char *base_error_name(BaseError error)
{
  /* 将内部故障码映射成蓝牙串口回复中的稳定短字符串。 */
  switch (error) {
    case BASE_ERROR_NONE: return "NONE";
    case BASE_ERROR_FEEDBACK: return "FEEDBACK_TIMEOUT";
    case BASE_ERROR_POSITION: return "POSITION_INVALID";
    case BASE_ERROR_HOME_TIMEOUT: return "HOME_TIMEOUT";
    case BASE_ERROR_RANGE: return "INVALID_RANGE";
    case BASE_ERROR_CENTER_TIMEOUT: return "CENTER_TIMEOUT";
    case BASE_ERROR_TRAVERSE_TIMEOUT: return "MOVE_TIMEOUT";
    case BASE_ERROR_CAN_TX: return "CAN_TX";
    default: return "UNKNOWN";
  }
}
