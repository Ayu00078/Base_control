#include "base_control.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static void decode(BaseFeedback *f, uint16_t angle, int16_t speed, int16_t current, uint32_t now)
{
  uint8_t data[8] = {(uint8_t)(angle >> 8), (uint8_t)angle,
                     (uint8_t)((uint16_t)speed >> 8), (uint8_t)speed,
                     (uint8_t)((uint16_t)current >> 8), (uint8_t)current, 25, 0};
  base_feedback_decode(f, data, now);
}

static void sample(BaseControl *c, BaseFeedback *f, uint32_t now,
                   int32_t position, int16_t speed, int16_t current)
{
  f->seen = 1;
  f->last_ms = now;
  f->position = position;
  f->speed = speed;
  f->filtered_speed = speed;
  f->current = current;
  base_step(c, f, now);
}

static void test_encoder(void)
{
  BaseFeedback f = {0};
  uint32_t t;
  decode(&f, 8000, 500, 3000, 0);
  decode(&f, 100, 500, -3200, 1);
  assert(f.position == 8292 && f.current == -3200 && !f.tracking_invalid);
  decode(&f, 300, 500, 123, 2);
  assert(f.position == 8492); /* Must include within-turn changes. */
  decode(&f, 8100, -500, -4000, 3);
  assert(f.position == 8100);
  for (t = 4; t < 2004; ++t) {
    int32_t expected = 8100 - (int32_t)(t - 3) * 100;
    uint16_t raw = (uint16_t)((expected % 8192 + 8192) % 8192);
    decode(&f, raw, -733, -4000, t);
    assert(f.position == expected && !f.tracking_invalid);
  }
  decode(&f, 100, 0, 0, 2003 + BASE_FEEDBACK_TIMEOUT_MS + 1U);
  assert(f.tracking_invalid && f.feedback_gap);
  base_feedback_rebase(&f);
  assert(f.position == 100 && !f.tracking_invalid && !f.feedback_gap);
  f.position = INT32_MAX - 10;
  decode(&f, 200, 0, 0, f.last_ms + 1U);
  assert(f.tracking_invalid);
  memset(&f, 0, sizeof(f));
  decode(&f, 8000, 4000, 0, 0);
  decode(&f, 5269, 4000, 0, 10);
  assert(f.position == 13461 && !f.tracking_invalid && f.max_gap_ms == 10U);
  memset(&f, 0, sizeof(f));
  decode(&f, 100, 0, 0, UINT32_MAX);
  decode(&f, 200, 0, 0, 0);
  assert(f.position == 200 && !f.tracking_invalid && base_feedback_fresh(&f, 0));
  decode(&f, 200, BASE_MAX_TRACKED_SPEED_RPM + 1, 0, 1);
  assert(f.tracking_invalid);
}

static void test_full_cycle(void)
{
  BaseControl c;
  BaseFeedback f = {0};
  uint32_t t = 0, end;
  base_init(&c);
  assert(base_start(&c, &f, 0) == -1 && c.error == BASE_ERROR_FEEDBACK);
  decode(&f, 0, 0, 0, t);
  assert(base_start(&c, &f, t) == 1);
  assert(base_start(&c, &f, t) == 0);
  /* Command saturation alone cannot establish an endpoint. */
  for (t = 1; t <= 600; ++t) sample(&c, &f, t, 30000, 0, 0);
  assert(c.state == BASE_SEEK_POSITIVE && c.output == BASE_HOME_CURRENT_LIMIT);
  /* High current with continuing motion is not an endpoint. */
  for (; t <= 900; ++t) sample(&c, &f, t, 30000, 200, 3000);
  assert(c.state == BASE_SEEK_POSITIVE);
  for (; t <= 1000; ++t) sample(&c, &f, t, 30000, 0, 3000);
  sample(&c, &f, t++, 30000, 0, 0); /* Break confirmation on a current dip. */
  end = t + BASE_HOME_CONFIRM_MS;
  for (; t < end; ++t) {
    sample(&c, &f, t, 30000, 0, 3000);
    assert(c.state == BASE_SEEK_POSITIVE);
  }
  sample(&c, &f, t++, 30000, 0, 3000);
  assert(c.state == BASE_REVERSE_PAUSE && c.output == 0 && c.positive_end == 30000);
  while (c.state == BASE_REVERSE_PAUSE) sample(&c, &f, t++, 30000, 0, 0);
  assert(c.state == BASE_SEEK_NEGATIVE);
  end = c.state_since + BASE_HOME_BLANK_MS + BASE_HOME_CONFIRM_MS;
  for (; t <= end; ++t) sample(&c, &f, t, -10000, 0, -3000);
  assert(c.state == BASE_CENTERING && c.negative_end == -10000);
  assert(c.midpoint == 10000 && c.range_valid && c.output == 0);
  sample(&c, &f, t++, 0, 0, 0);
  assert(c.output > 0 && c.output <= 16384);
  end = t + BASE_CENTER_STABLE_MS + 10;
  for (; t <= end; ++t) sample(&c, &f, t, c.midpoint, 100, 0);
  assert(c.state == BASE_CENTERING); /* Passing through midpoint is not done. */
  end = t + BASE_CENTER_STABLE_MS;
  for (; t <= end; ++t) sample(&c, &f, t, c.midpoint, 0, 0);
  assert(c.state == BASE_DONE && c.output == 0 && c.range_valid);
  sample(&c, &f, t++, 15000, 0, 0);
  assert(c.state == BASE_DONE && c.output == 0); /* No position holding. */
  assert(base_start(&c, &f, t) == 1 && !c.range_valid);
}

static void test_faults_and_time_wrap(void)
{
  BaseControl c;
  BaseFeedback f = {0};
  uint32_t t;
  base_init(&c);
  decode(&f, 0, 0, 0, 0);
  assert(base_start(&c, &f, 0) == 1);
  base_step(&c, &f, BASE_FEEDBACK_TIMEOUT_MS + 1);
  assert(c.state == BASE_FAULT && c.error == BASE_ERROR_FEEDBACK && c.output == 0);
  f.last_ms = 100;
  assert(base_start(&c, &f, 100) == 1);
  f.tracking_invalid = 1;
  base_step(&c, &f, 100);
  assert(c.error == BASE_ERROR_POSITION && !c.range_valid);
  f.feedback_gap = 1;
  assert(base_start(&c, &f, 100) == -1 && c.error == BASE_ERROR_FEEDBACK);
  f.tracking_invalid = 0;
  f.feedback_gap = 0;
  assert(base_start(&c, &f, 100) == 1);
  sample(&c, &f, 100 + BASE_HOME_TIMEOUT_MS, 0, 0, 0);
  assert(c.error == BASE_ERROR_HOME_TIMEOUT && c.output == 0);
  for (t = 0; t < 2; ++t) {
    base_init(&c);
    c.state = BASE_SEEK_NEGATIVE;
    c.positive_end = 10000;
    c.condition_active = 1;
    c.condition_since = BASE_HOME_BLANK_MS;
    sample(&c, &f, BASE_HOME_BLANK_MS + BASE_HOME_CONFIRM_MS,
           t == 0 ? 9990 : 11000, 0, -3000);
    assert(c.error == BASE_ERROR_RANGE && c.output == 0 && !c.range_valid);
    assert(c.positive_end == 0 && c.negative_end == 0 && c.midpoint == 0);
  }
  base_init(&c);
  c.state = BASE_CENTERING;
  c.range_valid = 1;
  sample(&c, &f, BASE_CENTER_TIMEOUT_MS, 0, 0, 0);
  assert(c.error == BASE_ERROR_CENTER_TIMEOUT && !c.range_valid);
  base_init(&c);
  f.last_ms = UINT32_MAX - 100;
  assert(base_start(&c, &f, f.last_ms) == 1);
  for (t = 1; t <= BASE_HOME_BLANK_MS + BASE_HOME_CONFIRM_MS; ++t)
    sample(&c, &f, (UINT32_MAX - 100) + t, 10000, 0, 3000);
  assert(c.state == BASE_REVERSE_PAUSE); /* Works through SysTick rollover. */
}

static void test_traverse_cycle(void)
{
  BaseControl c;
  BaseFeedback f = {0};
  uint32_t t;
  unsigned i;

  base_init(&c);
  f.seen = 1;
  f.last_ms = 100;
  f.position = 4500;
  assert(base_traverse_start(&c, &f, 100) == -1);

  c.negative_end = 0;
  c.positive_end = 10000;
  c.midpoint = 5000;
  c.range_valid = 1;
  assert(base_traverse_start(&c, &f, 100) == 1);
  assert(c.state == BASE_TRAVERSE_NEAR);
  assert(c.near_target == 100 && c.far_target == 9900);
  assert(c.traverse_tolerance == 25);
  assert(base_traverse_start(&c, &f, 100) == 0);
  sample(&c, &f, 101, 100000, 0, 0);
  assert(c.output < 0 && c.traverse_speed_command == -40.0f);
  for (t = 102; t <= 300; ++t) sample(&c, &f, t, 100000, 0, 0);
  assert(c.traverse_speed_command == -8000.0f);

  for (t = 301; t <= 301 + BASE_CENTER_STABLE_MS; ++t)
    sample(&c, &f, t, c.near_target, 0, 0);
  assert(c.state == BASE_TRAVERSE_WAIT && c.output == 0 && c.range_valid);
  sample(&c, &f, c.state_since + BASE_TRAVERSE_WAIT_MS - 1U, c.near_target, 0, 0);
  assert(c.state == BASE_TRAVERSE_WAIT && c.output == 0);
  sample(&c, &f, c.state_since + BASE_TRAVERSE_WAIT_MS, c.near_target, 0, 0);
  assert(c.state == BASE_TRAVERSE_FAR);
  assert(c.traverse_sample_count == 1U);
  assert(c.traverse_sample_positions[0] == 100);
  assert(c.traverse_sample_positions[BASE_TRAVERSE_SAMPLE_COUNT - 1U] == 9900);
  t = c.state_since + 1U;
  for (i = 1U; i < BASE_TRAVERSE_SAMPLE_COUNT - 1U; ++i) {
    sample(&c, &f, t++, c.traverse_sample_positions[i],
           (int16_t)(1000 + i), 0);
    assert(c.traverse_speed_samples[i] == (int16_t)(1000 + i));
  }
  sample(&c, &f, t, c.far_target, 0, 0);
  assert(c.traverse_sample_count == BASE_TRAVERSE_SAMPLE_COUNT);
  {
    uint32_t arrival = t++;
    for (; t <= arrival + BASE_CENTER_STABLE_MS; ++t)
      sample(&c, &f, t, c.far_target, 0, 0);
    assert(c.traverse_run_elapsed_ms ==
           (uint32_t)(arrival - c.traverse_run_start_ms));
  }
  assert(c.state == BASE_TRAVERSE_DONE && c.output == 0 && c.range_valid);

  base_init(&c);
  c.negative_end = -10000;
  c.positive_end = 30000;
  c.range_valid = 1;
  f.last_ms = t;
  f.position = 20000;
  assert(base_traverse_start(&c, &f, t) == 1);
  assert(c.near_target == 29600 && c.far_target == -9600);
  c.state = BASE_TRAVERSE_FAR;
  c.state_since = c.traverse_run_start_ms = t;
  for (i = 0U; i < BASE_TRAVERSE_SAMPLE_COUNT; ++i) {
    int16_t speed = i == BASE_TRAVERSE_SAMPLE_COUNT - 1U ? 0 : (int16_t)(-2000 - (int)i);
    sample(&c, &f, ++t, c.traverse_sample_positions[i], speed, 0);
    assert(c.traverse_speed_samples[i] == speed);
  }
  assert(c.traverse_sample_count == BASE_TRAVERSE_SAMPLE_COUNT);

  base_init(&c);
  c.negative_end = 0;
  c.positive_end = 10000;
  c.range_valid = 1;
  f.last_ms = 0;
  assert(base_traverse_start(&c, &f, BASE_FEEDBACK_TIMEOUT_MS + 1U) == -2);
  assert(c.state == BASE_FAULT && c.error == BASE_ERROR_FEEDBACK && !c.range_valid);
}

static void test_can_slot_and_pid(void)
{
  BaseControl c;
  uint8_t data[8];
  unsigned i;
  base_pack_current(-4000, data);
  assert(data[2] == 0xF0 && data[3] == 0x60);
  for (i = 0; i < 8; ++i) if (i != 2 && i != 3) assert(data[i] == 0);
  base_init(&c);
  assert(c.angle_pid.kp == 0.1f && c.angle_pid.ki == 0.01f && c.angle_pid.kd == 0.01f);
  assert(c.angle_pid.p_limit == 8000.0f && c.angle_pid.output_limit == 8000.0f);
  assert(c.speed_pid.kp == 25.0f && c.speed_pid.ki == 0.1f && c.speed_pid.kd == 0.01f);
  assert(c.speed_pid.output_limit == 16384.0f);
}

int main(void)
{
  test_encoder();
  test_full_cycle();
  test_faults_and_time_wrap();
  test_traverse_cycle();
  test_can_slot_and_pid();
  puts("PASS: encoder, current feedback, complete homing, spike rejection, release,");
  puts("      busy/restart, 1%-99% traverse, fault paths, timer wrap, PID preservation and CAN slot");
  return 0;
}
