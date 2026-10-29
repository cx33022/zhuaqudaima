#include "arm_vision_task.h"

#include <math.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "delay.h"
#include "board.h"
#include "Emm_V5.h"
#include "can.h"
#include "five_bar_arm.h"
#include "LED.h"
#include "swith.h"
#include "vision_link.h"
#include "arm_servos.h"
#include "arm_positions.h"
#include "arm_new_zero_measurements.h"

#define TASK_RPM                 30U
#define TASK_FINE_RPM            60U
#define TASK_SEARCH_RPM          20U
#define TASK_ACCEL              20U
#define TASK_MOVE_WAIT_FACTOR    4.0f
#define TASK_MOVE_MIN_WAIT_MS  400.0f
/* Ring-only travel wait: motor time estimate plus 80% settling margin. */
#define TASK_SEQUENCE_WAIT_FACTOR 1.8f
#define TASK_FINE_WAIT_FACTOR    3.0f
#define TASK_FINE_MIN_WAIT_MS  300.0f
#define TASK_ALIGN_MAX_DEG       5.0f
#define TASK_ALIGN_GAIN          0.8f
#define TASK_PICKUP_NEAR_PX      12
#define TASK_PICKUP_NEAR_GAIN     0.5f
#define TASK_PICKUP_NEAR_MAX_DEG  2.0f
#define TASK_PICKUP_SAMPLE_FRAMES 4U
#define TASK_PROBE_DEG           5.0f
#define TASK_PROBE_SAMPLE_FRAMES 5U
#define TASK_TARGET_CX           151
#define TASK_TARGET_CY           167
#define TASK_LEGACY_TARGET_CX    159
#define TASK_LEGACY_TARGET_CY    170
#define TASK_RING_TARGET_ID        1U
#define TASK_CENTER_PX           4
#define TASK_MAX_ALIGN_STEPS    90U
#define TASK_DETECT_TIMEOUT_MS  6000U
#define TASK_TRACK_TIMEOUT_MS    800U
#define TASK_SERVO_WAIT_MS      1000U
/* Left/right trays share a horizontal line: right -> left is +111 deg on
 * address 5 and +102.7 deg on address 6. Use only that direction ratio here;
 * the ring search does not travel to either material tray. */
#define TASK_RING_SEARCH_SIDE_5_DEG   8.0f
#define TASK_RING_SEARCH_SIDE_6_DEG   7.4f
#define TASK_RING_SEARCH_FORWARD_DEG  8.0f
#define TASK_RING_SEARCH_PASSES       4U
#define TASK_RING_SEARCH_WAIT_MS    900U
/* Measured ring 1 -> 2 offset. Ring 2 -> 3 is provisional until measured. */
#define TASK_RING_12_STEP_5_DEG  (-50.0f)
#define TASK_RING_12_STEP_6_DEG  (-66.5f)
#define TASK_RING_23_STEP_5_DEG  (-50.0f)
#define TASK_RING_23_STEP_6_DEG  (-66.5f)
#define TASK_ANY_RING_SIDE_5_DEG  10.0f
#define TASK_ANY_RING_SIDE_6_DEG  13.3f

/* PA8 is the OpenMV <-> STM32 link heartbeat.
 * Any CRC-valid frame restarts a TASK_LED_HOLD_MS window. While that window is
 * alive the lamp pulses TASK_LED_ON_MS every TASK_LED_PERIOD_MS; as soon as no
 * frame has arrived for TASK_LED_HOLD_MS it goes dark. So: blinking = the
 * serial link is working, dark = nothing is arriving. */
#define TASK_LED_HOLD_MS        200U
#define TASK_LED_ON_MS          100U
#define TASK_LED_PERIOD_MS      500U
#define TASK_LED_SERVICE_US    2000U
#define TASK_READY_PERIOD_MS    500U
#define TASK_READY_ON_MS        250U

typedef struct {
    float j00;
    float j01;
    float j10;
    float j11;
    float determinant;
} TaskJacobian;

static const ArmJointAngles task_home = {ARM_HOME_5_DEG, ARM_HOME_6_DEG};
static const ArmJointAngles task_stretch = {ARM_STRETCH_5_DEG, ARM_STRETCH_6_DEG};
static const ArmJointAngles task_trays[3] = {
    {ARM_TRAY_LEFT_5_DEG, ARM_TRAY_LEFT_6_DEG},
    {ARM_TRAY_RIGHT_5_DEG, ARM_TRAY_RIGHT_6_DEG},
    {ARM_TRAY_CENTER_5_DEG, ARM_TRAY_CENTER_6_DEG}
};

volatile uint32_t arm_vision_stage;
volatile uint32_t arm_vision_error;
volatile uint8_t arm_vision_tray_for_color[4];
volatile int16_t arm_vision_last_dx;
volatile int16_t arm_vision_last_dy;
volatile uint8_t arm_vision_centered;
volatile uint32_t arm_vision_probe_step;
volatile float arm_vision_probe_j00;
volatile float arm_vision_probe_j01;
volatile float arm_vision_probe_j10;
volatile float arm_vision_probe_j11;
volatile float arm_vision_probe_determinant;
volatile float arm_ring1_saved_5_deg;
volatile float arm_ring1_saved_6_deg;
volatile uint8_t arm_ring_initial_id;
volatile uint8_t arm_ring_poses_ready;
volatile uint8_t arm_ring_centered_mask;
volatile uint8_t arm_ring_visited_mask;
volatile float arm_ring_pose_5_deg[4];
volatile float arm_ring_pose_6_deg[4];
volatile uint8_t arm_material_pick_id;
volatile float arm_material_pick_5_deg;
volatile float arm_material_pick_6_deg;

static uint8_t task_ring_sequence_active;
static uint8_t task_material_vision_active;
static uint8_t task_material_track_any_id;
static uint8_t task_material_pickup_active;
static uint8_t task_new_transfer_active;
/* Baseline encoder positions are sampled at the manually established zero. */
static int64_t task_motor_zero_raw[2];

static uint8_t Task_ReadMotorPosition(uint8_t index, int64_t *result)
{
    uint32_t before, elapsed_us, mask;
    if (index > 1U || result == 0) { return 0U; }
    before = arm_motor_position_count[index];
    Emm_V5_Read_Sys_Params(index == 0U ? ARM_ADDR_UPPER : ARM_ADDR_LOWER, S_CPOS);
    if (!can_tx_last_cmd_ok) { return 0U; }
    for (elapsed_us = 0U; elapsed_us < 500000U; elapsed_us += 25U) {
        VisionLink_Poll();
        if (arm_motor_position_count[index] != before) {
            mask = __get_PRIMASK();
            __disable_irq();
            *result = arm_motor_position_raw[index];
            if (!mask) { __enable_irq(); }
            return 1U;
        }
        delay_us(25U);
    }
    return 0U;
}

static uint8_t Task_SyncStoppedPose(void)
{
    int64_t raw5, raw6;
    float a5, a6;
    if (!Task_ReadMotorPosition(0U, &raw5) || !Task_ReadMotorPosition(1U, &raw6)) {
        Arm_ClearReference();
        return 0U;
    }
    a5 = (float)(raw5 - task_motor_zero_raw[0]) * (360.0f / 65536.0f)
         / ARM_REDUCTION_RATIO * (ARM_UPPER_POSITIVE_DIR == 0U ? 1.0f : -1.0f);
    a6 = (float)(raw6 - task_motor_zero_raw[1]) * (360.0f / 65536.0f)
         / ARM_REDUCTION_RATIO * (ARM_LOWER_POSITIVE_DIR == 0U ? 1.0f : -1.0f);
    return Arm_SetCurrentJointAngles(a5, a6) == ARM_STATUS_OK;
}

static uint32_t task_led_frames_seen;
static uint32_t task_led_hold_ms;
static uint32_t task_led_phase_ms;
static uint32_t task_ready_phase_ms;

/* Called from every polling loop with the time it just consumed, so PA8 stays
 * live while the arm moves and while it waits for frames. */
static void Task_LedService(uint16_t elapsed_ms)
{
    uint32_t frames = vision_link_valid_frames;

    if (task_material_pickup_active) {
        if (arm_vision_centered) {
            task_ready_phase_ms += (uint32_t)elapsed_ms;
            task_ready_phase_ms %= TASK_READY_PERIOD_MS;
            GPIO_WriteBit(GPIOG, GPIO_Pin_15,
                          task_ready_phase_ms < TASK_READY_ON_MS
                          ? Bit_SET : Bit_RESET);
        } else {
            task_ready_phase_ms = 0U;
            GPIO_ResetBits(GPIOG, GPIO_Pin_15);
        }
    }

    if (frames != task_led_frames_seen) {
        task_led_frames_seen = frames;
        task_led_hold_ms = TASK_LED_HOLD_MS;
    } else if (task_led_hold_ms > (uint32_t)elapsed_ms) {
        task_led_hold_ms -= (uint32_t)elapsed_ms;
    } else {
        task_led_hold_ms = 0U;
    }

    if (task_led_hold_ms == 0U) {          /* link down: lamp dark */
        task_led_phase_ms = 0U;
        LED_Set2(0U);
        return;
    }

    /* Link up: one short pulse per TASK_LED_PERIOD_MS. */
    task_led_phase_ms += (uint32_t)elapsed_ms;
    if (task_led_phase_ms >= TASK_LED_PERIOD_MS) {
        task_led_phase_ms %= TASK_LED_PERIOD_MS;
    }
    LED_Set2((task_led_phase_ms < TASK_LED_ON_MS) ? 1U : 0U);
}

static void Task_LedCentered(uint8_t centered)
{
    arm_vision_centered = centered ? 1U : 0U;
    if (task_material_pickup_active) {
        arm_vision_stage = centered ? 5U : 4U;
    }
}

static uint8_t Task_MoveWithWait(ArmJointAngles target,
                                 uint16_t rpm, float wait_factor,
                                 float min_wait_ms)
{
    ArmJointAngles start;
    float largest, wait_ms;
    uint32_t remaining_ms;

    if (Arm_GetCurrentJointAngles(&start) != ARM_STATUS_OK) { return 0U; }
    largest = fmaxf(fabsf(target.upper_deg - start.upper_deg),
                    fabsf(target.lower_deg - start.lower_deg));
    if (largest < 0.001f) { return 1U; }
    if (!task_material_pickup_active && swith_out() == 0U) {
        Arm_Stop(); return 0U;
    }
    if (Arm_MoveJoint(target.upper_deg, target.lower_deg, rpm, TASK_ACCEL)
        != ARM_STATUS_OK) { return 0U; }

    wait_ms = largest * ARM_PULSES_PER_OUTPUT_DEG * 60000.0f
              / ((float)rpm * ARM_MOTOR_PULSES_PER_REV) * wait_factor;
    remaining_ms = (uint32_t)ceilf(fmaxf(wait_ms, min_wait_ms));
    while (remaining_ms > 0U) {
        uint16_t interval_ms = (remaining_ms > 20U) ? 20U : (uint16_t)remaining_ms;
        if (!task_material_pickup_active && swith_out() == 0U) {
            Arm_Stop(); return 0U;
        }
        delay_ms(interval_ms);
        remaining_ms -= interval_ms;
        /* Keep the USART drained while the arm is moving: USART1 has no receive
         * FIFO, and parsing here is what makes PA8 show frame arrival during the
         * stretch move instead of only after the first vision step starts. */
        VisionLink_Poll();
        Task_LedService(interval_ms);
    }
    return 1U;
}

static uint8_t Task_Move(ArmJointAngles target)
{
    return Task_MoveWithWait(target, TASK_RPM, TASK_MOVE_WAIT_FACTOR,
                             TASK_MOVE_MIN_WAIT_MS);
}

static uint8_t Task_MoveSequence(ArmJointAngles target)
{
    return Task_MoveWithWait(target, TASK_RPM, TASK_SEQUENCE_WAIT_FACTOR,
                             TASK_MOVE_MIN_WAIT_MS);
}

static uint8_t Task_MoveFine(ArmJointAngles target)
{
    return Task_MoveWithWait(target, TASK_FINE_RPM, TASK_FINE_WAIT_FACTOR,
                             TASK_FINE_MIN_WAIT_MS);
}

static uint8_t Task_SelectTarget(uint8_t mode, uint8_t selector)
{
    uint8_t expected = VisionLink_SetTarget(mode, selector);
    uint8_t sequence, result;
    uint32_t elapsed_us;

    /* USART1 has no receive FIFO: at 115200 baud a new byte arrives about
     * every 87 us. Poll faster than that for the whole reply frame. */
    for (elapsed_us = 0U; elapsed_us < 1500000U; elapsed_us += 25U) {
        VisionLink_Poll();
        if (VisionLink_TakeTargetAck(&sequence, &result)
            && sequence == expected) { return result == 0U; }
        delay_us(25U);
    }
    return 0U;
}

static uint8_t Task_WaitDetection(uint8_t mode, uint8_t id,
                                  uint16_t timeout_ms, uint8_t frames,
                                  VisionDetection *result)
{
    VisionDetection candidate;
    uint32_t elapsed_us;
    uint32_t led_us = 0U;
    uint8_t stable = 0U;
    uint8_t last_id = 0U;
    int32_t sum_dx = 0;
    int32_t sum_dy = 0;
    uint32_t sum_cx = 0U;
    uint32_t sum_cy = 0U;
    uint32_t sum_size = 0U;
    uint32_t sum_quality = 0U;

    VisionLink_ClearEvents();
    for (elapsed_us = 0U; elapsed_us < (uint32_t)timeout_ms * 1000U;
         elapsed_us += 25U) {
        VisionLink_Poll();
        if (VisionLink_TakeDetection(&candidate)) {
            if (candidate.mode == mode && candidate.found == 1U
                && candidate.id >= 1U && candidate.id <= 3U
                && (id == 0U || candidate.id == id)) {
                if (candidate.id == last_id) {
                    ++stable;
                    sum_dx += candidate.dx;
                    sum_dy += candidate.dy;
                    sum_cx += candidate.cx;
                    sum_cy += candidate.cy;
                    sum_size += candidate.size;
                    sum_quality += candidate.quality;
                } else {
                    stable = 1U;
                    sum_dx = candidate.dx;
                    sum_dy = candidate.dy;
                    sum_cx = candidate.cx;
                    sum_cy = candidate.cy;
                    sum_size = candidate.size;
                    sum_quality = candidate.quality;
                }
                last_id = candidate.id;
                if (stable >= frames) {
                    /* Average the same consecutive frames that establish a
                     * stable ID. This keeps ring-centre jitter out of the
                     * finite-difference probe and also smooths tracking. */
                    candidate.dx = (int16_t)(sum_dx / (int32_t)stable);
                    candidate.dy = (int16_t)(sum_dy / (int32_t)stable);
                    candidate.cx = (uint16_t)(sum_cx / stable);
                    candidate.cy = (uint16_t)(sum_cy / stable);
                    candidate.size = (uint16_t)(sum_size / stable);
                    candidate.quality = (uint8_t)(sum_quality / stable);
                    /* Use absolute pixels so the STM32 target does not depend
                     * on the OpenMV preview's hand_x/y setting. */
                    if ((mode == VISION_MODE_RING
                         && (task_ring_sequence_active || id == TASK_RING_TARGET_ID))
                        || (mode == VISION_MODE_GRAB && task_material_vision_active)) {
                        candidate.dx = (int16_t)candidate.cx - TASK_TARGET_CX;
                        candidate.dy = (int16_t)candidate.cy - TASK_TARGET_CY;
                    } else if (id == 2U
                               && (mode == VISION_MODE_RING || mode == VISION_MODE_GRAB)) {
                        candidate.dx = (int16_t)candidate.cx - TASK_LEGACY_TARGET_CX;
                        candidate.dy = (int16_t)candidate.cy - TASK_LEGACY_TARGET_CY;
                    }
                    arm_vision_last_dx = candidate.dx;
                    arm_vision_last_dy = candidate.dy;
                    *result = candidate;
                    return 1U;
                }
            } else {
                stable = 0U;
                last_id = 0U;
                sum_dx = sum_dy = 0;
                sum_cx = sum_cy = sum_size = sum_quality = 0U;
            }
        }
        delay_us(25U);
        led_us += 25U;
        if (led_us >= TASK_LED_SERVICE_US) {
            led_us = 0U;
            Task_LedService((uint16_t)(TASK_LED_SERVICE_US / 1000U));
        }
    }
    return 0U;
}

static uint8_t Task_Probe(uint8_t mode, uint8_t id, TaskJacobian *jacobian)
{
    ArmJointAngles base, probe;
    VisionDetection start, after_upper, returned, after_lower;
    float probe_deg = task_new_transfer_active ? 2.0f : TASK_PROBE_DEG;

    arm_vision_probe_step = 1U;
    arm_vision_probe_j00 = arm_vision_probe_j01 = 0.0f;
    arm_vision_probe_j10 = arm_vision_probe_j11 = 0.0f;
    arm_vision_probe_determinant = 0.0f;
    if (!Task_WaitDetection(mode, id, TASK_DETECT_TIMEOUT_MS,
                            TASK_PROBE_SAMPLE_FRAMES, &start)) { return 0U; }
    if (Arm_GetCurrentJointAngles(&base) != ARM_STATUS_OK) { return 0U; }

    probe = base;
    probe.upper_deg += probe_deg;
    arm_vision_probe_step = 2U;
    if (!Task_MoveFine(probe)) { return 0U; }
    arm_vision_probe_step = 3U;
    if (!Task_WaitDetection(mode, id, TASK_DETECT_TIMEOUT_MS,
                            TASK_PROBE_SAMPLE_FRAMES, &after_upper)) {
        (void)Task_MoveFine(base);
        return 0U;
    }
    arm_vision_probe_step = 4U;
    if (!Task_MoveFine(base)) { return 0U; }
    arm_vision_probe_step = 5U;
    if (!Task_WaitDetection(mode, id, TASK_DETECT_TIMEOUT_MS,
                            TASK_PROBE_SAMPLE_FRAMES, &returned)) { return 0U; }

    probe = base;
    probe.lower_deg += probe_deg;
    arm_vision_probe_step = 6U;
    if (!Task_MoveFine(probe)) { return 0U; }
    arm_vision_probe_step = 7U;
    if (!Task_WaitDetection(mode, id, TASK_DETECT_TIMEOUT_MS,
                            TASK_PROBE_SAMPLE_FRAMES, &after_lower)) {
        (void)Task_MoveFine(base);
        return 0U;
    }
    arm_vision_probe_step = 8U;
    if (!Task_MoveFine(base)) { return 0U; }

    arm_vision_probe_step = 9U;
    jacobian->j00 = (float)(after_upper.dx - start.dx) / probe_deg;
    jacobian->j10 = (float)(after_upper.dy - start.dy) / probe_deg;
    jacobian->j01 = (float)(after_lower.dx - returned.dx) / probe_deg;
    jacobian->j11 = (float)(after_lower.dy - returned.dy) / probe_deg;
    jacobian->determinant = jacobian->j00 * jacobian->j11
                            - jacobian->j01 * jacobian->j10;
    arm_vision_probe_j00 = jacobian->j00;
    arm_vision_probe_j01 = jacobian->j01;
    arm_vision_probe_j10 = jacobian->j10;
    arm_vision_probe_j11 = jacobian->j11;
    arm_vision_probe_determinant = jacobian->determinant;
    if (fabsf(jacobian->determinant) < 0.5f) { return 0U; }
    arm_vision_probe_step = 10U;
    return 1U;
}

static float Task_Clamp(float value, float limit)
{
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}

static uint8_t Task_Align(uint8_t mode, uint8_t id, uint8_t continuous,
                          uint8_t select_remote)
{
    TaskJacobian jacobian;
    VisionDetection detection, reacquired;
    ArmJointAngles current, target;
    uint32_t frames_before = vision_link_valid_frames;
    uint32_t aborts_before;
    uint8_t centered = 0U;
    uint8_t tracking_id = id;
    uint8_t last_tracking_id = id;
    uint8_t pickup_press_armed = 0U;
    uint32_t i;

    Task_LedCentered(0U);

    /* The current ring transfer filters incoming AUTO detections locally. Only
     * the older full-task path asks OpenMV to switch modes over UART. */
    if (select_remote && !Task_SelectTarget(mode, id)) {
        arm_vision_error = 4U;
        return 0U;
    }
    for (;;) {
        aborts_before = arm_can_batch_abort_count;
        if (Task_Probe(mode, id, &jacobian)) { break; }
        /* Only the pickup flow retries a probe interrupted by a lost block.
         * Task_Probe has returned to its base pose at steps 3, 5 and 7. */
        if (!task_material_pickup_active
            || arm_can_batch_abort_count != aborts_before
            || (arm_vision_probe_step != 1U
                && arm_vision_probe_step != 3U
                && arm_vision_probe_step != 5U
                && arm_vision_probe_step != 7U)) {
            arm_vision_error = (vision_link_valid_frames == frames_before) ? 4U : 5U;
            return 0U;
        }
        Task_LedCentered(0U);
        while (!Task_WaitDetection(mode, 0U, 1000U, 2U, &reacquired)) {}
        id = reacquired.id;
        tracking_id = id;
        last_tracking_id = id;
        arm_material_pick_id = id;
    }
    /* Probe one color consistently, then follow whichever material is
     * currently visible. The camera/joint mapping is color-independent. */
    if (mode == VISION_MODE_GRAB && task_material_track_any_id) {
        tracking_id = 0U;
    }

    for (i = 0U; continuous || i < TASK_MAX_ALIGN_STEPS; ++i) {
        float delta_upper, delta_lower, max_step_deg, gain, error_x, error_y;
        if (continuous && !task_material_pickup_active
            && swith_out() == 0U) { Arm_Stop(); return 1U; }
        if (!Task_WaitDetection(mode, tracking_id,
                                continuous ? TASK_TRACK_TIMEOUT_MS
                                           : TASK_DETECT_TIMEOUT_MS,
                                task_material_pickup_active
                                ? TASK_PICKUP_SAMPLE_FRAMES : 2U,
                                &detection)) {
            if (continuous) {
                centered = 0U;
                pickup_press_armed = 0U;
                Task_LedCentered(0U);
                continue;
            }
            arm_vision_error = 6U;
            return 0U;
        }
        if (task_material_track_any_id && mode == VISION_MODE_GRAB) {
            if (detection.id != last_tracking_id) {
                centered = 0U;
                pickup_press_armed = 0U;
            }
            last_tracking_id = detection.id;
            arm_material_pick_id = detection.id;
        }
        if (abs(detection.dx) <= TASK_CENTER_PX
            && abs(detection.dy) <= TASK_CENTER_PX) {
            if (continuous && task_material_pickup_active) {
                if (centered < 3U) { ++centered; }
                if (centered >= 3U) {
                    Task_LedCentered(1U);
                    if (swith_out() != 0U) {
                        pickup_press_armed = 1U;
                    } else if (pickup_press_armed) {
                        delay_ms(30U);
                        if (swith_out() == 0U) { return 1U; }
                    }
                } else {
                    Task_LedCentered(0U);
                }
                continue;
            }
            Task_LedCentered(1U);
            if (continuous) { continue; }
            if (++centered >= 3U) { return 1U; }
            continue;
        }
        Task_LedCentered(0U);
        centered = 0U;
        pickup_press_armed = 0U;
        error_x = (float)detection.dx;
        error_y = (float)detection.dy;
        /* Bring the nearest edge back into view before full 2D alignment. */
        if (task_new_transfer_active && mode == VISION_MODE_RING) {
            if (detection.cx < 55U || detection.cx > 265U) { error_y = 0.0f; }
            else if (detection.cy < 45U || detection.cy > 205U) { error_x = 0.0f; }
        }
        delta_upper = (-jacobian.j11 * error_x
                       + jacobian.j01 * error_y) / jacobian.determinant;
        delta_lower = (jacobian.j10 * error_x
                       - jacobian.j00 * error_y) / jacobian.determinant;
        max_step_deg = TASK_ALIGN_MAX_DEG;
        gain = TASK_ALIGN_GAIN;
        if (task_material_pickup_active && mode == VISION_MODE_GRAB
            && abs(detection.dx) <= TASK_PICKUP_NEAR_PX
            && abs(detection.dy) <= TASK_PICKUP_NEAR_PX) {
            /* A smaller last step avoids chasing blob-centre jitter around
             * the four-pixel pickup window. */
            max_step_deg = TASK_PICKUP_NEAR_MAX_DEG;
            gain = TASK_PICKUP_NEAR_GAIN;
        }
        delta_upper = Task_Clamp(delta_upper * gain, max_step_deg);
        delta_lower = Task_Clamp(delta_lower * gain, max_step_deg);
        if (fabsf(delta_upper) + fabsf(delta_lower) < 0.05f) {
            if (continuous) { continue; }
            arm_vision_error = 7U;
            return 0U;
        }
        if (Arm_GetCurrentJointAngles(&current) != ARM_STATUS_OK) {
            arm_vision_error = 7U;
            return 0U;
        }
        target.upper_deg = current.upper_deg + delta_upper;
        target.lower_deg = current.lower_deg + delta_lower;
        if (!Task_MoveFine(target)) {
            if (continuous && !task_material_pickup_active
                && swith_out() == 0U) { return 1U; }
            arm_vision_error = 7U;
            return 0U;
        }
    }
    arm_vision_error = 7U;
    return 0U;
}

static uint8_t Task_ScanTrays(void)
{
    VisionDetection detection;
    uint8_t tray, color, identified;

    for (tray = 0U; tray < 3U; ++tray) {
        arm_vision_stage = 10U + tray;
        if (!Task_SelectTarget(VISION_MODE_IDLE, 0U)
            || !Task_Move(task_trays[tray])) { return 0U; }
        identified = 0U;
        for (color = 1U; color <= 3U; ++color) {
            if (arm_vision_tray_for_color[color]) { continue; }
            if (!Task_SelectTarget(VISION_MODE_GRAB, color)) { return 0U; }
            if (Task_WaitDetection(VISION_MODE_GRAB, color, 1500U, 2U, &detection)
                && abs(detection.dx) <= 25 && abs(detection.dy) <= 25) {
                arm_vision_tray_for_color[color] = tray + 1U;
                identified = 1U;
                break;
            }
        }
        if (!identified) { return 0U; }
        if (!Task_Move(task_stretch)) { return 0U; }
    }
    return arm_vision_tray_for_color[1] && arm_vision_tray_for_color[2]
           && arm_vision_tray_for_color[3];
}

static uint8_t Task_Pick(uint8_t color)
{
    uint8_t tray = arm_vision_tray_for_color[color] - 1U;
    if (!Task_SelectTarget(VISION_MODE_IDLE, 0U)
        || !Task_Move(task_trays[tray])
        || !Task_Align(VISION_MODE_GRAB, color, 0U, 1U)) { return 0U; }

    ArmServos_Lift(ARM_PICKUP_DOWN_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_Claw(ARM_CLAW_CLOSE_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_Lift(ARM_LIFT_UP_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    return Task_Move(task_stretch);
}

static uint8_t Task_Place(uint8_t number)
{
    if (!Task_Align(VISION_MODE_RING, number, 0U, 1U)) { return 0U; }
    ArmServos_Lift(ARM_PLACE_DOWN_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_Claw(ARM_CLAW_OPEN_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_Lift(ARM_LIFT_UP_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    return Task_Move(task_stretch);
}

static void Task_Error(uint32_t error)
{
    uint32_t i;
    arm_vision_error = error;
    if (task_material_pickup_active) {
        task_material_pickup_active = 0U;
        GPIO_ResetBits(GPIOG, GPIO_Pin_15);
    }
    if (board_can_init_ok) { Arm_Stop(); }
    while (1) {
        for (i = 0U; i < error; ++i) {
            LED_Set2(1U);
            delay_ms(150U);
            LED_Set2(0U);
            delay_ms(200U);
        }
        delay_ms(1000U);
    }
}

static void Task_WaitForPress(void)
{
    /* Keep the link drained while waiting for PC9: PA8 then shows the OpenMV
     * heartbeat before anything moves, so the wiring can be checked with no
     * motion at all. */
    while (swith_out() == 0U) { delay_ms(10U); VisionLink_Poll(); Task_LedService(10U); }
    while (swith_out() != 0U) { delay_ms(10U); VisionLink_Poll(); Task_LedService(10U); }
    delay_ms(30U);
    while (swith_out() == 0U) { delay_ms(10U); VisionLink_Poll(); Task_LedService(10U); }
    delay_ms(30U);
}

void ArmVisionTask_Run(void)
{
    uint8_t color;

    arm_vision_stage = 1U;
    board_init();
    Swith_Init();
    LED2_Init();
    if (!board_can_init_ok) { Task_Error(1U); }
    VisionLink_Init();
    if (ARM_PLACE_DOWN_US != 0U) { ArmServos_Init(); }
    LED_Set2(1U);
    delay_ms(120U);
    LED_Set2(0U);
    delay_ms(120U);
    LED_Set2(1U);
    delay_ms(120U);
    LED_Set2(0U);
    delay_ms(2000U);
    Emm_V5_En_Control(ARM_ADDR_UPPER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    delay_ms(20U);
    Emm_V5_En_Control(ARM_ADDR_LOWER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    Arm_SetCurrentJointAngles(ARM_HOME_5_DEG, ARM_HOME_6_DEG);

    arm_vision_stage = 2U;
    Task_WaitForPress();
    /* From here PA8 is owned by Task_LedService: solid while CRC-valid OpenMV
     * frames keep arriving, blinking once the target centre is in tolerance. */
    LED_Set2(0U);
    if (ARM_PLACE_DOWN_US != 0U) {
        ArmServos_Lift(ARM_LIFT_UP_US);
        ArmServos_Claw(ARM_CLAW_OPEN_US);
        delay_ms(TASK_SERVO_WAIT_MS);
    }
    if (ARM_PLACE_DOWN_US == 0U) {
        /* The manually positioned power-on pose is the local (0, 0) reference.
         * Do not make the old -152/+152 degree stretch move: learn the two
         * pixel/joint directions here, then keep ring 2 at (160, 120). */
        arm_vision_stage = 22U;
        if (!Task_Align(VISION_MODE_RING, 2U, 1U, 0U)) { Task_Error(arm_vision_error); }
    } else {
        arm_vision_stage = 3U;
        if (!Task_Move(task_stretch)) { Task_Error(3U); }
        if (!Task_ScanTrays()) { Task_Error(4U); }
        for (color = 1U; color <= 3U; ++color) {
            arm_vision_stage = 30U + color;
            if (!Task_Pick(color)) { Task_Error(6U); }
            arm_vision_stage = 40U + color;
            if (!Task_Place(color)) { Task_Error(7U); }
        }
    }

    if (ARM_PLACE_DOWN_US != 0U) {
        arm_vision_stage = 90U;
        if (!Task_Move(task_home)) { Task_Error(8U); }
        Task_SelectTarget(VISION_MODE_IDLE, 0U);
    }
    arm_vision_stage = 100U;
    LED_Set2(0U);
    while (1) { delay_ms(1000U); }
}

/* Ring 1 is left of ring 2 in the placement area. Increasing both joint
 * setpoints follows the measured right-to-left tray direction, without
 * travelling toward a tray pose. */
static uint8_t Task_FindRing1(void)
{
    ArmJointAngles base, candidate;
    VisionDetection seen;
    uint8_t forward, side;

    for (forward = 0U; forward < TASK_RING_SEARCH_PASSES; ++forward) {
        base.upper_deg = task_stretch.upper_deg
                         + (float)forward * TASK_RING_SEARCH_FORWARD_DEG;
        base.lower_deg = task_stretch.lower_deg
                         - (float)forward * TASK_RING_SEARCH_FORWARD_DEG;
        for (side = 0U; side <= forward + 1U; ++side) {
            candidate.upper_deg = base.upper_deg
                                  + (float)side * TASK_RING_SEARCH_SIDE_5_DEG;
            candidate.lower_deg = base.lower_deg
                                  + (float)side * TASK_RING_SEARCH_SIDE_6_DEG;
            if (!Task_MoveFine(candidate)) { return 0U; }
            if (Task_WaitDetection(VISION_MODE_RING, TASK_RING_TARGET_ID,
                                   TASK_RING_SEARCH_WAIT_MS, 3U, &seen)) {
                return 1U;
            }
        }
    }
    return 0U;
}

/* Search both sides of the transit pose for a stable ring or material ID.
 * The measured forward direction is address 5 negative / address 6 positive. */
static uint8_t Task_FindAnyTarget(uint8_t mode, uint8_t *target_id)
{
    ArmJointAngles base, candidate;
    VisionDetection seen;
    uint8_t forward, side, direction;
    float sign;

    /* Two matching frames are enough to start the deliberate 5-degree probe;
     * allow a little longer at the known transit pose before moving away. */
    if (Task_WaitDetection(mode, 0U, 1500U, 2U, &seen)) {
        *target_id = seen.id;
        return 1U;
    }
    for (forward = 0U; forward < TASK_RING_SEARCH_PASSES; ++forward) {
        base.upper_deg = task_stretch.upper_deg
                         - (float)forward * TASK_RING_SEARCH_FORWARD_DEG;
        base.lower_deg = task_stretch.lower_deg
                         + (float)forward * TASK_RING_SEARCH_FORWARD_DEG;
        if (forward != 0U) {
            if (!Task_MoveFine(base)) { return 0U; }
            if (Task_WaitDetection(mode, 0U,
                                   TASK_RING_SEARCH_WAIT_MS, 3U, &seen)) {
                *target_id = seen.id;
                return 1U;
            }
        }
        for (side = 1U; side <= forward + 1U; ++side) {
            for (direction = 0U; direction < 2U; ++direction) {
                sign = direction == 0U ? 1.0f : -1.0f;
                candidate.upper_deg = base.upper_deg
                                      + sign * (float)side * TASK_ANY_RING_SIDE_5_DEG;
                candidate.lower_deg = base.lower_deg
                                      + sign * (float)side * TASK_ANY_RING_SIDE_6_DEG;
                if (!Task_MoveFine(candidate)) { return 0U; }
                if (Task_WaitDetection(mode, 0U,
                                       TASK_RING_SEARCH_WAIT_MS, 3U, &seen)) {
                    *target_id = seen.id;
                    return 1U;
                }
            }
        }
    }
    return 0U;
}

static void Task_WaitSecondPressBlink(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t on = 1U;
    uint8_t ticks = 0U;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG, &gpio);
    GPIO_SetBits(GPIOG, GPIO_Pin_15);

    /* The stage is complete before this wait; an already-held PC9 is ignored. */
    while (swith_out() == 0U) { delay_ms(10U); }
    while (1) {
        if (swith_out() == 0U) {
            delay_ms(30U);
            if (swith_out() == 0U) { break; }
        }
        delay_ms(10U);
        if (++ticks >= 25U) {
            ticks = 0U;
            on = on ? 0U : 1U;
            GPIO_WriteBit(GPIOG, GPIO_Pin_15, on ? Bit_SET : Bit_RESET);
        }
    }
    while (swith_out() == 0U) { delay_ms(10U); }
    delay_ms(30U);
    GPIO_ResetBits(GPIOG, GPIO_Pin_15);
}

/* Search moves are monitored byte-by-byte; two matching ring frames interrupt
 * the move, then actual motor positions replace the unfinished setpoints. */
static int Task_SearchMove(ArmJointAngles target, uint8_t *id)
{
    ArmJointAngles start;
    VisionDetection d;
    float largest, wait_ms;
    uint32_t elapsed_us, limit_us, led_us = 0U;
    uint8_t last_id = 0U, hits = 0U;
    if (Arm_GetCurrentJointAngles(&start) != ARM_STATUS_OK) { return -1; }
    largest = fmaxf(fabsf(target.upper_deg - start.upper_deg),
                    fabsf(target.lower_deg - start.lower_deg));
    if (largest < 0.001f) { return 0; }
    VisionLink_ClearEvents();
    if (Arm_MoveJoint(target.upper_deg, target.lower_deg, TASK_SEARCH_RPM, TASK_ACCEL)
        != ARM_STATUS_OK) { return -1; }
    wait_ms = largest * ARM_PULSES_PER_OUTPUT_DEG * 60000.0f /
              ((float)TASK_SEARCH_RPM * ARM_MOTOR_PULSES_PER_REV) * TASK_FINE_WAIT_FACTOR;
    limit_us = (uint32_t)(fmaxf(wait_ms, TASK_FINE_MIN_WAIT_MS) * 1000.0f);
    for (elapsed_us = 0U; elapsed_us < limit_us; elapsed_us += 25U) {
        VisionLink_Poll();
        if (VisionLink_TakeDetection(&d)) {
            if (d.mode == VISION_MODE_RING && d.found && d.id >= 1U && d.id <= 3U) {
                hits = d.id == last_id ? (uint8_t)(hits + 1U) : 1U;
                last_id = d.id;
                if (hits >= 2U) {
                    if (Arm_Stop() != ARM_STATUS_OK) { Arm_ClearReference(); return -1; }
                    delay_ms(300U);
                    if (!Task_SyncStoppedPose()) { return -1; }
                    {
                        ArmJointAngles actual;
                        if (Arm_GetCurrentJointAngles(&actual) != ARM_STATUS_OK ||
                            actual.upper_deg < fminf(start.upper_deg, target.upper_deg) - 1.0f ||
                            actual.upper_deg > fmaxf(start.upper_deg, target.upper_deg) + 1.0f ||
                            actual.lower_deg < fminf(start.lower_deg, target.lower_deg) - 1.0f ||
                            actual.lower_deg > fmaxf(start.lower_deg, target.lower_deg) + 1.0f) {
                            Arm_ClearReference(); return -1;
                        }
                    }
                    *id = d.id;
                    return 1;
                }
            } else { hits = last_id = 0U; }
        }
        delay_us(25U);
        led_us += 25U;
        if (led_us >= TASK_LED_SERVICE_US) {
            led_us = 0U;
            Task_LedService((uint16_t)(TASK_LED_SERVICE_US / 1000U));
        }
    }
    return 0;
}

static uint8_t Task_SearchNewRing(uint8_t *id)
{
    const ArmJointAngles shared = {NEW_ZERO_SHARED_5_DEG, NEW_ZERO_SHARED_6_DEG};
    ArmJointAngles p;
    VisionDetection d;
    uint8_t layer, side;
    int found;
    if (Task_WaitDetection(VISION_MODE_RING, 0U, 800U, 2U, &d)) {
        *id = d.id; return 1U;
    }
    for (layer = 0U; layer < 4U; ++layer) {
        p.upper_deg = shared.upper_deg - 5.0f * (layer + 1U);
        p.lower_deg = shared.lower_deg + 5.0f * (layer + 1U);
        found = Task_SearchMove(p, id);
        if (found < 0) { arm_vision_error = 9U; return 0U; }
        if (found > 0) { return 1U; }
        for (side = 1U; side <= 2U; ++side) {
            p.upper_deg = shared.upper_deg - 5.0f * (layer + 1U) + 8.0f * side;
            p.lower_deg = shared.lower_deg + 5.0f * (layer + 1U) + 12.7f * side;
            found = Task_SearchMove(p, id);
            if (found < 0) { arm_vision_error = 9U; return 0U; }
            if (found > 0) { return 1U; }
            p.upper_deg = shared.upper_deg - 5.0f * (layer + 1U) - 8.0f * side;
            p.lower_deg = shared.lower_deg + 5.0f * (layer + 1U) - 12.7f * side;
            found = Task_SearchMove(p, id);
            if (found < 0) { arm_vision_error = 9U; return 0U; }
            if (found > 0) { return 1U; }
        }
    }
    return 0U;
}

void ArmVisionTask_RunNewRingPickup(void)
{
    const ArmJointAngles shared = {NEW_ZERO_SHARED_5_DEG, NEW_ZERO_SHARED_6_DEG};
    const ArmJointAngles tray[2] = {
        {NEW_ZERO_REAR_TRAY_5_DEG, NEW_ZERO_REAR_TRAY_6_DEG},
        {NEW_ZERO_LEFT_TRAY_5_DEG, NEW_ZERO_LEFT_TRAY_6_DEG}
    };
    const ArmJointAngles measured[4] = {
        {0.0f, 0.0f}, {NEW_ZERO_RING_1_5_DEG, NEW_ZERO_RING_1_6_DEG},
        {NEW_ZERO_RING_2_5_DEG, NEW_ZERO_RING_2_6_DEG},
        {NEW_ZERO_RING_3_5_DEG, NEW_ZERO_RING_3_6_DEG}
    };
    ArmJointAngles anchor, poses[4];
    uint8_t id, n;
    uint16_t lift = ARM_BOOT_LIFT_US, claw = ARM_BOOT_CLAW_US;
    task_new_transfer_active = task_ring_sequence_active = 1U;
    arm_vision_error = 0U;
    arm_vision_stage = 1U;
    board_init(); Swith_Init(); LED2_Init();
    if (!board_can_init_ok) { Task_Error(1U); }
    VisionLink_Init();
    ArmServos_Init();
    ArmServos_Lift(lift); ArmServos_Claw(claw);
    ArmServos_LiftSlow(&lift, ARM_LIFT_UP_US);
    ArmServos_ClawSlow(&claw, ARM_CLAW_OPEN_US);
    delay_ms(2000U);
    Emm_V5_En_Control(ARM_ADDR_UPPER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    delay_ms(20U);
    Emm_V5_En_Control(ARM_ADDR_LOWER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    if (!Task_ReadMotorPosition(0U, &task_motor_zero_raw[0]) ||
        !Task_ReadMotorPosition(1U, &task_motor_zero_raw[1])) { Task_Error(4U); }
    Arm_SetCurrentJointAngles(0.0f, 0.0f);
    while (swith_out() == 0U) { delay_ms(10U); }
    arm_vision_stage = 2U;
    if (!Task_MoveSequence(shared)) { Task_Error(3U); }
    arm_vision_stage = 3U;
    if (!Task_SearchNewRing(&id)) {
        Task_Error(arm_vision_error ? arm_vision_error : 5U);
    }
    arm_ring_initial_id = id;
    arm_vision_stage = 4U;
    if (!Task_Align(VISION_MODE_RING, id, 0U, 0U)) { Task_Error(arm_vision_error); }
    if (Arm_GetCurrentJointAngles(&anchor) != ARM_STATUS_OK) { Task_Error(7U); }
    for (n = 1U; n <= 3U; ++n) {
        poses[n].upper_deg = anchor.upper_deg + measured[n].upper_deg - measured[id].upper_deg;
        poses[n].lower_deg = anchor.lower_deg + measured[n].lower_deg - measured[id].lower_deg;
        arm_ring_pose_5_deg[n] = poses[n].upper_deg;
        arm_ring_pose_6_deg[n] = poses[n].lower_deg;
    }
    arm_ring_poses_ready = 1U;
    arm_vision_stage = 5U;
    Task_WaitSecondPressBlink();
    for (n = 0U; n < 2U; ++n) {
        arm_vision_stage = 6U + n * 3U;
        if (!Task_MoveSequence(tray[n])) { Task_Error(7U); }
        ArmServos_ClawSlow(&claw, ARM_CLAW_OPEN_US);
        ArmServos_LiftSlow(&lift, ARM_TRAY_HEIGHT_US);
        delay_ms(TASK_SERVO_WAIT_MS);
        ArmServos_ClawSlow(&claw, ARM_CLAW_CLOSE_US);
        delay_ms(TASK_SERVO_WAIT_MS);
        ArmServos_LiftSlow(&lift, ARM_LIFT_UP_US);
        delay_ms(TASK_SERVO_WAIT_MS);
        arm_vision_stage = 7U + n * 3U;
        if (!Task_MoveSequence(poses[n + 1U])) { Task_Error(7U); }
        ArmServos_LiftSlow(&lift, ARM_GROUND_HEIGHT_US);
        delay_ms(TASK_SERVO_WAIT_MS);
        ArmServos_ClawSlow(&claw, ARM_CLAW_OPEN_US);
        delay_ms(TASK_SERVO_WAIT_MS);
        ArmServos_LiftSlow(&lift, ARM_LIFT_UP_US);
        delay_ms(TASK_SERVO_WAIT_MS);
    }
    arm_vision_stage = 100U;
    GPIO_SetBits(GPIOG, GPIO_Pin_15);
    while (1) { delay_ms(1000U); }
}

void ArmVisionTask_RunRingSequence(void)
{
    ArmJointAngles anchor, ring_pose[4];
    uint8_t found_id, id;
    uint16_t lift_us = ARM_BOOT_LIFT_US;
    uint16_t claw_us = ARM_BOOT_CLAW_US;
    float step_5[4] = {0.0f, 0.0f,
                       TASK_RING_12_STEP_5_DEG, TASK_RING_23_STEP_5_DEG};
    float step_6[4] = {0.0f, 0.0f,
                       TASK_RING_12_STEP_6_DEG, TASK_RING_23_STEP_6_DEG};

    arm_vision_stage = 1U;
    arm_ring_initial_id = 0U;
    arm_ring_poses_ready = 0U;
    arm_ring_centered_mask = 0U;
    arm_ring_visited_mask = 0U;
    task_ring_sequence_active = 1U;
    board_init();
    Swith_Init();
    LED2_Init();
    if (!board_can_init_ok) { Task_Error(1U); }
    VisionLink_Init();
    ArmServos_Init();
    ArmServos_Lift(lift_us);
    ArmServos_Claw(claw_us);
    ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
    ArmServos_ClawSlow(&claw_us, ARM_CLAW_OPEN_US);
    delay_ms(TASK_SERVO_WAIT_MS);

    delay_ms(2000U); /* Allow motor-side CAN modules to power up. */
    Emm_V5_En_Control(ARM_ADDR_UPPER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    delay_ms(20U);
    Emm_V5_En_Control(ARM_ADDR_LOWER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    /* Both joints were manually placed at the fixed home pose before power-on. */
    Arm_SetCurrentJointAngles(ARM_HOME_5_DEG, ARM_HOME_6_DEG);
    while (swith_out() == 0U) { delay_ms(10U); }

    arm_vision_stage = 2U;
    if (!Task_MoveSequence(task_stretch)) { Task_Error(3U); }
    arm_vision_stage = 3U;
    if (!Task_FindAnyTarget(VISION_MODE_RING, &found_id)) { Task_Error(5U); }
    arm_ring_initial_id = found_id;
    arm_vision_stage = 4U;
    if (!Task_Align(VISION_MODE_RING, found_id, 0U, 0U)) {
        Task_Error(arm_vision_error);
    }
    if (Arm_GetCurrentJointAngles(&anchor) != ARM_STATUS_OK) {
        Task_Error(7U);
    }
    arm_ring_centered_mask = (uint8_t)(1U << (found_id - 1U));

    ring_pose[found_id] = anchor;
    for (id = found_id; id > 1U; --id) {
        ring_pose[id - 1U].upper_deg = ring_pose[id].upper_deg - step_5[id];
        ring_pose[id - 1U].lower_deg = ring_pose[id].lower_deg - step_6[id];
    }
    for (id = found_id + 1U; id <= 3U; ++id) {
        ring_pose[id].upper_deg = ring_pose[id - 1U].upper_deg + step_5[id];
        ring_pose[id].lower_deg = ring_pose[id - 1U].lower_deg + step_6[id];
    }
    for (id = 1U; id <= 3U; ++id) {
        arm_ring_pose_5_deg[id] = ring_pose[id].upper_deg;
        arm_ring_pose_6_deg[id] = ring_pose[id].lower_deg;
    }
    arm_ring_poses_ready = 1U;

    /* Initial visual anchor: wait, then test the computed joint poses in order. */
    arm_vision_stage = 5U;
    Task_WaitSecondPressBlink();
    for (id = 1U; id <= 3U; ++id) {
        arm_vision_stage = 5U + 2U * id;
        if (!Task_MoveSequence(ring_pose[id])) { Task_Error(7U); }
        arm_ring_visited_mask |= (uint8_t)(1U << (id - 1U));
        if (id < 3U) {
            arm_vision_stage = 6U + 2U * id;
            Task_WaitSecondPressBlink();
        }
    }
    arm_vision_stage = 100U;
    GPIO_SetBits(GPIOG, GPIO_Pin_15); /* Three ring positions visited. */
    while (1) { delay_ms(1000U); }
}

void ArmVisionTask_RunMaterialTracking(void)
{
    ArmJointAngles centered_pose;
    VisionDetection seen;
    uint8_t material_id;
    uint16_t lift_us = ARM_BOOT_LIFT_US;
    uint16_t claw_us = ARM_BOOT_CLAW_US;

    arm_vision_stage = 1U;
    arm_material_pick_id = 0U;
    arm_material_pick_5_deg = 0.0f;
    arm_material_pick_6_deg = 0.0f;
    task_material_vision_active = 1U;
    task_material_track_any_id = 1U;
    board_init();
    Swith_Init();
    LED2_Init();
    if (!board_can_init_ok) { Task_Error(1U); }
    VisionLink_Init(); /* Receive OpenMV AUTO material frames; no mode command. */
    ArmServos_Init();
    ArmServos_Lift(lift_us);
    ArmServos_Claw(claw_us);
    ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
    ArmServos_ClawSlow(&claw_us, ARM_CLAW_OPEN_US);
    delay_ms(TASK_SERVO_WAIT_MS);

    delay_ms(2000U);
    Emm_V5_En_Control(ARM_ADDR_UPPER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    delay_ms(20U);
    Emm_V5_En_Control(ARM_ADDR_LOWER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    /* Manual fixed home pose is the software (0, 0) reference. */
    Arm_SetCurrentJointAngles(ARM_HOME_5_DEG, ARM_HOME_6_DEG);
    while (swith_out() == 0U) { delay_ms(10U); }

    arm_vision_stage = 2U;
    if (!Task_MoveSequence(task_stretch)) { Task_Error(3U); }
    arm_vision_stage = 3U;
    /* Stay at the shared pose until actual MATERIAL frames arrive. Do not
     * sweep sideways when the IDE preview sees a color but UART has not yet
     * supplied a stable material detection. PC9 can end this wait. */
    while (!Task_WaitDetection(VISION_MODE_GRAB, 0U, 1000U, 2U, &seen)) {
        if (swith_out() == 0U) {
            arm_vision_stage = 100U;
            while (1) { delay_ms(1000U); }
        }
    }
    material_id = seen.id;
    arm_material_pick_id = material_id;
    arm_vision_stage = 4U;
    /* Use the initial color for calibration, then follow any material color. */
    if (!Task_Align(VISION_MODE_GRAB, material_id, 1U, 0U)) {
        Task_Error(arm_vision_error);
    }
    if (Arm_GetCurrentJointAngles(&centered_pose) != ARM_STATUS_OK) {
        Task_Error(7U);
    }
    arm_material_pick_5_deg = centered_pose.upper_deg;
    arm_material_pick_6_deg = centered_pose.lower_deg;
    arm_vision_stage = 100U;
    while (1) { delay_ms(1000U); }
}

void ArmVisionTask_RunMaterialToLeftTray(void)
{
    ArmJointAngles pickup_pose;
    VisionDetection seen;
    GPIO_InitTypeDef gpio;
    uint8_t material_id;
    uint16_t lift_us = ARM_BOOT_LIFT_US;
    uint16_t claw_us = ARM_BOOT_CLAW_US;

    arm_vision_stage = 1U;
    arm_material_pick_id = 0U;
    arm_material_pick_5_deg = 0.0f;
    arm_material_pick_6_deg = 0.0f;
    task_material_vision_active = 1U;
    task_material_track_any_id = 1U;
    board_init();
    Swith_Init();
    LED2_Init();
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG, &gpio);
    GPIO_ResetBits(GPIOG, GPIO_Pin_15);
    if (!board_can_init_ok) { Task_Error(1U); }
    VisionLink_Init(); /* OpenMV AUTO reports MATERIAL frames; no mode command. */
    ArmServos_Init();
    ArmServos_Lift(lift_us);
    ArmServos_Claw(claw_us);
    ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
    ArmServos_ClawSlow(&claw_us, ARM_CLAW_OPEN_US);
    delay_ms(TASK_SERVO_WAIT_MS);

    delay_ms(2000U); /* Allow motor-side CAN modules to power up. */
    Emm_V5_En_Control(ARM_ADDR_UPPER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    delay_ms(20U);
    Emm_V5_En_Control(ARM_ADDR_LOWER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    /* The operator placed both joints at the fixed home pose before power-on. */
    Arm_SetCurrentJointAngles(ARM_HOME_5_DEG, ARM_HOME_6_DEG);
    while (swith_out() == 0U) { delay_ms(10U); }

    arm_vision_stage = 2U;
    if (!Task_MoveSequence(task_stretch)) { Task_Error(3U); }
    arm_vision_stage = 3U;
    /* Wait at the shared pose as in the successful tracking test; do not
     * re-enable the old large left/right search before pickup. */
    while (!Task_WaitDetection(VISION_MODE_GRAB, 0U, 1000U, 2U, &seen)) {}
    material_id = seen.id;
    arm_material_pick_id = material_id;
    arm_vision_stage = 4U;
    task_material_pickup_active = 1U;
    /* Calibrate on the first visible color, then track any material until
     * PC9 is pressed while fresh detections remain centered. */
    if (!Task_Align(VISION_MODE_GRAB, material_id, 1U, 0U)) {
        Task_Error(arm_vision_error);
    }
    task_material_pickup_active = 0U;
    GPIO_ResetBits(GPIOG, GPIO_Pin_15);
    if (Arm_GetCurrentJointAngles(&pickup_pose) != ARM_STATUS_OK) {
        Task_Error(7U);
    }
    arm_material_pick_5_deg = pickup_pose.upper_deg;
    arm_material_pick_6_deg = pickup_pose.lower_deg;

    /* Require the release before starting the physical pickup. */
    while (swith_out() == 0U) {
        delay_ms(10U);
        VisionLink_Poll();
        Task_LedService(10U);
    }
    delay_ms(30U);

    arm_vision_stage = 6U;
    ArmServos_Claw(ARM_CLAW_OPEN_US); /* Keep open during the descent. */
    ArmServos_LiftSlow(&lift_us, ARM_GROUND_HEIGHT_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_ClawSlow(&claw_us, ARM_CLAW_CLOSE_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
    delay_ms(TASK_SERVO_WAIT_MS);

    arm_vision_stage = 7U;
    if (!Task_MoveSequence(task_trays[0])) { Task_Error(7U); }
    arm_vision_stage = 8U;
    ArmServos_LiftSlow(&lift_us, ARM_TRAY_HEIGHT_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_ClawSlow(&claw_us, ARM_CLAW_OPEN_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
    delay_ms(TASK_SERVO_WAIT_MS);

    arm_vision_stage = 100U;
    GPIO_SetBits(GPIOG, GPIO_Pin_15); /* Left tray placement finished. */
    while (1) { delay_ms(1000U); }
}

void ArmVisionTask_RunRing1Transfer(void)
{
    ArmJointAngles ring1_pose;
    uint16_t lift_us = ARM_BOOT_LIFT_US;
    uint16_t claw_us = ARM_BOOT_CLAW_US;

    arm_vision_stage = 1U;
    board_init();
    Swith_Init();
    LED2_Init();
    if (!board_can_init_ok) { Task_Error(1U); }
    VisionLink_Init();             /* Incoming OpenMV detections only. */
    ArmServos_Init();
    ArmServos_Lift(lift_us);
    ArmServos_Claw(claw_us);
    ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
    ArmServos_ClawSlow(&claw_us, ARM_CLAW_OPEN_US);
    delay_ms(TASK_SERVO_WAIT_MS);

    delay_ms(2000U);             /* Motor-side CAN modules finish powering up. */
    Emm_V5_En_Control(ARM_ADDR_UPPER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    delay_ms(20U);
    Emm_V5_En_Control(ARM_ADDR_LOWER, true, false);
    if (!can_tx_last_cmd_ok) { Task_Error(2U); }
    /* Operator placed both joints at the fixed (0, 0) pose before power-on. */
    Arm_SetCurrentJointAngles(ARM_HOME_5_DEG, ARM_HOME_6_DEG);
    while (swith_out() == 0U) { delay_ms(10U); }

    arm_vision_stage = 2U;
    if (!Task_Move(task_stretch)) { Task_Error(3U); }
    arm_vision_stage = 3U;
    if (!Task_FindRing1()) { Task_Error(5U); }
    arm_vision_stage = 4U;
    if (!Task_Align(VISION_MODE_RING, TASK_RING_TARGET_ID, 0U, 0U)) {
        Task_Error(arm_vision_error);
    }
    if (Arm_GetCurrentJointAngles(&ring1_pose) != ARM_STATUS_OK) {
        Task_Error(7U);
    }
    arm_ring1_saved_5_deg = ring1_pose.upper_deg;
    arm_ring1_saved_6_deg = ring1_pose.lower_deg;

    arm_vision_stage = 5U;       /* Ring 1 centered; PG15 blinks until PC9. */
    Task_WaitSecondPressBlink();

    arm_vision_stage = 6U;
    if (!Task_Move(task_trays[2])) { Task_Error(7U); }
    arm_vision_stage = 7U;       /* Fixed middle-tray pickup; no close-range vision. */

    arm_vision_stage = 8U;
    ArmServos_Claw(ARM_CLAW_OPEN_US); /* Keep open throughout the descent. */
    ArmServos_LiftSlow(&lift_us, ARM_TRAY_HEIGHT_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_ClawSlow(&claw_us, ARM_CLAW_CLOSE_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
    delay_ms(TASK_SERVO_WAIT_MS);

    arm_vision_stage = 9U;
    if (!Task_Move(ring1_pose)) { Task_Error(7U); }
    arm_vision_stage = 10U;
    ArmServos_LiftSlow(&lift_us, ARM_GROUND_HEIGHT_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_ClawSlow(&claw_us, ARM_CLAW_OPEN_US);
    delay_ms(TASK_SERVO_WAIT_MS);
    ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
    delay_ms(TASK_SERVO_WAIT_MS);

    arm_vision_stage = 100U;
    while (1) { delay_ms(1000U); }
}
