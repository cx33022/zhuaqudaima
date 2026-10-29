#include "five_bar_arm.h"

#include <math.h>

#include "Emm_V5.h"
#include "can.h"

#define ARM_PI_F                    3.14159265358979323846f
#define ARM_RAD_TO_DEG              (180.0f / ARM_PI_F)
#define ARM_COS_TOLERANCE           0.0001f
#define ARM_MIN_RADIUS_MM           0.001f
#define ARM_MAX_MOTOR_RPM           5000U

static ArmJointAngles arm_current_angles;
static uint8_t arm_reference_valid = 0U;

volatile uint32_t arm_can_batch_abort_count = 0U;

/*
 * Volatile mirrors of the software joint angles. The internal state above is
 * static and non-volatile, so an optimised build can keep it in registers and
 * Keil Watch cannot show it. These four are always watchable during calibration.
 */
volatile float   arm_debug_upper_deg = 0.0f;
volatile float   arm_debug_lower_deg = 0.0f;
volatile uint8_t arm_debug_reference_valid = 0U;

static float Arm_ClampUnit(float value)
{
    if (value > 1.0f) { return 1.0f; }
    if (value < -1.0f) { return -1.0f; }
    return value;
}

static ArmStatus Arm_SolveChain(float motor_y_mm,
                                float active_link_mm,
                                float passive_link_mm,
                                float branch,
                                float x_mm,
                                float y_mm,
                                float *angle_deg)
{
    float dx;
    float dy;
    float radius_sq;
    float radius;
    float cosine;
    float alpha;
    float beta;

    if (angle_deg == 0) { return ARM_STATUS_BAD_ARGUMENT; }

    dx = x_mm;
    dy = y_mm - motor_y_mm;
    radius_sq = dx * dx + dy * dy;
    radius = sqrtf(radius_sq);
    if (radius < ARM_MIN_RADIUS_MM) { return ARM_STATUS_UNREACHABLE; }

    cosine = (active_link_mm * active_link_mm + radius_sq
              - passive_link_mm * passive_link_mm)
             / (2.0f * active_link_mm * radius);
    if (cosine > (1.0f + ARM_COS_TOLERANCE)
        || cosine < (-1.0f - ARM_COS_TOLERANCE)) {
        return ARM_STATUS_UNREACHABLE;
    }

    alpha = atan2f(dy, dx);
    beta = acosf(Arm_ClampUnit(cosine));
    *angle_deg = (alpha + branch * beta) * ARM_RAD_TO_DEG;
    return ARM_STATUS_OK;
}

ArmStatus Arm_InverseKinematics(float x_mm, float y_mm, ArmJointAngles *result)
{
    ArmStatus status;

    if (result == 0) { return ARM_STATUS_BAD_ARGUMENT; }

    status = Arm_SolveChain(ARM_BASE_D_MM * 0.5f,
                            ARM_LINK_L1_MM, ARM_LINK_L2_MM,
                            ARM_UPPER_IK_BRANCH,
                            x_mm, y_mm, &result->upper_deg);
    if (status != ARM_STATUS_OK) { return status; }

    return Arm_SolveChain(-ARM_BASE_D_MM * 0.5f,
                          ARM_LINK_L3_MM, ARM_LINK_L4_MM,
                          ARM_LOWER_IK_BRANCH,
                          x_mm, y_mm, &result->lower_deg);
}

ArmStatus Arm_SetCurrentJointAngles(float upper_deg, float lower_deg)
{
    arm_current_angles.upper_deg = upper_deg;
    arm_current_angles.lower_deg = lower_deg;
    arm_reference_valid = 1U;
    arm_debug_upper_deg = upper_deg;
    arm_debug_lower_deg = lower_deg;
    arm_debug_reference_valid = 1U;
    return ARM_STATUS_OK;
}

ArmStatus Arm_SetCurrentXY(float x_mm, float y_mm)
{
    ArmJointAngles angles;
    ArmStatus status = Arm_InverseKinematics(x_mm, y_mm, &angles);

    if (status != ARM_STATUS_OK) { return status; }
    return Arm_SetCurrentJointAngles(angles.upper_deg, angles.lower_deg);
}

void Arm_ClearReference(void)
{
    arm_reference_valid = 0U;
    arm_debug_reference_valid = 0U;
}

uint8_t Arm_IsReferenced(void)
{
    return arm_reference_valid;
}

ArmStatus Arm_GetCurrentJointAngles(ArmJointAngles *result)
{
    if (result == 0) { return ARM_STATUS_BAD_ARGUMENT; }
    if (!arm_reference_valid) { return ARM_STATUS_NOT_REFERENCED; }
    *result = arm_current_angles;
    return ARM_STATUS_OK;
}

static uint32_t Arm_DegreesToPulses(float delta_deg)
{
    float magnitude = fabsf(delta_deg) * ARM_PULSES_PER_OUTPUT_DEG;
    return (uint32_t)(magnitude + 0.5f);
}

static uint8_t Arm_Direction(float delta_deg, uint8_t positive_dir)
{
    return (delta_deg >= 0.0f) ? positive_dir : (uint8_t)(positive_dir ? 0U : 1U);
}

static uint16_t Arm_ScaledRpm(float magnitude_deg,
                              float largest_magnitude_deg,
                              uint16_t max_motor_rpm)
{
    float scaled;

    if (magnitude_deg <= 0.0f || largest_magnitude_deg <= 0.0f) { return 1U; }
    scaled = (float)max_motor_rpm * magnitude_deg / largest_magnitude_deg;
    if (scaled < 1.0f) { return 1U; }
    if (scaled > (float)max_motor_rpm) { return max_motor_rpm; }
    return (uint16_t)(scaled + 0.5f);
}

ArmStatus Arm_MoveJoint(float upper_deg, float lower_deg,
                        uint16_t max_motor_rpm, uint8_t accel)
{
    float delta_upper;
    float delta_lower;
    float magnitude_upper;
    float magnitude_lower;
    float largest_magnitude;
    uint32_t pulses_upper;
    uint32_t pulses_lower;
    uint16_t rpm_upper;
    uint16_t rpm_lower;

    if (!arm_reference_valid) { return ARM_STATUS_NOT_REFERENCED; }
    if (max_motor_rpm == 0U || max_motor_rpm > ARM_MAX_MOTOR_RPM) {
        return ARM_STATUS_BAD_ARGUMENT;
    }

    delta_upper = upper_deg - arm_current_angles.upper_deg;
    delta_lower = lower_deg - arm_current_angles.lower_deg;
    magnitude_upper = fabsf(delta_upper);
    magnitude_lower = fabsf(delta_lower);
    largest_magnitude = (magnitude_upper > magnitude_lower)
                        ? magnitude_upper : magnitude_lower;
    pulses_upper = Arm_DegreesToPulses(delta_upper);
    pulses_lower = Arm_DegreesToPulses(delta_lower);

    if (pulses_upper == 0U && pulses_lower == 0U) { return ARM_STATUS_OK; }

    rpm_upper = Arm_ScaledRpm(magnitude_upper, largest_magnitude, max_motor_rpm);
    rpm_lower = Arm_ScaledRpm(magnitude_lower, largest_magnitude, max_motor_rpm);

    /* Always overwrite both cached targets before the broadcast start. */
    Emm_V5_Pos_Control(ARM_ADDR_UPPER,
                       Arm_Direction(delta_upper, ARM_UPPER_POSITIVE_DIR),
                       rpm_upper, accel, pulses_upper, false, true);
    if (!can_tx_last_cmd_ok) {
        ++arm_can_batch_abort_count;
        return ARM_STATUS_CAN_TX_FAILED;
    }

    Emm_V5_Pos_Control(ARM_ADDR_LOWER,
                       Arm_Direction(delta_lower, ARM_LOWER_POSITIVE_DIR),
                       rpm_lower, accel, pulses_lower, false, true);
    if (!can_tx_last_cmd_ok) {
        ++arm_can_batch_abort_count;
        return ARM_STATUS_CAN_TX_FAILED;
    }

    Emm_V5_Synchronous_motion(0x00U);
    if (!can_tx_last_cmd_ok) {
        ++arm_can_batch_abort_count;
        return ARM_STATUS_CAN_TX_FAILED;
    }

    arm_current_angles.upper_deg = upper_deg;
    arm_current_angles.lower_deg = lower_deg;
    arm_debug_upper_deg = upper_deg;
    arm_debug_lower_deg = lower_deg;
    return ARM_STATUS_OK;
}

ArmStatus Arm_MoveXY(float x_mm, float y_mm,
                     uint16_t max_motor_rpm, uint8_t accel)
{
    ArmJointAngles target;
    ArmStatus status = Arm_InverseKinematics(x_mm, y_mm, &target);

    if (status != ARM_STATUS_OK) { return status; }
    return Arm_MoveJoint(target.upper_deg, target.lower_deg,
                         max_motor_rpm, accel);
}

ArmStatus Arm_Stop(void)
{
    Emm_V5_Stop_Now(ARM_ADDR_UPPER, true);
    if (!can_tx_last_cmd_ok) { return ARM_STATUS_CAN_TX_FAILED; }

    Emm_V5_Stop_Now(ARM_ADDR_LOWER, true);
    if (!can_tx_last_cmd_ok) { return ARM_STATUS_CAN_TX_FAILED; }

    Emm_V5_Synchronous_motion(0x00U);
    return can_tx_last_cmd_ok ? ARM_STATUS_OK : ARM_STATUS_CAN_TX_FAILED;
}
