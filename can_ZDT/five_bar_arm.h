#ifndef __FIVE_BAR_ARM_H
#define __FIVE_BAR_ARM_H

#include <stdint.h>

/*
 * Four moving links plus the fixed motor base.
 * Origin: midpoint between motor shafts.
 * +X: vehicle forward; +Y: lower motor (address 6) toward upper motor (address 5).
 */
#define ARM_LINK_L1_MM              240.0f
#define ARM_LINK_L2_MM              120.0f
#define ARM_LINK_L3_MM              240.0f
#define ARM_LINK_L4_MM              120.0f
#define ARM_BASE_D_MM               250.0f

#define ARM_ADDR_UPPER              5U
#define ARM_ADDR_LOWER              6U
#define ARM_MOTOR_PULSES_PER_REV    3200.0f
#define ARM_REDUCTION_RATIO         5.0f
#define ARM_PULSES_PER_OUTPUT_DEG   (ARM_MOTOR_PULSES_PER_REV * ARM_REDUCTION_RATIO / 360.0f)

/* Photo assembly branch: upper chain +acos, lower chain -acos. */
#define ARM_UPPER_IK_BRANCH         1.0f
#define ARM_LOWER_IK_BRANCH        (-1.0f)

/* ZDT dir value which increases the mathematical joint angle. Verify at low speed. */
#define ARM_UPPER_POSITIVE_DIR      0U
#define ARM_LOWER_POSITIVE_DIR      0U

typedef enum {
    ARM_STATUS_OK = 0,
    ARM_STATUS_BAD_ARGUMENT = -1,
    ARM_STATUS_UNREACHABLE = -2,
    ARM_STATUS_NOT_REFERENCED = -3,
    ARM_STATUS_CAN_TX_FAILED = -4
} ArmStatus;

typedef struct {
    float upper_deg;
    float lower_deg;
} ArmJointAngles;

ArmStatus Arm_InverseKinematics(float x_mm, float y_mm, ArmJointAngles *result);

/* Establish software position after homing/manual alignment; neither function moves. */
ArmStatus Arm_SetCurrentJointAngles(float upper_deg, float lower_deg);
ArmStatus Arm_SetCurrentXY(float x_mm, float y_mm);
void Arm_ClearReference(void);
uint8_t Arm_IsReferenced(void);
ArmStatus Arm_GetCurrentJointAngles(ArmJointAngles *result);

ArmStatus Arm_MoveJoint(float upper_deg, float lower_deg,
                        uint16_t max_motor_rpm, uint8_t accel);
ArmStatus Arm_MoveXY(float x_mm, float y_mm,
                     uint16_t max_motor_rpm, uint8_t accel);
ArmStatus Arm_Stop(void);

extern volatile uint32_t arm_can_batch_abort_count;

/*
 * Volatile mirrors of the software joint angles, safe to add to the Keil Watch
 * window at any optimisation level. They follow every reference update and move.
 */
extern volatile float   arm_debug_upper_deg;
extern volatile float   arm_debug_lower_deg;
extern volatile uint8_t arm_debug_reference_valid;

#endif
