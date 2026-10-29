#ifndef ARM_SERVOS_H
#define ARM_SERVOS_H

#include <stdint.h>

/* PA3 measured heights: highest, material tray, and ground ring. */
#define ARM_LIFT_UP_US          1350U
#define ARM_TRAY_HEIGHT_US      1230U
#define ARM_GROUND_HEIGHT_US     700U
/* PA1 measured gripper setpoints. */
#define ARM_CLAW_OPEN_US         780U
#define ARM_CLAW_CLOSE_US       1530U
/* Estimated power-on starting pulses; actual servo position is not sensed. */
#define ARM_BOOT_LIFT_US        1300U
#define ARM_BOOT_CLAW_US         830U

/* Older test/task names kept for their disabled branches. */
#define ARM_PICKUP_DOWN_US      ARM_TRAY_HEIGHT_US
#define ARM_LIFT_RELEASE_US     ARM_TRAY_HEIGHT_US
#define ARM_SERVO_RAMP_STEP_US    10U
#define ARM_SERVO_RAMP_WAIT_MS    20U

void ArmServos_Init(void);
void ArmServos_Lift(uint16_t pulse_us);
void ArmServos_Claw(uint16_t pulse_us);
void ArmServos_LiftSlow(uint16_t *current_us, uint16_t target_us);
void ArmServos_ClawSlow(uint16_t *current_us, uint16_t target_us);

#endif
