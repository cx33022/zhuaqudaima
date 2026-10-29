#ifndef ARM_VISION_TASK_H
#define ARM_VISION_TASK_H

#include <stdint.h>

/* Zero selects motor-only ring-2 tracking from the power-on pose and leaves
 * servo PWM off. Set to a measured PA3 placement pulse only when restoring the
 * complete grab/place task. */
#define ARM_PLACE_DOWN_US 0U

void ArmVisionTask_Run(void);
/* From the shared pose, wait for a material block and track its center only. */
void ArmVisionTask_RunMaterialTracking(void);
/* Center a material block, pause for PC9, pick from ground and place at left tray. */
void ArmVisionTask_RunMaterialToLeftTray(void);
/* Locate one numbered ring and visit estimated ring 1, 2, 3 poses by PC9. */
void ArmVisionTask_RunRingSequence(void);
void ArmVisionTask_RunNewRingPickup(void);
/* Passive OpenMV reports only: locate ring 1, pause for PC9, pick tray 2 and place. */
void ArmVisionTask_RunRing1Transfer(void);

extern volatile uint32_t arm_vision_stage;
extern volatile uint32_t arm_vision_error;
extern volatile uint8_t arm_vision_tray_for_color[4];
extern volatile int16_t arm_vision_last_dx;
extern volatile int16_t arm_vision_last_dy;
/* 1 when centered; the active pickup flow requires three consecutive
 * centered detection groups before it marks the target ready. PA8 is the
 * CRC-valid OpenMV frame heartbeat, while PG15 blinks when pickup is ready. */
extern volatile uint8_t arm_vision_centered;
/* Probe diagnostics: step 10 means the 5-degree, five-frame averaged mapping
 * was accepted. j00/j10 are address 5 pixel/degree; j01/j11 are address 6. */
extern volatile uint32_t arm_vision_probe_step;
extern volatile float arm_vision_probe_j00;
extern volatile float arm_vision_probe_j01;
extern volatile float arm_vision_probe_j10;
extern volatile float arm_vision_probe_j11;
extern volatile float arm_vision_probe_determinant;
/* Ring 1 joint setpoint saved after visual centering, for Keil Watch. */
extern volatile float arm_ring1_saved_5_deg;
extern volatile float arm_ring1_saved_6_deg;
/* Ring sequence diagnostics; indices 1..3 hold software joint setpoints. */
extern volatile uint8_t arm_ring_initial_id;
extern volatile uint8_t arm_ring_poses_ready;
/* Only the initially found ring is visually centered in this test sequence. */
extern volatile uint8_t arm_ring_centered_mask;
/* Bit 0/1/2 set after the computed ring 1/2/3 pose was visited by PC9. */
extern volatile uint8_t arm_ring_visited_mask;
/* Material tracking and pickup update this to the currently followed color. */
extern volatile uint8_t arm_material_pick_id;
extern volatile float arm_material_pick_5_deg;
extern volatile float arm_material_pick_6_deg;
extern volatile float arm_ring_pose_5_deg[4];
extern volatile float arm_ring_pose_6_deg[4];

#endif
