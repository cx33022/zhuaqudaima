#ifndef ARM_POSITIONS_H
#define ARM_POSITIONS_H

/* All values below are software joint angles from the manually aligned (0, 0)
 * pose. They are measured navigation or pickup points, not visual centering results. */
/* Fixed power-on pose: both address 5 and address 6 are manually placed here. */
#define ARM_HOME_5_DEG       0.0f
#define ARM_HOME_6_DEG       0.0f
/* Shared transit pose, slightly forward of the left/right tray line.
 * Tray joint midpoint: ((-45 - 156)/2, (150 + 47.3)/2) = (-100.5, 98.65).
 * On this arm (-5, +5) was measured as forward. Apply (-35, +35) to the
 * midpoint: (-135.5, 133.65). This is a joint-space approach estimate. */
#define ARM_STRETCH_5_DEG (-135.5f)
#define ARM_STRETCH_6_DEG  133.65f
/* Left material tray, measured coarse pickup approach. */
#define ARM_TRAY_LEFT_5_DEG  (-45.0f)
#define ARM_TRAY_LEFT_6_DEG  150.0f
/* Right material tray, measured coarse pickup approach. */
#define ARM_TRAY_RIGHT_5_DEG (-156.0f)
#define ARM_TRAY_RIGHT_6_DEG  47.3f
/* Middle material tray, fixed pickup pose for yellow block 2. */
#define ARM_TRAY_CENTER_5_DEG (-91.0f)
#define ARM_TRAY_CENTER_6_DEG  91.3f
/* In the disabled ring-sequence test, one ring was visually centered and the
 * other ring poses were estimated at runtime from relative joint offsets. */

#endif
