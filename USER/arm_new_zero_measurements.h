#ifndef ARM_NEW_ZERO_MEASUREMENTS_H
#define ARM_NEW_ZERO_MEASUREMENTS_H

/* Measured relative to the new fixed mechanical zero, in output-joint degrees.
 * Used by ArmVisionTask_RunNewRingPickup(). */
#define NEW_ZERO_SHARED_5_DEG       (-136.0f)
#define NEW_ZERO_SHARED_6_DEG        130.0f
#define NEW_ZERO_LEFT_TRAY_5_DEG     (-32.0f)
#define NEW_ZERO_LEFT_TRAY_6_DEG      156.0f
#define NEW_ZERO_RIGHT_TRAY_5_DEG   (-153.0f)
#define NEW_ZERO_RIGHT_TRAY_6_DEG     47.0f
#define NEW_ZERO_REAR_TRAY_5_DEG    (-101.0f)
#define NEW_ZERO_REAR_TRAY_6_DEG      99.0f
#define NEW_ZERO_RING_1_5_DEG        (-89.0f)
#define NEW_ZERO_RING_1_6_DEG        217.0f
#define NEW_ZERO_RING_2_5_DEG       (-138.0f)
#define NEW_ZERO_RING_2_6_DEG        139.0f
#define NEW_ZERO_RING_3_5_DEG       (-212.0f)
#define NEW_ZERO_RING_3_6_DEG         90.0f

/* Measured ring 1 -> ring 2 offset at the measured vehicle/arm pose. */
#define NEW_ZERO_RING_12_5_DEG       (-49.0f)
#define NEW_ZERO_RING_12_6_DEG       (-78.0f)
#define NEW_ZERO_RING_23_5_DEG       (-74.0f)
#define NEW_ZERO_RING_23_6_DEG       (-49.0f)

#endif
