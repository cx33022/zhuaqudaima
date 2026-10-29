#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdint.h>

#define VISION_MODE_GRAB   0xCCU
#define VISION_MODE_RING   0xEEU
#define VISION_MODE_AUTO   0xCAU
#define VISION_MODE_IDLE   0xFFU

#define VISION_COLOR_RED     1U
#define VISION_COLOR_YELLOW  2U
#define VISION_COLOR_BLUE    3U

typedef struct {
    uint8_t mode;
    uint8_t id;
    uint8_t found;
    int16_t dx;
    int16_t dy;
    uint16_t cx;
    uint16_t cy;
    uint16_t size;
    uint8_t quality;
    uint8_t sequence;
} VisionDetection;

/* USART1: PA9 TX -> OpenMV P5 RX; PA10 RX <- OpenMV P4 TX. */
void VisionLink_Init(void);
void VisionLink_Poll(void);
void VisionLink_ClearEvents(void);
uint8_t VisionLink_TakeDetection(VisionDetection *result);
uint8_t VisionLink_TakeTargetAck(uint8_t *sequence, uint8_t *result);
uint8_t VisionLink_SetTarget(uint8_t mode, uint8_t selector);

extern volatile uint32_t vision_link_valid_frames;
extern volatile uint32_t vision_link_bad_frames;
extern volatile uint32_t vision_link_uart_errors;
extern volatile uint32_t vision_link_uart_overruns;
extern volatile uint32_t vision_link_uart_framing_errors;
extern volatile uint32_t vision_link_uart_noise_errors;
extern volatile uint32_t vision_link_rx_bytes;

#endif
