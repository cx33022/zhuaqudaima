#include "vision_link.h"

#include "stm32f4xx.h"

#define VISION_SOF_0             0xAAU
#define VISION_SOF_1             0x55U
#define VISION_VERSION           0x01U
#define VISION_EOF_0             0x0DU
#define VISION_EOF_1             0x0AU
#define VISION_CMD_SET_TARGET    0x10U
#define VISION_RSP_ACK           0x80U
#define VISION_EVT_DETECTION     0x81U
#define VISION_MAX_PAYLOAD       32U

typedef struct {
    uint8_t state;
    uint8_t version;
    uint8_t type;
    uint8_t sequence;
    uint8_t length;
    uint8_t index;
    uint8_t crc;
    uint8_t payload[VISION_MAX_PAYLOAD];
} VisionParser;

static VisionParser vision_parser;
static VisionDetection vision_detection;
static uint8_t vision_detection_ready;
static uint8_t vision_ack_ready;
static uint8_t vision_ack_sequence;
static uint8_t vision_ack_result;
static uint8_t vision_tx_sequence;

volatile uint32_t vision_link_valid_frames;
volatile uint32_t vision_link_bad_frames;
volatile uint32_t vision_link_uart_errors;
volatile uint32_t vision_link_uart_overruns;
volatile uint32_t vision_link_uart_framing_errors;
volatile uint32_t vision_link_uart_noise_errors;
volatile uint32_t vision_link_rx_bytes;

static uint8_t VisionLink_Crc8(uint8_t crc, uint8_t value)
{
    uint8_t i;
    crc ^= value;
    for (i = 0U; i < 8U; ++i) {
        crc = (uint8_t)((crc & 0x80U) ? ((crc << 1U) ^ 0x07U) : (crc << 1U));
    }
    return crc;
}

static uint16_t VisionLink_U16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static void VisionLink_OnFrame(void)
{
    const uint8_t *p = vision_parser.payload;
    ++vision_link_valid_frames;

    if (vision_parser.type == VISION_EVT_DETECTION && vision_parser.length == 14U) {
        vision_detection.mode = p[0];
        vision_detection.id = p[1];
        vision_detection.found = p[2];
        vision_detection.dx = (int16_t)VisionLink_U16(&p[3]);
        vision_detection.dy = (int16_t)VisionLink_U16(&p[5]);
        vision_detection.cx = VisionLink_U16(&p[7]);
        vision_detection.cy = VisionLink_U16(&p[9]);
        vision_detection.size = VisionLink_U16(&p[11]);
        vision_detection.quality = p[13];
        vision_detection.sequence = vision_parser.sequence;
        vision_detection_ready = 1U;
    } else if (vision_parser.type == VISION_RSP_ACK && vision_parser.length == 4U
               && p[0] == VISION_CMD_SET_TARGET) {
        vision_ack_sequence = vision_parser.sequence;
        vision_ack_result = p[1];
        vision_ack_ready = 1U;
    }
}

static void VisionLink_ParseByte(uint8_t value)
{
    VisionParser *parser = &vision_parser;
    switch (parser->state) {
    case 0U:
        if (value == VISION_SOF_0) { parser->state = 1U; }
        break;
    case 1U:
        parser->state = (value == VISION_SOF_1) ? 2U : 0U;
        break;
    case 2U:
        parser->version = value;
        parser->crc = VisionLink_Crc8(0U, value);
        parser->state = 3U;
        break;
    case 3U:
        parser->type = value;
        parser->crc = VisionLink_Crc8(parser->crc, value);
        parser->state = 4U;
        break;
    case 4U:
        parser->sequence = value;
        parser->crc = VisionLink_Crc8(parser->crc, value);
        parser->state = 5U;
        break;
    case 5U:
        parser->length = value;
        parser->index = 0U;
        parser->crc = VisionLink_Crc8(parser->crc, value);
        if (value > VISION_MAX_PAYLOAD) {
            ++vision_link_bad_frames;
            parser->state = 0U;
        } else {
            parser->state = value ? 6U : 7U;
        }
        break;
    case 6U:
        parser->payload[parser->index++] = value;
        parser->crc = VisionLink_Crc8(parser->crc, value);
        if (parser->index == parser->length) { parser->state = 7U; }
        break;
    case 7U:
        if (value != parser->crc || parser->version != VISION_VERSION) {
            ++vision_link_bad_frames;
            parser->state = 0U;
        } else {
            parser->state = 8U;
        }
        break;
    case 8U:
        if (value == VISION_EOF_0) { parser->state = 9U; }
        else { ++vision_link_bad_frames; parser->state = 0U; }
        break;
    case 9U:
        if (value == VISION_EOF_1) { VisionLink_OnFrame(); }
        else { ++vision_link_bad_frames; }
        parser->state = 0U;
        break;
    default:
        parser->state = 0U;
        break;
    }
}

static void VisionLink_SendByte(uint8_t value)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, value);
}

void VisionLink_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200U;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);
    vision_parser.state = 0U;
    VisionLink_ClearEvents();
}

void VisionLink_Poll(void)
{
    uint32_t status;
    while (1) {
        status = USART1->SR;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
            volatile uint32_t ignored = USART1->DR;
            (void)ignored;
            ++vision_link_rx_bytes;
            ++vision_link_uart_errors;
            if (status & USART_SR_ORE) { ++vision_link_uart_overruns; }
            if (status & USART_SR_FE) { ++vision_link_uart_framing_errors; }
            if (status & USART_SR_NE) { ++vision_link_uart_noise_errors; }
            vision_parser.state = 0U;
        } else if (status & USART_SR_RXNE) {
            ++vision_link_rx_bytes;
            VisionLink_ParseByte((uint8_t)USART1->DR);
        } else {
            return;
        }
    }
}

void VisionLink_ClearEvents(void)
{
    VisionLink_Poll();
    vision_detection_ready = 0U;
    vision_ack_ready = 0U;
}

uint8_t VisionLink_TakeDetection(VisionDetection *result)
{
    if (result == 0 || !vision_detection_ready) { return 0U; }
    *result = vision_detection;
    vision_detection_ready = 0U;
    return 1U;
}

uint8_t VisionLink_TakeTargetAck(uint8_t *sequence, uint8_t *result)
{
    if (sequence == 0 || result == 0 || !vision_ack_ready) { return 0U; }
    *sequence = vision_ack_sequence;
    *result = vision_ack_result;
    vision_ack_ready = 0U;
    return 1U;
}

uint8_t VisionLink_SetTarget(uint8_t mode, uint8_t selector)
{
    uint8_t sequence = ++vision_tx_sequence;
    uint8_t body[6];
    uint8_t crc = 0U;
    uint8_t i;

    body[0] = VISION_VERSION;
    body[1] = VISION_CMD_SET_TARGET;
    body[2] = sequence;
    body[3] = 2U;
    body[4] = mode;
    body[5] = selector;
    VisionLink_ClearEvents();
    VisionLink_SendByte(VISION_SOF_0);
    VisionLink_SendByte(VISION_SOF_1);
    for (i = 0U; i < sizeof(body); ++i) {
        crc = VisionLink_Crc8(crc, body[i]);
        VisionLink_SendByte(body[i]);
    }
    VisionLink_SendByte(crc);
    VisionLink_SendByte(VISION_EOF_0);
    VisionLink_SendByte(VISION_EOF_1);
    return sequence;
}
