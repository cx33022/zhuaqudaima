#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "board.h"
#include "Emm_V5.h"
#include "uart6_ckp.h"
#include "Serial.h"
#include "yyb_move.h"
#include "imu.h"
#include "PWM.h"
#include "LED.h"
#include "OLED.h"
#include "swith.h"
#include "pid.h"
#include "usart5.h"
#include "wit_c_sdk.h"
#include "exti.h"
#include "servo_calibration.h"
#include "arm_servos.h"
#include <math.h>
#include <stdio.h>
#include "five_bar_arm.h"
#include "arm_positions.h"
#include "arm_vision_task.h"
#include "vision_link.h"
#include "timer.h"

/* Temporary zero-setup mode: keep STM32 idle while setting motor-side home. */
#define ARM_ZERO_SETUP_IDLE 0
/* New-zero ring search and two material transfers. */
#define ARM_NEW_RING_PICKUP 1
/* Optional PC9/PA8 isolation test; normal visual task is selected below. */
#define POWER_BUTTON_DIAG 0
/* Track one material block from the shared pose; no pickup or placement. */
#define ARM_MATERIAL_TRACK_ONLY 0
/* Center one detected material block, then pick and place on left tray. */
#define ARM_MATERIAL_LEFT_TRANSFER 0
/* Locate any numbered ring, then visit estimated ring 1/2/3 poses by PC9. */
#define ARM_RING_SEQUENCE 0
/* Automatic ring-1 registration, then PC9 starts middle-tray pickup/place. */
#define ARM_RING1_TRANSFER 0
/* Previous PC9-only PA3/PA1 servo test, kept disabled. */
#define SERVO_PICK_CYCLE 0
/* Standalone PA3/PA1 servo calibration over USART6 PC6/PC7. */
#define SERVO_UART6_CALIBRATION 0
/* Show the OpenMV ring number received on PA10 with PA8/PG15. */
#define OPENMV_UART_LED_TEST 0
/* PC9 starts address 5/6 tracking ring 2 from the power-on pose; no servo PWM. */
#define ARM_VISION_TASK 0
/* Address 5/6 only: each PC9 press performs one small direction-calibration step. */
#define ARM_DIRECTION_TEST 0
/* PC9 runs the calibrated 5/6 arm route: home -> stretch -> left -> right -> center -> home. */
#define ARM_STATION_TEST 0
/* Serial jog tool on USART6 (PC6/PC7, 115200): measure ring offsets. */
#define ARM_SERIAL_JOG 0
/* Each PC9 press starts the HWT101-guided two-round fixed route. */
#define PA2_PA3_SERVO_TEST 0
#define PA3_FAST_SWEEP_TEST 1  /* 1: only PA3 moves; PA2 holds its startup position. */
#define CHASSIS_POWER_ON_TEST 0
/* Four motors run continuously at 30 RPM, directions 0/1/0/1. */
#define CHASSIS_CAN_MOTOR_TEST 0
/* Link test only: PA6 fill light ramps 0-100% then holds 40%, nothing else runs. */
#define LIGHT_TEST 0

#define ROUTE_START_TRIM 0

#if OPENMV_UART_LED_TEST
#define OPENMV_RING_WINDOW_US          800000U
#define OPENMV_RING_BLINK_HALF_US      200000U

volatile uint32_t openmv_ring_frames = 0U;
volatile uint32_t openmv_ring_id = 0U;
volatile uint32_t openmv_ring_cx = 0U;
volatile uint32_t openmv_ring_cy = 0U;

static void OpenMvPg15_Set(uint8_t on)
{
    GPIO_WriteBit(GPIOG, GPIO_Pin_15, on ? Bit_SET : Bit_RESET);
}

static void OpenMvUartTest_Run(void)
{
    GPIO_InitTypeDef pg15;
    VisionDetection detection;
    uint32_t window_us = 0U;
    uint32_t blink_us = 0U;
    uint8_t window_id = 0U;
    uint8_t displayed_id = 0U;
    uint8_t led_on = 0U;
    uint8_t wanted_on;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    GPIO_StructInit(&pg15);
    pg15.GPIO_Pin = GPIO_Pin_15;
    pg15.GPIO_Mode = GPIO_Mode_OUT;
    pg15.GPIO_OType = GPIO_OType_PP;
    pg15.GPIO_PuPd = GPIO_PuPd_NOPULL;
    pg15.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &pg15);
    OpenMvPg15_Set(0U);

    OpenMvPg15_Set(1U);
    delay_ms(300U);
    OpenMvPg15_Set(0U);
    LED_Set2(1U);
    delay_ms(300U);
    LED_Set2(0U);
    VisionLink_Init();

    while (1) {
        VisionLink_Poll();
        if (VisionLink_TakeDetection(&detection) && detection.mode == VISION_MODE_RING) {
            ++openmv_ring_frames;
            if (detection.found && detection.id >= 1U && detection.id <= 3U) {
                window_id = detection.id;
                openmv_ring_cx = detection.cx;
                openmv_ring_cy = detection.cy;
            }
        }
        delay_us(25U);
        window_us += 25U;
        blink_us += 25U;
        if (blink_us >= (OPENMV_RING_BLINK_HALF_US * 2U)) {
            blink_us = 0U;
        }
        if (window_us >= OPENMV_RING_WINDOW_US) {
            window_us = 0U;
            displayed_id = window_id;
            window_id = 0U;
            openmv_ring_id = displayed_id;
            OpenMvPg15_Set(displayed_id == 1U);
            if (displayed_id == 0U) {
                openmv_ring_cx = 0U;
                openmv_ring_cy = 0U;
            }
        }
        wanted_on = (displayed_id == 2U ||
                     (displayed_id == 3U && blink_us < OPENMV_RING_BLINK_HALF_US)) ? 1U : 0U;
        if (wanted_on != led_on) {
            LED_Set2(wanted_on);
            led_on = wanted_on;
        }
    }
}
#endif

#if ARM_DIRECTION_TEST
#define ARM_DIRECTION_TEST_STEP_DEG    2.0f
#define ARM_DIRECTION_TEST_RPM         20U
#define ARM_DIRECTION_TEST_ACCEL       20U

/* Keil Watch diagnostics: stage 2 means ready for the first PC9 press. */
volatile uint32_t arm_direction_test_stage = 0U;
volatile uint32_t arm_direction_test_press_count = 0U;
volatile int32_t arm_direction_test_last_status = ARM_STATUS_OK;
volatile uint32_t arm_direction_test_last_rx_id = 0U;

static void ArmDirectionTest_WaitForPress(void)
{
    /* PC9 is active-low. A held key at power-on must be released first. */
    while (swith_out() == 0U) {
        LED_Set2(1U);
        delay_ms(10);
    }
    LED_Set2(0U);
    while (swith_out() != 0U) { delay_ms(10); }
    LED_Set2(1U);
    delay_ms(30);
    while (swith_out() == 0U) { delay_ms(10); }
    LED_Set2(0U);
    delay_ms(30);
}

static ArmStatus ArmDirectionTest_RunStep(uint32_t step)
{
    switch (step & 0x03U) {
    case 0U:
        /* Address 5: mathematical positive direction. */
        return Arm_MoveJoint(ARM_DIRECTION_TEST_STEP_DEG, 0.0f,
                             ARM_DIRECTION_TEST_RPM, ARM_DIRECTION_TEST_ACCEL);
    case 1U:
        /* Address 5: return to the software zero. */
        return Arm_MoveJoint(0.0f, 0.0f,
                             ARM_DIRECTION_TEST_RPM, ARM_DIRECTION_TEST_ACCEL);
    case 2U:
        /* Address 6: mathematical positive direction. */
        return Arm_MoveJoint(0.0f, ARM_DIRECTION_TEST_STEP_DEG,
                             ARM_DIRECTION_TEST_RPM, ARM_DIRECTION_TEST_ACCEL);
    default:
        /* Address 6: return to the software zero. */
        return Arm_MoveJoint(0.0f, 0.0f,
                             ARM_DIRECTION_TEST_RPM, ARM_DIRECTION_TEST_ACCEL);
    }
}
#endif

#if ARM_STATION_TEST
#define ARM_STATION_TEST_RPM            30U
#define ARM_STATION_TEST_ACCEL          20U
#define ARM_STATION_TEST_DWELL_MS       100U
#define ARM_STATION_TEST_STEP_DEG       2.0f
#define ARM_STATION_TEST_STATION_COUNT  3U

/* ---- 2026-10-27 实物标定的固定点 ---- */
/* home    = 人工固定的上电初始姿态 */
/* retract = 三个工位共用的伸展/过渡位置 */
/* extend  = 对应物料盘的定位位置 */
typedef struct {
    float upper_deg;
    float lower_deg;
} ArmStationAngles;

typedef struct {
    ArmStationAngles retract;
    ArmStationAngles extend;
} ArmStation;

static const ArmStationAngles arm_home = {
    ARM_HOME_5_DEG,
    ARM_HOME_6_DEG
};

static const ArmStation arm_stations[ARM_STATION_TEST_STATION_COUNT] = {
    /* 左一：公共伸展位 -> 左一定位 */
    { {ARM_STRETCH_5_DEG, ARM_STRETCH_6_DEG},
      {ARM_TRAY_LEFT_5_DEG, ARM_TRAY_LEFT_6_DEG} },
    /* 右二：公共伸展位 -> 右二定位 */
    { {ARM_STRETCH_5_DEG, ARM_STRETCH_6_DEG},
      {ARM_TRAY_RIGHT_5_DEG, ARM_TRAY_RIGHT_6_DEG} },
    /* 中间：公共伸展位 -> 中间定位 */
    { {ARM_STRETCH_5_DEG, ARM_STRETCH_6_DEG},
      {ARM_TRAY_CENTER_5_DEG, ARM_TRAY_CENTER_6_DEG} },
};

volatile uint32_t arm_station_test_stage = 0U;
volatile uint32_t arm_station_test_press_count = 0U;
volatile int32_t  arm_station_test_last_status = ARM_STATUS_OK;
volatile uint32_t arm_station_test_current_station = 0U;
volatile uint32_t arm_station_test_move_seq = 0U;

static void ArmStationTest_WaitForPress(void)
{
    while (swith_out() == 0U) { delay_ms(10); }
    LED_Set2(0U);
    while (swith_out() != 0U) { delay_ms(10); }
    LED_Set2(1U);
    delay_ms(30);
    while (swith_out() == 0U) { delay_ms(10); }
    LED_Set2(0U);
    delay_ms(30);
}

static ArmStatus ArmStationTest_MoveBlocking(ArmJointAngles target)
{
    ArmStatus status;
    ArmJointAngles cur;
    float du, dl;

    if (Arm_GetCurrentJointAngles(&cur) != ARM_STATUS_OK) { return ARM_STATUS_NOT_REFERENCED; }
    du = target.upper_deg - cur.upper_deg;
    dl = target.lower_deg - cur.lower_deg;
    if (du == 0.0f && dl == 0.0f) { return ARM_STATUS_OK; }

    status = Arm_MoveJoint(target.upper_deg, target.lower_deg,
                           ARM_STATION_TEST_RPM, ARM_STATION_TEST_ACCEL);
    if (status != ARM_STATUS_OK) { return status; }
    /* 没有到位反馈，用保守延时近似。时间 = 最大脉冲数 / (RPM * 步/转 / 60) * 1000 * 1.5 */
    {
        float pu = (du >= 0.0f ? du : -du) * ARM_PULSES_PER_OUTPUT_DEG;
        float pl = (dl >= 0.0f ? dl : -dl) * ARM_PULSES_PER_OUTPUT_DEG;
        float pmax = (pu > pl) ? pu : pl;
        uint32_t ms = (uint32_t)(pmax * 60000.0f / ((float)ARM_STATION_TEST_RPM * ARM_MOTOR_PULSES_PER_REV) * 1.5f);
        if (ms < ARM_STATION_TEST_DWELL_MS) { ms = ARM_STATION_TEST_DWELL_MS; }
        delay_ms1(ms);
    }
    return ARM_STATUS_OK;
}

/* Linear joint-space interpolation. Each synchronized command changes neither
 * joint by more than ARM_STATION_TEST_STEP_DEG. */
static ArmStatus ArmStationTest_MoveInSteps(ArmJointAngles target)
{
    ArmJointAngles start;
    ArmJointAngles next;
    ArmStatus status;
    float du, dl, largest;
    uint32_t segments, i;

    status = Arm_GetCurrentJointAngles(&start);
    if (status != ARM_STATUS_OK) { return status; }

    du = target.upper_deg - start.upper_deg;
    dl = target.lower_deg - start.lower_deg;
    largest = (fabsf(du) > fabsf(dl)) ? fabsf(du) : fabsf(dl);
    if (largest <= 0.0f) { return ARM_STATUS_OK; }

    segments = (uint32_t)(largest / ARM_STATION_TEST_STEP_DEG);
    if ((float)segments * ARM_STATION_TEST_STEP_DEG < largest) { ++segments; }
    if (segments == 0U) { segments = 1U; }

    for (i = 1U; i <= segments; ++i) {
        next.upper_deg = start.upper_deg + du * (float)i / (float)segments;
        next.lower_deg = start.lower_deg + dl * (float)i / (float)segments;
        status = ArmStationTest_MoveBlocking(next);
        if (status != ARM_STATUS_OK) { return status; }
    }
    return ARM_STATUS_OK;
}

static ArmStatus ArmStationTest_Goto(uint32_t station_index, uint8_t extend)
{
    const ArmStation *st = &arm_stations[station_index];
    ArmJointAngles target;

    target.upper_deg = extend ? st->extend.upper_deg : st->retract.upper_deg;
    target.lower_deg = extend ? st->extend.lower_deg : st->retract.lower_deg;
    return ArmStationTest_MoveInSteps(target);
}

/* One PC9 press runs the complete calibrated route:
 * home -> common stretch -> left -> stretch -> right -> stretch
 *      -> center -> stretch -> home. */
static void ArmStationTest_RunAll(void)
{
    ArmJointAngles target;

    arm_station_test_move_seq = 0U;

    /* step 1: start from the fixed software zero. */
    target.upper_deg = arm_home.upper_deg;
    target.lower_deg = arm_home.lower_deg;
    arm_station_test_move_seq = 1U;
    arm_station_test_last_status = ArmStationTest_MoveInSteps(target);
    if (arm_station_test_last_status != ARM_STATUS_OK) { return; }

    /* step 2: common stretch position. */
    arm_station_test_move_seq = 2U;
    arm_station_test_last_status = ArmStationTest_Goto(0U, 0U);
    if (arm_station_test_last_status != ARM_STATUS_OK) { return; }

    /* step 3: left station. */
    arm_station_test_move_seq = 3U;
    arm_station_test_last_status = ArmStationTest_Goto(0U, 1U);
    if (arm_station_test_last_status != ARM_STATUS_OK) { return; }

    /* step 4: common stretch position. */
    arm_station_test_move_seq = 4U;
    arm_station_test_last_status = ArmStationTest_Goto(0U, 0U);
    if (arm_station_test_last_status != ARM_STATUS_OK) { return; }

    /* step 5: right station. */
    arm_station_test_move_seq = 5U;
    arm_station_test_last_status = ArmStationTest_Goto(1U, 1U);
    if (arm_station_test_last_status != ARM_STATUS_OK) { return; }

    /* step 6: common stretch position. */
    arm_station_test_move_seq = 6U;
    arm_station_test_last_status = ArmStationTest_Goto(1U, 0U);
    if (arm_station_test_last_status != ARM_STATUS_OK) { return; }

    /* step 7: center station. */
    arm_station_test_move_seq = 7U;
    arm_station_test_last_status = ArmStationTest_Goto(2U, 1U);
    if (arm_station_test_last_status != ARM_STATUS_OK) { return; }

    /* step 8: common stretch position. */
    arm_station_test_move_seq = 8U;
    arm_station_test_last_status = ArmStationTest_Goto(2U, 0U);
    if (arm_station_test_last_status != ARM_STATUS_OK) { return; }

    /* step 9: return to the fixed initial position. */
    target.upper_deg = arm_home.upper_deg;
    target.lower_deg = arm_home.lower_deg;
    arm_station_test_move_seq = 9U;
    arm_station_test_last_status = ArmStationTest_MoveInSteps(target);
}
#endif

#if ARM_SERIAL_JOG
/*
 * Serial jog tool for calibrating the parallel arm.
 *
 * USART6 PC6(TX)/PC7(RX), 115200 8N1. Single characters execute immediately,
 * no CR/LF required, and every move prints the resulting software joint angles
 * so positions can be logged straight from the terminal.
 *
 *   1 / 2 : address 5  +step / -step
 *   3 / 4 : address 6  +step / -step
 *   a / d : both axes  +step / -step (coordinated)
 *   p     : print current angles
 *   b     : mark current pose as a ring reference; POS then shows offsets
 *   B     : clear the ring reference
 *   z     : declare the CURRENT pose as the software zero (does not move)
 *   h     : drive back to the software zero
 *   s     : stop now
 *   + / - : step size x2 / /2   (0.25 .. 20 deg)
 *   ?     : help
 */
#define ARM_JOG_STEP_MIN_DEG    0.25f
#define ARM_JOG_STEP_MAX_DEG    20.0f
#define ARM_JOG_DEFAULT_STEP    2.0f
#define ARM_JOG_DEFAULT_RPM     20U
#define ARM_JOG_ACCEL           20U

/*
 * 碰撞回零参数（驱动器手册：回零模式 2 = 多圈无限位碰撞回零）。
 *
 * 原理：电机以 o_vel 低速朝 o_dir 方向转，顶到机械硬限位后，
 * 当实际转速低于 sl_vel、电流超过 sl_ma 并持续 sl_ms 毫秒，
 * 驱动器判定"撞到了"，把该位置定义为零点。
 *
 * !!! 下面四个“待实测”的值是保守占位值，必须先空载/低速试出来 !!!
 * 电流阈值设太大 = 顶死时用力过猛会崩连杆；设太小 = 正常运行就误触发。
 * 请对照手册"堵转保护参数"一节确认 sl_vel / sl_ma / sl_ms 的量纲和推荐值。
 */
#define ARM_HOME_MODE           2U        /* 2 = 多圈无限位碰撞回零 */
#define ARM_HOME_VEL_RPM        10U       /* 回零速度(RPM)，必须低速 */
#define ARM_HOME_TIMEOUT_MS     30000UL   /* 回零超时(ms) */
#define ARM_HOME_STALL_VEL      100U      /* 碰撞检测转速(RPM)，待实测 */
#define ARM_HOME_STALL_MA       800U      /* 碰撞检测电流(mA)，待实测 */
#define ARM_HOME_STALL_MS       500U      /* 碰撞检测时间(ms)，待实测 */
#define ARM_HOME_DIR_UPPER      0U        /* 地址5 回零方向，实测定 */
#define ARM_HOME_DIR_LOWER      0U        /* 地址6 回零方向，实测定 */
/*
 * potF = 驱动器上电自动回零。
 * 并联机构两个轴同时自动回零会互相较劲，默认关闭，先手动逐个回零验证。
 */
#define ARM_HOME_AUTO_ON_POWER  0

volatile uint32_t arm_jog_stage = 0U;
volatile uint32_t arm_jog_cmd_count = 0U;
volatile uint32_t arm_jog_move_count = 0U;
volatile int32_t  arm_jog_last_status = ARM_STATUS_OK;
volatile float    arm_jog_step_deg = ARM_JOG_DEFAULT_STEP;
volatile float    arm_jog_last_upper_deg = 0.0f;
volatile float    arm_jog_last_lower_deg = 0.0f;
static uint8_t     arm_jog_ring_reference_valid = 0U;
static ArmJointAngles arm_jog_ring_reference;

static void ArmJog_Send(const char *s)
{
    while (*s) {
        while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART6, (uint8_t)*s++);
    }
}

/* Two decimal places preserve the 0.25-degree jog increments. */
static void ArmJog_SendFixed2(float v)
{
    char buf[32];
    long hundredths = (long)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
    long magnitude = (hundredths < 0) ? -hundredths : hundredths;
    sprintf(buf, "%s%ld.%02ld", (hundredths < 0) ? "-" : "",
            magnitude / 100, magnitude % 100);
    ArmJog_Send(buf);
}

static void ArmJog_PrintPos(void)
{
    ArmJointAngles cur;
    char buf[24];

    arm_jog_last_status = Arm_GetCurrentJointAngles(&cur);
    if (arm_jog_last_status != ARM_STATUS_OK) {
        ArmJog_Send("POS no-reference (send z first)\r\n");
        return;
    }
    arm_jog_last_upper_deg = cur.upper_deg;
    arm_jog_last_lower_deg = cur.lower_deg;
    ArmJog_Send("POS 5=");
    ArmJog_SendFixed2(cur.upper_deg);
    ArmJog_Send(" 6=");
    ArmJog_SendFixed2(cur.lower_deg);
    ArmJog_Send(" deg  step=");
    ArmJog_SendFixed2(arm_jog_step_deg);
    if (arm_jog_ring_reference_valid) {
        ArmJog_Send("  from-ring 5=");
        ArmJog_SendFixed2(cur.upper_deg - arm_jog_ring_reference.upper_deg);
        ArmJog_Send(" 6=");
        ArmJog_SendFixed2(cur.lower_deg - arm_jog_ring_reference.lower_deg);
    }
    sprintf(buf, "  LIGHT %u%%\r\n", (unsigned)Light_GetPercent());
    ArmJog_Send(buf);
}

static void ArmJog_Move(float delta_upper, float delta_lower)
{
    ArmJointAngles cur;
    arm_jog_last_status = Arm_GetCurrentJointAngles(&cur);
    if (arm_jog_last_status != ARM_STATUS_OK) {
        ArmJog_Send("ERR no reference: send z at the known pose first\r\n");
        return;
    }
    arm_jog_last_status = Arm_MoveJoint(cur.upper_deg + delta_upper,
                                        cur.lower_deg + delta_lower,
                                        ARM_JOG_DEFAULT_RPM, ARM_JOG_ACCEL);
    if (arm_jog_last_status != ARM_STATUS_OK) {
        ArmJog_Send("ERR move command failed\r\n");
        return;
    }
    ++arm_jog_move_count;
    ArmJog_PrintPos();
}

static void ArmJog_SetEnable(bool state)
{
    Emm_V5_En_Control(ARM_ADDR_UPPER, state, false);
    delay_ms(20);
    Emm_V5_En_Control(ARM_ADDR_LOWER, state, false);
    delay_ms(20);
    ArmJog_Send(state ? "ENABLED both axes\r\n"
                      : "RELEASED both axes (free to move by hand)\r\n");
}

/*
 * 配置并触发单个地址的碰撞回零。
 * 顺序：写参数(存驱动器flash) -> 解除堵转保护 -> 触发回零。
 * 回零是驱动器自己执行的，函数立即返回；必须等机构停稳后再发 z 同步软件基准。
 */
static void ArmJog_HomeAxis(uint8_t addr, uint8_t dir, const char *name)
{
    char buf[80];

    Emm_V5_Origin_Modify_Params(addr, true,
                                ARM_HOME_MODE, dir,
                                ARM_HOME_VEL_RPM, ARM_HOME_TIMEOUT_MS,
                                ARM_HOME_STALL_VEL, ARM_HOME_STALL_MA,
                                ARM_HOME_STALL_MS,
                                ARM_HOME_AUTO_ON_POWER ? true : false);
    if (!can_tx_last_cmd_ok) { ArmJog_Send("ERR home params tx failed\r\n"); return; }
    delay_ms(50);

    Emm_V5_Reset_Clog_Pro(addr);
    if (!can_tx_last_cmd_ok) { ArmJog_Send("ERR unclog tx failed\r\n"); return; }
    delay_ms(50);

    Emm_V5_Origin_Trigger_Return(addr, ARM_HOME_MODE, false);
    if (!can_tx_last_cmd_ok) { ArmJog_Send("ERR home trigger tx failed\r\n"); return; }

    sprintf(buf, "HOME %s (addr %u) triggered: %u RPM dir=%u, wait for stop then send z\r\n",
            name, (unsigned)addr, (unsigned)ARM_HOME_VEL_RPM, (unsigned)dir);
    ArmJog_Send(buf);
}

/* 只把回零参数写进两个驱动器的 flash，不触发运动。 */
static void ArmJog_WriteHomeParams(void)
{
    Emm_V5_Origin_Modify_Params(ARM_ADDR_UPPER, true,
                                ARM_HOME_MODE, ARM_HOME_DIR_UPPER,
                                ARM_HOME_VEL_RPM, ARM_HOME_TIMEOUT_MS,
                                ARM_HOME_STALL_VEL, ARM_HOME_STALL_MA,
                                ARM_HOME_STALL_MS,
                                ARM_HOME_AUTO_ON_POWER ? true : false);
    if (!can_tx_last_cmd_ok) { ArmJog_Send("ERR addr5 params tx failed\r\n"); return; }
    delay_ms(50);

    Emm_V5_Origin_Modify_Params(ARM_ADDR_LOWER, true,
                                ARM_HOME_MODE, ARM_HOME_DIR_LOWER,
                                ARM_HOME_VEL_RPM, ARM_HOME_TIMEOUT_MS,
                                ARM_HOME_STALL_VEL, ARM_HOME_STALL_MA,
                                ARM_HOME_STALL_MS,
                                ARM_HOME_AUTO_ON_POWER ? true : false);
    if (!can_tx_last_cmd_ok) { ArmJog_Send("ERR addr6 params tx failed\r\n"); return; }

    ArmJog_Send("HOME params stored to both drivers (no motion)\r\n");
}

/* ---------------- 补光灯 PA6 / TIM3_CH1 ---------------- */
#define ARM_JOG_LIGHT_STEP_PERCENT 10
#define ARM_JOG_LIGHT_BOOT_PERCENT 20U

static void ArmJog_PrintLight(void)
{
    char buf[40];
    uint8_t percent = Light_GetPercent();

    sprintf(buf, "LIGHT %u%%  CCR %u/%u\r\n", (unsigned)percent,
            (unsigned)((uint32_t)percent * LIGHT_PWM_PERIOD / 100U),
            (unsigned)LIGHT_PWM_PERIOD);
    ArmJog_Send(buf);
}

static void ArmJog_LightStep(int16_t delta)
{
    int16_t level = (int16_t)Light_GetPercent() + delta;

    if (level < 0) { level = 0; }
    if (level > 100) { level = 100; }
    Light_SetPercent((uint8_t)level);
    ArmJog_PrintLight();
}

/* 0→100→0 渐变，用来肉眼确认 PWM 调光有效（不是只有开/关）。 */
static void ArmJog_LightSweep(void)
{
    uint8_t level;

    ArmJog_Send("LIGHT sweep 0->100->0 ...\r\n");
    for (level = 0U; level <= 100U; level += 10U) {
        Light_SetPercent(level);
        delay_ms(150U);
    }
    for (level = 100U; level > 0U; level -= 10U) {
        Light_SetPercent((uint8_t)(level - 10U));
        delay_ms(150U);
    }
    ArmJog_PrintLight();
}

static void ArmJog_Help(void)
{
    ArmJog_Send("== jog ==\r\n");
    ArmJog_Send("1:5+  2:5-  3:6+  4:6-  a:both+  d:both-\r\n");
    ArmJog_Send("p:pos  z:zero-here  h:goto-zero  s:stop\r\n");
    ArmJog_Send("b:mark ring reference  B:clear ring reference\r\n");
    ArmJog_Send("+:step x2  -:step /2  ?:help\r\n");
    ArmJog_Send("== homing / enable ==\r\n");
    ArmJog_Send("r:home addr5   R:home addr6   q:store params only\r\n");
    ArmJog_Send("i:abort homing  u:clear stall-protect\r\n");
    ArmJog_Send("e:enable both   E:release both\r\n");
    ArmJog_Send("== fill light (PA6) ==\r\n");
    ArmJog_Send("l:+10%  L:-10%  o:off  f:full  g:sweep test\r\n");
}

static void ArmJog_Command(uint8_t c)
{
    float step = arm_jog_step_deg;

    ++arm_jog_cmd_count;
    switch (c) {
    case '1': ArmJog_Move( step,  0.0f); break;
    case '2': ArmJog_Move(-step,  0.0f); break;
    case '3': ArmJog_Move(0.0f,   step); break;
    case '4': ArmJog_Move(0.0f,  -step); break;
    case 'a': ArmJog_Move( step,  step); break;
    case 'd': ArmJog_Move(-step, -step); break;
    case 'p': ArmJog_PrintPos(); break;
    case 'b':
        if (Arm_GetCurrentJointAngles(&arm_jog_ring_reference) != ARM_STATUS_OK) {
            ArmJog_Send("ERR no reference: send z first\r\n");
        } else {
            arm_jog_ring_reference_valid = 1U;
            ArmJog_Send("RING reference saved\r\n");
            ArmJog_PrintPos();
        }
        break;
    case 'B':
        arm_jog_ring_reference_valid = 0U;
        ArmJog_Send("RING reference cleared\r\n");
        ArmJog_PrintPos();
        break;
    case 'z':
        Arm_SetCurrentJointAngles(0.0f, 0.0f);
        arm_jog_ring_reference_valid = 0U;
        ArmJog_Send("ZERO set at current pose (no movement)\r\n");
        ArmJog_PrintPos();
        break;
    case 'h':
        arm_jog_last_status = Arm_MoveJoint(0.0f, 0.0f,
                                            ARM_JOG_DEFAULT_RPM, ARM_JOG_ACCEL);
        ArmJog_PrintPos();
        break;
    case 's':
        arm_jog_last_status = Arm_Stop();
        ArmJog_Send("STOP sent\r\n");
        break;
    case '+':
        arm_jog_step_deg *= 2.0f;
        if (arm_jog_step_deg > ARM_JOG_STEP_MAX_DEG) {
            arm_jog_step_deg = ARM_JOG_STEP_MAX_DEG;
        }
        ArmJog_PrintPos();
        break;
    case '-':
        arm_jog_step_deg *= 0.5f;
        if (arm_jog_step_deg < ARM_JOG_STEP_MIN_DEG) {
            arm_jog_step_deg = ARM_JOG_STEP_MIN_DEG;
        }
        ArmJog_PrintPos();
        break;
    case 'r': ArmJog_HomeAxis(ARM_ADDR_UPPER, ARM_HOME_DIR_UPPER, "addr5"); break;
    case 'R': ArmJog_HomeAxis(ARM_ADDR_LOWER, ARM_HOME_DIR_LOWER, "addr6"); break;
    case 'q': ArmJog_WriteHomeParams(); break;
    case 'i':
        Emm_V5_Origin_Interrupt(ARM_ADDR_UPPER);
        delay_ms(20);
        Emm_V5_Origin_Interrupt(ARM_ADDR_LOWER);
        ArmJog_Send("HOME abort sent to both\r\n");
        break;
    case 'u':
        Emm_V5_Reset_Clog_Pro(ARM_ADDR_UPPER);
        delay_ms(20);
        Emm_V5_Reset_Clog_Pro(ARM_ADDR_LOWER);
        ArmJog_Send("stall protection cleared (both)\r\n");
        break;
    case 'e': ArmJog_SetEnable(true); break;
    case 'E': ArmJog_SetEnable(false); break;
    case 'l': ArmJog_LightStep((int16_t)ARM_JOG_LIGHT_STEP_PERCENT); break;
    case 'L': ArmJog_LightStep(-(int16_t)ARM_JOG_LIGHT_STEP_PERCENT); break;
    case 'o': Light_SetPercent(0U); ArmJog_PrintLight(); break;
    case 'f': Light_SetPercent(100U); ArmJog_PrintLight(); break;
    case 'g': ArmJog_LightSweep(); break;
    case '?': ArmJog_Help(); break;
    default:  ArmJog_Send("ERR unknown command; send ? for help\r\n"); break;
    }
}

static void ArmJog_InitUart(void)
{
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    GPIO_StructInit(&g);
    g.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &g);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    USART_StructInit(&u);
    u.USART_BaudRate = 115200;
    USART_Init(USART6, &u);
    USART_Cmd(USART6, ENABLE);
}

static void ArmJog_Run(void)
{
    while (1) {
        uint32_t status = USART6->SR;

        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
            volatile uint32_t ignored = USART6->DR;  /* clear the error by reading */
            (void)ignored;
        } else if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) != RESET) {
            uint8_t b = (uint8_t)USART_ReceiveData(USART6);
            if (b >= 32U && b <= 126U) { ArmJog_Command(b); }
            /* CR/LF are ignored: every printable key acts immediately. */
        } else {
            delay_us(10);
        }
    }
}
#endif

#if CHASSIS_CAN_MOTOR_TEST
/* 在 Keil Watch 中观察：循环次数、是否收到回帧、CAN 错误与发送状态。 */
volatile uint32_t can_test_internal_loopback = CAN_INTERNAL_LOOPBACK_TEST;
/* 仅在内部回环模式有效：0 等待结果；1 收发通过；2 连续五轮仍未通过。 */
volatile uint32_t can_test_loopback_result = 0;
volatile uint32_t can_test_stage = 0;
volatile uint32_t can_test_init_ok = 0;
volatile uint32_t can_test_loop_count = 0;
volatile uint32_t can_test_rx_count = 0;
volatile uint32_t can_test_tx_no_mailbox_count = 0;
volatile uint32_t can_test_last_rx_id = 0;
volatile uint32_t can_test_error_reg = 0;
volatile uint32_t can_test_tx_error_count = 0;
volatile uint32_t can_test_bus_off = 0;
volatile uint32_t can_test_tx_reg = 0;
volatile uint32_t can_test_tx_ok_count = 0;
volatile uint32_t can_test_tx_failed_count = 0;
volatile uint32_t can_test_tx_timeout_count = 0;

/*
 * 轮位与原 yyb_move.c 保持一致：前左地址 2、前右地址 1、
 * 后左地址 3、后右地址 4。左右安装方向相反，所以正向滚动的
 * 电机方向分别为 0、1、0、1。速度只用 30 RPM，先架空车轮测试。
 */
static void ChassisCanMotorTest_Enable(void)
{
    Emm_V5_En_Control(2, true, false);
    delay_ms(10);
    Emm_V5_En_Control(1, true, false);
    delay_ms(10);
    Emm_V5_En_Control(3, true, false);
    delay_ms(10);
    Emm_V5_En_Control(4, true, false);
    delay_ms(10);
}

static void ChassisCanMotorTest_SendSpeed(void)
{
    /* false 表示立即执行，不等待同步启动命令。 */
    Emm_V5_Vel_Control(2, 0, 30, 30, false);
    delay_ms(10);
    Emm_V5_Vel_Control(1, 1, 30, 30, false);
    delay_ms(10);
    Emm_V5_Vel_Control(3, 0, 30, 30, false);
    delay_ms(10);
    Emm_V5_Vel_Control(4, 1, 30, 30, false);
}
#endif

//x减物块右移（从外侧往内看）//y+物块往地图内侧走
#define  x_center_red       955
#define  y_center_red		519

#define  x_center_green		955
#define  y_center_green		519

#define  x_center_blue		955
#define  y_center_blue		519

char tjcstr[100];
int acc[4];
float direct_x = 0;
float direct_y = 0;
float distance_wukuai = 150;
float x_origin = 0;
float y_origin = 0;
int task[6]; //存储颜色信息
int ypcode[6]; //存储颜色信息
float angle = 0;

void transfer_position_wukuai(uint8_t task_choose,float distance,uint8_t color);
void transfer_position_sehuan(uint8_t fenbian_choose,uint8_t task_num);

void fangzhi(uint8_t task); //色环位置放置物块
void fangzhi_maduo(uint8_t task);  //码垛位置放置物块
void na(uint8_t task);   //色环位置拿物块

void yuantai_na(uint8_t task);//圆盘位置拿物块
void yuantai_na1(uint8_t task);//圆盘位置拿物块，识别就位
void yuantai_na2(uint8_t task);//圆盘位置拿物块，识别到物块静止，此时抓取

void wait_send_shumeipai(uint8_t message,uint16_t time);
void clear_ckp(void);
void jixie_reset(void);
char message[] = "Hello, BT - 04 from STM32F407!\r\n";
volatile uint32_t chassis_imu_config_fail_mask = 0U;

/* HWT101 显式输出角度和角速度，避免20 Hz控制环反复使用默认低频旧数据。 */
static void ChassisImu_ConfigureYawOutput(void)
{
    uint32_t fail_mask = 0U;

    if (WitSetOutputRate(RRATE_100HZ) != WIT_HAL_OK) { fail_mask |= 0x01U; }
    if (WitSetContent(RSW_ANGLE | RSW_GYRO) != WIT_HAL_OK) { fail_mask |= 0x02U; }
    if (WitSetBandwidth(BANDWIDTH_184HZ) != WIT_HAL_OK) { fail_mask |= 0x04U; }
    chassis_imu_config_fail_mask = fail_mask;
    delay_ms1(50);
}

#if CHASSIS_POWER_ON_TEST && !CHASSIS_CAN_MOTOR_TEST
volatile uint32_t chassis_route_stage = 0U; /* 1 waiting HWT, 2 waiting key, 3 running, 4 finished, E1 CAN init error, E2 HWT zero error. */
volatile uint32_t chassis_route_run_count = 0U;
volatile float chassis_route_yaw_target = 0.0f;
volatile float chassis_turn_start_yaw = 0.0f;
volatile float chassis_turn_end_yaw = 0.0f;
volatile float chassis_turn_measured_delta = 0.0f;

static void ChassisRoute_WaitForKey(void)
{
    /* A key held at boot or after a route must be released first. */
    for (;;) {
        while (swith_out() == 0U) { delay_ms1(10); }
        while (swith_out() != 0U) { delay_ms1(10); }
        delay_ms1(30);
        if (swith_out() != 0U) { continue; }
        while (swith_out() == 0U) { delay_ms1(10); }
        delay_ms1(30);
        if (swith_out() != 0U) { return; }
    }
}

static float ChassisRoute_WrapYaw(float yaw_deg)
{
    while (yaw_deg > 180.0f) { yaw_deg -= 360.0f; }
    while (yaw_deg < -180.0f) { yaw_deg += 360.0f; }
    return yaw_deg;
}

/*
 * 副工程每次按键都先以当前朝向校零，再转固定90°。主路线不能中途反复
 * 改写HWT101零点，因此读取转弯前的新航向，并在它的基础上生成相对目标。
 */
static void ChassisRoute_TurnBy(float delta_yaw_deg)
{
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    delay_ms1(100);
    imu_scan(fAcc, fGyro, fAngle);
    chassis_turn_start_yaw = fAngle[2];
    chassis_route_yaw_target = ChassisRoute_WrapYaw(chassis_turn_start_yaw + delta_yaw_deg);

    car_turn_to_stable(chassis_route_yaw_target);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    delay_ms1(100);
    imu_scan(fAcc, fGyro, fAngle);
    chassis_turn_end_yaw = fAngle[2];
    chassis_turn_measured_delta = ChassisRoute_WrapYaw(chassis_turn_end_yaw - chassis_turn_start_yaw);
}

/*
 * 速度模式横移结束后，车体仍可能带着横摆惯性。PID_move 到时只负责停车，
 * 不会继续把航向拉回目标值；这里等车体停稳，再用低速无积分闭环回正。
 */
static void ChassisRoute_LateralMove(int v_x, uint32_t time_ms, float yaw_deg)
{
    PID_move(v_x, 0, 0, time_ms, yaw_deg, 1);
    car_move(0, 0, 0);
    delay_ms1(200);
    PID_turn_stable(yaw_deg, 2000);
}

static void ChassisOriginalFixedRoute_Run(void)
{
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    delay_ms1(100);
    imu_scan(fAcc, fGyro, fAngle);
    chassis_route_yaw_target = fAngle[2];

    /* 第一轮：起点 -> 扫码区 -> 原料区。 */
    car_move2(30, 0, 100, 100); /* 按当前实车轮向，正X为左横移：开局左移30 mm。 */
    car_move1(0, -1050, 150, 200);
    car_move(0, 0, 0);
    delay_ms1(1000);
    car_move1(0, -950, 150, 200);
    ChassisRoute_TurnBy(90.0f); /* 路线航向 -90° -> 0°，左转90° */
    car_move1(0, -980, 150, 200);
#if ROUTE_START_TRIM
    car_move2(-30, 0, 100, 100);
#endif

    /* 原料区 -> 粗加工区。 */
#if ROUTE_START_TRIM
    ChassisRoute_LateralMove(-30, 400, chassis_route_yaw_target);
#endif
    ChassisRoute_TurnBy(90.0f);
    car_move1(0, -1800, 150, 250);
    ChassisRoute_TurnBy(90.0f);

    /* 粗加工区 -> 色环区。 */
    ChassisRoute_LateralMove(-30, 300, chassis_route_yaw_target);
    car_move1(140, 140 + 850, 180, 180);
    ChassisRoute_TurnBy(-90.0f);
    car_move1(140, 140 + 790, 180, 150);

    /* 色环区 -> 圆盘区。 */
    ChassisRoute_LateralMove(-30, 300, chassis_route_yaw_target);
    car_move1(140, 220 + 690, 180, 180);
    ChassisRoute_TurnBy(-90.0f);
    car_move1(140, 370 + 20, 180, 180);

    /* 第二轮：从第一轮终点继续。 */
//    ChassisRoute_LateralMove(10, 300, chassis_route_yaw_target);

    /* 第二轮原料区 -> 粗加工区。 */
    ChassisRoute_LateralMove(-30, 400, chassis_route_yaw_target);
    car_move1(0, 400, 150, 200);
    ChassisRoute_TurnBy(90.0f);
    car_move1(0, -1700, 150, 250);
    ChassisRoute_TurnBy(90.0f);

    /* 第二轮粗加工区 -> 码垛区。 */
    ChassisRoute_LateralMove(-30, 300, chassis_route_yaw_target);
    car_move1(140, 140 + 680, 180, 180);
    ChassisRoute_TurnBy(-90.0f);
    car_move1(140, 140 + 620, 180, 150);

    /* 码垛区 -> 终点。 */
    ChassisRoute_LateralMove(-30, 300, chassis_route_yaw_target);
    car_move1(140, 220 + 650, 180, 180);
    ChassisRoute_TurnBy(-90.0f);
    car_move1(140, 2000 - 250, 180, 180);
    ChassisRoute_TurnBy(-90.0f); /* 路线航向0° -> -90°，右转90° */
    car_move1(0, 2000, 180, 180);
    car_move(0, 0, 0);
    delay_ms1(200);
    PID_turn_stable(chassis_route_yaw_target, 2000); /* 最后一段位置运动后再把残余偏角拉回。 */

}
#endif

#if PA2_PA3_SERVO_TEST
/* Current assembly: PA2 drives the claw, PA3 drives the lift; PA1 stays idle. */
#define PA2_TEST_HOME_US 648U
#define PA2_TEST_TARGET_US 688U
#define PA3_TEST_HOME_US 838U
#define PA3_TEST_TARGET_US 878U
volatile uint32_t pa3_gimbal_test_stage = 0U;
volatile uint32_t pa3_gimbal_test_pulse_us = PA3_TEST_HOME_US;
volatile uint32_t pa3_gimbal_test_cycle_count = 0U;
volatile uint32_t pa2_servo_test_stage = 0U;
volatile uint32_t pa2_servo_test_pulse_us = PA2_TEST_HOME_US;
volatile uint32_t pa2_servo_test_cycle_count = 0U;

static void Pa2Pa3ServoTest_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

    /* Same 1 MHz timer tick and 50 Hz PWM as the existing PWM_Init(). */
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 84U - 1U;
    timer.TIM_Period = 20000U - 1U;
    TIM_TimeBaseInit(TIM2, &timer);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = PA2_TEST_HOME_US;  /* Existing PA2 startup pulse. */
    TIM_OC3Init(TIM2, &oc);
    oc.TIM_Pulse = PA3_TEST_HOME_US;  /* Existing PA3 startup pulse. */
    TIM_OC4Init(TIM2, &oc);
    TIM_Cmd(TIM2, ENABLE);
}

static void Pa3GimbalTest_MoveTo(uint16_t target_us)
{
    pa3_gimbal_test_pulse_us = target_us;
    TIM_SetCompare4(TIM2, target_us);
}

#if !PA3_FAST_SWEEP_TEST
static void Pa2ServoTest_MoveTo(uint16_t target_us)
{
    while (pa2_servo_test_pulse_us != target_us) {
        if (pa2_servo_test_pulse_us < target_us) {
            pa2_servo_test_pulse_us += 4U;
        } else {
            pa2_servo_test_pulse_us -= 4U;
        }
        TIM_SetCompare3(TIM2, (uint16_t)pa2_servo_test_pulse_us);
        delay_ms(40);
    }
}
#endif
#endif
int main(void)
{	
/*
	所有的中断：
		1.OLED每隔500ms更新角度数据（最好不启用，里面耗时很长）（会影响delay函数的调用正确与否）
		2.delayms1函数在启用时最好关闭所有中断
		3.PID每间隔50ms进行一次PID计算，花25ms给所有电机发送指令（耗时中等）
		4.IMU每次接受信息触发一次中断
		5.树莓派给stm32发送信息时触发中断
		6.步进电机到位时发送信息中断
		7.舵机控制每隔20ms定时控制角度
*/
	 delay_init(168);
#if ARM_ZERO_SETUP_IDLE
    /* Motion tasks are temporarily disabled for motor zero calibration.
     * No CAN init, motor enable/position command, or servo PWM is issued. */
    while (1) { delay_ms(1000U); }
#elif ARM_NEW_RING_PICKUP
    ArmVisionTask_RunNewRingPickup();
#elif POWER_BUTTON_DIAG
    Swith_Init();
    LED2_Init();
    while (1) {
        LED_Set2(swith_out() == 0U ? 1U : 0U);
        delay_ms(10U);
    }
#elif LIGHT_TEST
    /* PA6 fill light only: one 0-100% ramp, then hold 40% so dimming is visible. */
    Light_Init();
    {
        uint8_t level;
        while (1) {
            for (level = 0U; level <= 100U; level += 10U) {
                Light_SetPercent(level);
                delay_ms(400U);
            }
            delay_ms(500U);
            Light_SetPercent(40U);
            delay_ms(2000U);
        }
    }
#elif ARM_MATERIAL_TRACK_ONLY
    ArmVisionTask_RunMaterialTracking();
#elif ARM_MATERIAL_LEFT_TRANSFER
    ArmVisionTask_RunMaterialToLeftTray();
#elif ARM_RING_SEQUENCE
    ArmVisionTask_RunRingSequence();
#elif ARM_RING1_TRANSFER
    ArmVisionTask_RunRing1Transfer();
#elif SERVO_PICK_CYCLE
    uint16_t lift_us = ARM_BOOT_LIFT_US;
    uint16_t claw_us = ARM_BOOT_CLAW_US;
    ArmServos_Init();
    Swith_Init();
    ArmServos_Lift(lift_us);
    ArmServos_Claw(claw_us);
    ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
    ArmServos_ClawSlow(&claw_us, ARM_CLAW_OPEN_US);
    delay_ms(1000U);
    /* A key held during power-up must be released before the first cycle. */
    while (swith_out() == 0U) { delay_ms(10U); }
    while (1) {
        while (swith_out() != 0U) { delay_ms(10U); }
        delay_ms(30U);
        if (swith_out() != 0U) { continue; }
        while (swith_out() == 0U) { delay_ms(10U); }
        delay_ms(30U);
        if (swith_out() == 0U) { continue; }

        ArmServos_LiftSlow(&lift_us, ARM_GROUND_HEIGHT_US);
        delay_ms(1000U);
        ArmServos_ClawSlow(&claw_us, ARM_CLAW_CLOSE_US);
        delay_ms(1000U);
        ArmServos_LiftSlow(&lift_us, ARM_LIFT_RELEASE_US);
        delay_ms(1000U);
        ArmServos_ClawSlow(&claw_us, ARM_CLAW_OPEN_US);
        delay_ms(1000U);
        ArmServos_LiftSlow(&lift_us, ARM_LIFT_UP_US);
        delay_ms(1000U);
    }
#elif SERVO_UART6_CALIBRATION
    ServoCalibrationRun();
#elif OPENMV_UART_LED_TEST
    /* Receive-only validation. CAN, motors, servos and PC9 stay inactive. */
    LED2_Init();
    OpenMvUartTest_Run();
#elif ARM_VISION_TASK
    ArmVisionTask_Run();
#elif ARM_DIRECTION_TEST
    /*
     * This branch intentionally skips the chassis, IMU, PID, OLED and servos.
     * Power-on never issues a position command; the first movement requires PC9.
     */
    arm_direction_test_stage = 1U;
    board_init();
    if (!board_can_init_ok) {
        arm_direction_test_stage = 0xE1U;
        while (1) { delay_ms(1000); }
    }

    Swith_Init();
    LED2_Init();
    delay_ms(2000); /* Let both motor-side CAN modules finish powering up. */

    Emm_V5_En_Control(ARM_ADDR_UPPER, true, false);
    if (!can_tx_last_cmd_ok) {
        arm_direction_test_stage = 0xE2U;
        while (1) { delay_ms(1000); }
    }
    delay_ms(20);
    Emm_V5_En_Control(ARM_ADDR_LOWER, true, false);
    if (!can_tx_last_cmd_ok) {
        arm_direction_test_stage = 0xE2U;
        while (1) { delay_ms(1000); }
    }

    /* Local relative zero only; this call does not move either motor. */
    Arm_SetCurrentJointAngles(0.0f, 0.0f);
    arm_direction_test_stage = 2U;

    while (1) {
        ArmDirectionTest_WaitForPress();
        arm_direction_test_stage = 3U + (arm_direction_test_press_count & 0x03U);
        arm_direction_test_last_status =
            ArmDirectionTest_RunStep(arm_direction_test_press_count);

        if (can.rxFrameFlag) {
            can.rxFrameFlag = false;
            arm_direction_test_last_rx_id = can.CAN_RxMsg.ExtId;
        }
        if (arm_direction_test_last_status != ARM_STATUS_OK) {
            arm_direction_test_stage = 0xE3U;
            Arm_Stop();
            while (1) { delay_ms(1000); }
        }

        ++arm_direction_test_press_count;
        arm_direction_test_stage = 2U;
    }
#elif ARM_STATION_TEST
    /*
     * Three-station retract/extend cycle. Hardware must be manually aligned
     * to the home position before each power-up; this branch only initializes
     * CAN and the arm motors — no chassis, IMU, PID, OLED or servos.
     */
    arm_station_test_stage = 1U;
    board_init();
    if (!board_can_init_ok) {
        arm_station_test_stage = 0xE1U;
        while (1) { delay_ms(1000); }
    }

    Swith_Init();
    LED2_Init();
    delay_ms(2000); /* Let motor-side CAN modules power up. */
    Timer6_Init();  /* Required by delay_ms1 in ArmStationTest_MoveBlocking. */

    Emm_V5_En_Control(ARM_ADDR_UPPER, true, false);
    if (!can_tx_last_cmd_ok) {
        arm_station_test_stage = 0xE2U;
        while (1) { delay_ms(1000); }
    }
    delay_ms(20);
    Emm_V5_En_Control(ARM_ADDR_LOWER, true, false);
    if (!can_tx_last_cmd_ok) {
        arm_station_test_stage = 0xE2U;
        while (1) { delay_ms(1000); }
    }

    /*
     * IMPORTANT: The arm must already be mechanically placed at the home
     * position. This call tells the software "you are at home angles now"
     * and does NOT move either motor.
     */
    Arm_SetCurrentJointAngles(arm_home.upper_deg, arm_home.lower_deg);
    arm_station_test_stage = 2U;  /* Ready for first PC9 press. */

    while (1) {
        ArmStationTest_WaitForPress();
        arm_station_test_stage = 3U;
        arm_station_test_current_station = 0U;
        arm_station_test_move_seq = 0U;

        LED_Set2(1U);
        ArmStationTest_RunAll();
        LED_Set2(0U);

        if (arm_station_test_last_status != ARM_STATUS_OK) {
            arm_station_test_stage = 0xE3U;
            Arm_Stop();
            while (1) { delay_ms(1000); }
        }

        ++arm_station_test_press_count;
        arm_station_test_stage = 2U;
    }
#elif ARM_SERIAL_JOG
    /*
     * Serial jog entry: only delay, CAN1, USART6 and addresses 5/6.
     * No chassis, no IMU, no PID, no OLED, no servos.
     */
    arm_jog_stage = 1U;
    board_init();
    if (!board_can_init_ok) {
        arm_jog_stage = 0xE1U;
        while (1) { delay_ms(1000); }
    }

    LED2_Init();
    ArmJog_InitUart();
    delay_ms(2000); /* Let both motor-side CAN modules finish powering up. */

    Emm_V5_En_Control(ARM_ADDR_UPPER, true, false);
    if (!can_tx_last_cmd_ok) {
        arm_jog_stage = 0xE2U;
        while (1) { delay_ms(1000); }
    }
    delay_ms(20);
    Emm_V5_En_Control(ARM_ADDR_LOWER, true, false);
    if (!can_tx_last_cmd_ok) {
        arm_jog_stage = 0xE2U;
        while (1) { delay_ms(1000); }
    }

    ArmJog_Send("\r\nARM JOG READY  (5/6 only; chassis, IMU, PID, servos off)\r\n");
    ArmJog_Send("Put the arm at a known safe pose, then send z to set zero.\r\n");
    Light_SetPercent(ARM_JOG_LIGHT_BOOT_PERCENT);
    ArmJog_PrintLight();
    ArmJog_Help();
    arm_jog_stage = 2U;
    ArmJog_Run();
#elif PA2_PA3_SERVO_TEST
    Pa2Pa3ServoTest_Init();
    pa3_gimbal_test_stage = 1U;
    pa2_servo_test_stage = 1U;
    delay_ms(1500);
#if PA3_FAST_SWEEP_TEST
    while (1) {
        Pa3GimbalTest_MoveTo(PA3_TEST_TARGET_US);
        pa3_gimbal_test_stage = 2U;
        delay_ms(700);
        Pa3GimbalTest_MoveTo(PA3_TEST_HOME_US);
        pa3_gimbal_test_stage = 3U;
        ++pa3_gimbal_test_cycle_count;
        delay_ms(700);
    }
#else
    while (1) {
        Pa3GimbalTest_MoveTo(PA3_TEST_TARGET_US);
        pa3_gimbal_test_stage = 2U;
        delay_ms(1000);
        Pa3GimbalTest_MoveTo(PA3_TEST_HOME_US);
        pa3_gimbal_test_stage = 3U;
        ++pa3_gimbal_test_cycle_count;
        delay_ms(500);
        Pa2ServoTest_MoveTo(PA2_TEST_TARGET_US);
        pa2_servo_test_stage = 2U;
        delay_ms(1000);
        Pa2ServoTest_MoveTo(PA2_TEST_HOME_US);
        pa2_servo_test_stage = 3U;
        ++pa2_servo_test_cycle_count;
        delay_ms(500);
    }
#endif
#elif CHASSIS_CAN_MOTOR_TEST
    /*
     * 这一路测试跳过 HWT101、PID 和定位路线，避免角度未就绪时
     * 卡在原来的等待循环。速度模式发令后电机会持续转动。
     */
    can_test_stage = 1;  /* 已进入四轮 CAN 测试入口。 */
    board_init();
    can_test_init_ok = board_can_init_ok;
    if (!can_test_init_ok) {
        can_test_stage = 0xE1U;
        while (1) { delay_ms(1000); }
    }
    can_test_stage = 2;  /* CAN 初始化完成。 */
    delay_ms(2000); /* Wait for motor-side CAN modules to power up. */
    ChassisCanMotorTest_Enable();
    can_test_stage = 3;  /* 四个使能命令已提交。 */
    delay_ms(100);
    while (1) {
        ChassisCanMotorTest_SendSpeed();
        can_test_stage = 4;  /* 四个速度命令已提交，开始持续循环。 */
        ++can_test_loop_count;
        /* 中断里的计数是实际收到的 CAN 帧数；最后帧 ID 用于核对电机地址。 */
        can_test_rx_count = can_rx_frame_count;
        can_test_tx_no_mailbox_count = can_tx_no_mailbox_count;
        can_test_tx_ok_count = can_tx_ok_count;
        can_test_tx_failed_count = can_tx_failed_count;
        can_test_tx_timeout_count = can_tx_timeout_count;
        if (can.rxFrameFlag) {
            can.rxFrameFlag = false;
            can_test_last_rx_id = can.CAN_RxMsg.ExtId;
        }
        /* ESR 可看发送/接收错误计数及 Bus-Off；TSR 可看发送邮箱状态。 */
        can_test_error_reg = CAN1->ESR;
        can_test_tx_error_count = (can_test_error_reg >> 16) & 0xFFU;
        can_test_bus_off = (can_test_error_reg & CAN_ESR_BOFF) ? 1U : 0U;
        can_test_tx_reg = CAN1->TSR;
#if CAN_INTERNAL_LOOPBACK_TEST
        /*
         * 回环帧不经过收发器和外部电机。实际收到回环帧且没有发送错误，
         * 才能说明 MCU 内部 CAN 控制器、过滤器和接收中断这段链路正常。
         */
        if (can_test_rx_count > 0U &&
            can_test_tx_no_mailbox_count == 0U &&
            can_test_tx_error_count == 0U &&
            can_test_bus_off == 0U) {
            can_test_loopback_result = 1U;
        } else if (can_test_loop_count >= 5U) {
            can_test_loopback_result = 2U;
        }
#endif
        delay_ms(500);
    }
#elif CHASSIS_POWER_ON_TEST
    board_init();        /* CAN1 drives the four chassis motors. */
    if (!board_can_init_ok) {
        chassis_route_stage = 0xE1U;
        while (1) { delay_ms(1000); }
    }
    Timer6_Init();       /* Position moves use TIM6-based delay_ms1. */
    FunctionLampKey_Init(); /* PA8 follows the physical PC9 level. */
    delay_ms1(1000);
    imu_init();          /* HWT101 supplies yaw on USART2. */
    ChassisImu_ConfigureYawOutput();
    PIDInit();           /* TIM4 updates yaw PID every 50 ms. */
    dianji_move_flag = 0;
    chassis_route_stage = 1U;
    while (!imu_angle_ready()) {
        delay_ms1(100);  /* Never start the route without an HWT101 angle. */
    }
    chassis_route_stage = 2U;
    while (1) {
        ChassisRoute_WaitForKey();

        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        mypid.integral = 0;
        mypid.error = 0;
        mypid.lastError = 0;
        if (WHT101_ANGLEZCali() != WIT_HAL_OK) {
            chassis_route_stage = 0xE2U;
            continue;
        }
        delay_ms1(300);  /* Let the route start from a fresh 0-degree heading. */
        dianji_move_flag = 1;
        chassis_route_stage = 3U;
        ChassisOriginalFixedRoute_Run(); /* Run both fixed-route task rounds. */
        dianji_move_flag = 0;
        car_move(0, 0, 0);   /* Command zero speed after the route. */
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        ++chassis_route_run_count;
        chassis_route_stage = 4U;
    }
#else
	board_init();  //配置can总线协议
	LED_Init();  //配置LED指示灯
	OLED_Init();   //暂时不启用OLED //OLED中断和TIM5绑定在一起（两个必须同时用）
	Timer6_Init(); //delayms1的时钟
	PWM_Init();	  //使用定时器二，开启中断，实时控制舵机角度
	//	uart5_init(9600);  //蓝牙，暂时不用
//	uart6_init(9600);//串口屏，暂时不用
//	Serial_Init();	//树莓派串口，暂时不用
	Swith_Init();   //开关初始化
	imu_init();     //使用UART2
	ChassisImu_ConfigureYawOutput();
	PIDInit();		//用定时器实现定时中断//周期为50ms//该中断最好不要被别的中断打断
	Key_EXTI_Init();//按键中断//向树莓派发送重启命令
    //    clear_ckp();    //清空串口屏，串口屏未初始化
	dianji_move_flag = 0;//四轮底盘已注释掉，PID不发送CAN底盘帧
	TIM_SetCompare1(TIM3,0);  //关闭补光灯
//**************************机械臂测试代码 (四轮已禁用) ***************************//
	while(swith_out() == 1){		
		LED_Toggle2();
		delay_ms1(50);
	}//树莓派已经准备好，按下开关即可启动
	move_all(-120.0f,1000,200   ,0,2000,200   ,278.0f,13);
//	set_zhuashou_kai();  //不开夹爪
	jixie_reset();
	while(1){}
//*************************第一轮*************************//
	while(1){
		LED_Toggle2();
		delay_ms1(200);
		if((Serial_RxPacket[0]==0x39) && (Serial_RxPacket[1]==0x38)){
			break;
	    }
	}
//**************************指示灯闪烁，可以开始***************************//
	while(swith_out() == 1){		
		LED_Toggle2();
		delay_ms1(50);
	}//树莓派已经准备好，按下开关即可启动
	mypid.integral = 0;
	WHT101_ANGLEZCali();//101z轴置零
	Serial_GetRxFlag();Serial_SendByte(0x31);
//***************************第一部分，识别二维码并显示，大致移动到原料区*********************//	
	//扫二维码
	car_move1(0,-530,150,200);
    //读取树莓派信息	
	while(1){
		if (Serial_GetRxFlag() == 1){   //收到数据包
			   sprintf(tjcstr, "t0.txt=\"%d%d%d+%d%d%d \"",
										Serial_RxPacket[0]-0x30,Serial_RxPacket[1]-0x30,Serial_RxPacket[2]-0x30,
										Serial_RxPacket[4]-0x30,Serial_RxPacket[5]-0x30,Serial_RxPacket[6]-0x30);
				HMISends(tjcstr);
	            HMISendb(0xff);//在显示屏上面显示任务
				break;
			  }
	}	
    task[0]= Serial_RxPacket[0]-0x30;task[1] = Serial_RxPacket[1]-0x30;task[2] = Serial_RxPacket[2]-0x30;
	task[3]= Serial_RxPacket[4]-0x30;task[4] = Serial_RxPacket[5]-0x30;task[5] = Serial_RxPacket[6]-0x30;
	set_wukuaipingtai_weizhi(0x01);
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//********************第二部分，识别物块颜色，按照指定的顺序搬运**************************//
	//复位到识别的位置
	move_all1(-120.0f,1000,200   ,0,2000,200   ,278,13);
	set_zhuashou_kai();
	
	car_move1(0,-1000+90,150,200);
	move_all1(40.0f,1000,200   ,0,2000,200   ,278,13);
	//提前发送命令2
	Serial_GetRxFlag();Serial_SendByte(0x32);
	car_move2(-30,0,100,100);	
	while(1){
		while(1){
			if(Serial_GetRxFlag() == 1){
				break;  //是wait发送的分解
			}
		}
		//0x01代表选择的是转盘上的物块，70.0f是物块的高度，0x00代表不选择颜色
		transfer_position_wukuai(0x01,70.0f,0x00); 
		car_move_distance(direct_x,direct_y,100,80);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}else{
			wait_send_shumeipai(0x32,15000);
		}
	}
	move_all(60.0f,1000,200   ,0,2000,200   ,278,13);
	wait_send_shumeipai(0x33,20000);  //识别圆盘上要抓物块的顺序
	ypcode[0]= Serial_RxPacket[0]-0x30;ypcode[1] = Serial_RxPacket[1]-0x30;ypcode[2] = Serial_RxPacket[2]-0x30;
	//1.圆台拿走第一个物块
	set_wukuaipingtai_weizhi(0x01);
	yuantai_na(ypcode[0]);
	
	//2.圆台拿走第二个物块
	set_wukuaipingtai_weizhi(0x02);
	yuantai_na1(ypcode[1]);   //达到指定的识别的位置
	//判断物块是否静止
	wait_send_shumeipai(0x35,30000);
	while(1){
		if(Serial_RxPacket[0] == 0x39 && Serial_RxPacket[1] == 0x38){
			break;
		}
	}Serial_RxPacket[0] = 0x30;Serial_RxPacket[1] = 0x30;
	yuantai_na2(ypcode[1]);  //一旦识别到物块静止，抓物块到物块平台上
		
	//3.圆台拿走第三个物块
	set_wukuaipingtai_weizhi(0x03);
	yuantai_na1(ypcode[2]);	 //达到指定的识别的位置
	//判断物块是否静止
	wait_send_shumeipai(0x36,30000);
	while(1){
		if(Serial_RxPacket[0] == 0x39 && Serial_RxPacket[1] == 0x38){
			break;
		}
	}Serial_RxPacket[0] = 0x30;Serial_RxPacket[1] = 0x30;	
	yuantai_na2(ypcode[2]);	 //一旦识别到物块静止，抓物块到物块平台上

	TIM_SetCompare1(TIM3,0); //关闭补光灯
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//***************************到下一个区域**************************//	
	//到粗加工区
	PID_move(-30,0,0,400,0,1);//回倒车
	delay_ms1(200);
	car_move1(0,400,150,200);
	
	car_turn_to_stable(90.0f);//在原地旋转并稳定
	
	car_move1(0,-1700,150,250);//快速通过
	
	car_turn_to_stable(-180.0f);//在粗加工区原地旋转并稳定
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//*************************第三部分，色环位置拿并放置**********************//
	while(1){
		wait_send_shumeipai(0x37,10000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x01,0x02);
		car_move_distance(direct_x,direct_y,180,150);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}
		//如果返回了没看到的信息，那就微微调整一下，再重复给树莓派发送
		//反之如果返回了看到的信息（也即第九位并非0x34）,则可以退出该程序
	}
	PID_turn_stable(-180.0f,2000);
	//精细定标
	{
		wait_send_shumeipai(0x38,10000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x02,0x02);	
		if(Serial_RxPacket[8] == 0x34){
			car_move_distance(0,0,70,50);//如果没有看到颜色，那就不移动
		}else{	
			car_move_distance(direct_x,direct_y,130,100);//如果看到颜色，那就按照正常的方式移动
		}
	}//至此第二次定标完成
	
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	//色换放置第一个
	fangzhi(task[0]);	
	set_wukuaipingtai_weizhi(0x02);
	delay_ms1((int)shengjiang_control(33,2000,240));
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	
	//色环放置第二个	
	fangzhi(task[1]);
	set_wukuaipingtai_weizhi(0x03);
	delay_ms1((int)shengjiang_control(33,2000,240));
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,2000,200   ,135,13);
	set_zhuashou_he();
	
	//色环放置第三个	
	fangzhi(task[2]);
	set_wukuaipingtai_weizhi(0x01);
	//色环拿一
	na(task[0]);
	set_wukuaipingtai_weizhi(0x02);
	//色环拿二
	na(task[1]);
	set_wukuaipingtai_weizhi(0x03);
	//色环拿三
	na(task[2]);
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//*************************到下一个区域***************************//	
	PID_move(-30,0,0,300,-180,1);//在粗加工区回退
	PID_turn_stable(-180.0f,2000);
	car_move1(140,140+680,180,180);//从绿色出发
	delay_ms1(100);
	
	car_turn_to_stable(90.0f);//原地旋转并稳定
	
	car_move1(140,140+620,180,150);
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//*************************第四部分，色环位置放置**********************//
	while(1){
		wait_send_shumeipai(0x37,10000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x01,0x02);
		car_move_distance(direct_x,direct_y,180,150);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}
		//如果返回了没看到的信息，那就微微调整一下，再重复给树莓派发送
		//反之如果返回了看到的信息（也即第九位并非0x34）,则可以退出该程序
	}
	PID_turn_stable(90.0f,2000);
	//精细定标
	{
		wait_send_shumeipai(0x38,10000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x02,0x02);	
		if(Serial_RxPacket[8] == 0x34){
			car_move_distance(0,0,70,50);//如果没有看到颜色，那就不移动
		}else{	
			car_move_distance(direct_x,direct_y,130,100);//如果看到颜色，那就按照正常的方式移动
		}
	}//至此第二次定标完成
	
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	//色环放置第一个
	fangzhi(task[0]);
	set_wukuaipingtai_weizhi(0x02);
	delay_ms1((int)shengjiang_control(33,2000,240));
	
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	//色环放置第二个	
	fangzhi(task[1]);
	set_wukuaipingtai_weizhi(0x03);
	delay_ms1((int)shengjiang_control(33,2000,240));
	
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,2000,200   ,135,13);
	set_zhuashou_he();
	//放置第三个	
	fangzhi(task[2]);
	
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//*************************第五部分，回到圆盘**********************//
	PID_move(-30,0,0,300,90,1);//回退
	delay_ms1(100);
	//回到原料
	car_move1(140,220+650,180,180);//从绿色出发
	
	car_turn_to_stable(0.0f);//转回 0° 并稳定
	
	car_move1(140,370+20,180,180);
//*****************************第二轮**************************//
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//********************第二部分，识别物块颜色，按照指定的顺序搬运**************************//
	move_all1(40.0f,1000,200   ,0,2000,200   ,278,13);
	//提前发送0x61
	Serial_GetRxFlag();Serial_SendByte(0x61);
	PID_move(30,0,0,300,0,1);//在粗加工区往前靠
	delay_ms1(100);	
	while(1){
		while(1){
			if(Serial_GetRxFlag() == 1){
				break;  //是wait发送的分解版
			}
		}
		transfer_position_wukuai(0x01,70.0f,0x00);
		car_move_distance(direct_x,direct_y,100,80);//至此第一次定标完成
		if(Serial_RxPacket[8] != 0x34){
			break;
		}else{
			wait_send_shumeipai(0x61,15000);
		}
	}
	move_all(60.0f,1000,200   ,0,2000,200   ,278,13);
	wait_send_shumeipai(0x34,20000);
	ypcode[0]= Serial_RxPacket[0]-0x30;ypcode[1] = Serial_RxPacket[1]-0x30;ypcode[2] = Serial_RxPacket[2]-0x30;
	
	//1.圆台拿走第一个物块
	set_wukuaipingtai_weizhi(0x01);
	yuantai_na(ypcode[0]);
	
	//2.圆台拿走第二个物块
	set_wukuaipingtai_weizhi(0x02);
	yuantai_na1(ypcode[1]);   //达到指定的识别的位置
		//判断物块是否静止
	wait_send_shumeipai(0x62,30000);
	while(1){
		if(Serial_RxPacket[0] == 0x39 && Serial_RxPacket[1] == 0x38){
			break;
		}
	}Serial_RxPacket[0] = 0x30;Serial_RxPacket[1] = 0x30;
	yuantai_na2(ypcode[1]);  //一旦识别到物块静止，抓物块到物块平台上
		
	//3.圆台拿走第三个物块
	set_wukuaipingtai_weizhi(0x03);
	yuantai_na1(ypcode[2]);	 //达到指定的识别的位置
	wait_send_shumeipai(0x63,30000);
	while(1){
		if(Serial_RxPacket[0] == 0x39 && Serial_RxPacket[1] == 0x38){
			break;
		}
	}Serial_RxPacket[0] = 0x30;Serial_RxPacket[1] = 0x30;	
	yuantai_na2(ypcode[2]);	 //一旦识别到物块静止，抓物块到物块平台上

	TIM_SetCompare1(TIM3,0); //关闭补光灯
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//***************************到下一个区域**************************//	
	//到粗加工区
	PID_move(-30,0,0,400,0,1);//回倒车
	delay_ms1(200);
	car_move1(0,400,150,200);
	
	car_turn_to_stable(90.0f);//在原地旋转并稳定
	
	car_move1(0,-1700,150,250);//快速通过
	
	car_turn_to_stable(-180.0f);//在粗加工区原地旋转并稳定
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//*************************第三部分，色环位置拿并放置**********************//
	while(1){
		wait_send_shumeipai(0x37,20000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x01,0x02);
		car_move_distance(direct_x,direct_y,180,150);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}
		//如果返回了没看到的信息，那就微微调整一下，再重复给树莓派发送
		//反之如果返回了看到的信息（也即第九位并非0x34）,则可以退出该程序
	}
	PID_turn_stable(-180.0f,2000);
	//精细定标
	{
		wait_send_shumeipai(0x38,20000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x02,0x02);	
		if(Serial_RxPacket[8] == 0x34){
			car_move_distance(0,0,70,50);//如果没有看到颜色，那就不移动
		}else{	
			car_move_distance(direct_x,direct_y,130,100);//如果看到颜色，那就按照正常的方式移动
		}
	}//至此第二次定标完成
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	//色环放置第一个
	fangzhi(task[3]);
	set_wukuaipingtai_weizhi(0x02);
	delay_ms1((int)shengjiang_control(33,2000,240));
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	
	//色环放置第二个	
	fangzhi(task[4]);
	set_wukuaipingtai_weizhi(0x03);
	delay_ms1((int)shengjiang_control(33,2000,240));
	move_all(-48.0f,1000,200   ,33,2000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	
	//色环放置第二个	
	fangzhi(task[5]);
	set_wukuaipingtai_weizhi(0x01);
	//拿一
	na(task[3]);
	set_wukuaipingtai_weizhi(0x02);
	//拿二
	na(task[4]);
	set_wukuaipingtai_weizhi(0x03);
	//拿三
	na(task[5]);
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//*************************到下一个区域***************************//	
	PID_move(-30,0,0,300,-180,1);//在粗加工区回退
	PID_turn_stable(-180.0f,2000);
	car_move1(140,140+680,180,180);//从绿色出发
	delay_ms1(100);
	
	car_turn_to_stable(90.0f);//原地旋转并稳定
	
	car_move1(140,140+620,180,150);
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//********************************码垛****************************
	while(1){
		wait_send_shumeipai(0x39,10000);	
		transfer_position_wukuai(0x02,70.0f,0x02);
		car_move_distance(direct_x,direct_y,180,150);//至此第一次定标完成
		delay_ms1(200);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}
	}
	{
		wait_send_shumeipai(0x39,10000);		
		transfer_position_wukuai(0x02,70.0f,0x02);
		if(Serial_RxPacket[8] == 0x34){
			car_move_distance(0,0,70,50);
		}else{	
			car_move_distance(direct_x,direct_y,180,150);//至此第二次定标完成
		}
	}
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	//放置第一个
	fangzhi_maduo(task[3]);
	set_wukuaipingtai_weizhi(0x02);
	delay_ms1((int)shengjiang_control(33,2000,240));
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	
	//放置第二个	
	fangzhi_maduo(task[4]);
	set_wukuaipingtai_weizhi(0x03);
	delay_ms1((int)shengjiang_control(33,2000,240));
	move_all(-48.0f,1000,200   ,33,2000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	
	//放置第三个	
	fangzhi_maduo(task[5]);
	set_wukuaipingtai_weizhi(0x01);
	//回到启停区
	PID_move(-30,0,0,300,90,1);//回退
	car_move1(140,220+650,180,180);//从绿色出发
	move_all1(0.0f,1000,200   ,0,1000,200   ,64.8,13);
	
	car_turn_to_stable(0.0f);//转回 0° 并稳定
	
	car_move1(140,2000-150,180,180);
	car_move2(-130,0,150,150);//在粗加工区往前靠
	
	jixie_reset();
#endif
}

void fangzhi_green(void)
{
	move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
	move_all(-20.0f,1000,200   ,25,1000,200   ,135,13);
	move_all(-20.0f,1000,200   ,25,1000,200   ,278,13);
	move_all(-50.0f,1000,200   ,132,1000,200   ,278,13);
	set_zhuashou_kai();
}

void fangzhi_red(void)
{
	move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
	move_all(3.0f+1.0f,1000,200   ,25,1000,200   ,236.0f,13);
	move_all(3.0f+1.0f,1000,200   ,132,1000,200   ,236.0f,13);
	
	set_zhuashou_kai();
}
void fangzhi_blue(void)
{
	move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
	move_all(3.0f+5.8f,1000,200   ,25,1000,200   ,317.0f+1.4f,13);
	move_all(3.0f+5.8f,1000,200   ,132,1000,200   ,317.0f+1.4f,13);
	set_zhuashou_kai();
}

void fangzhi(uint8_t task)
{
	if(task == 0x01){
		fangzhi_red();
	}else if(task == 0x02){
		fangzhi_green();
	}else if(task == 0x03){
		fangzhi_blue();
	}
}

void fangzhi_maduo(uint8_t task)
{
	if(task == 0x01){
		move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
		move_all(3.0f,1000,200   ,25,1000,200   ,236,13);
		move_all(3.0f,1000,200   ,130-70,1000,200   ,236,13);
		set_zhuashou_kai();
	}else if(task == 0x02){
		move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
		move_all(-48.0f,1000,200   ,25,1000,200   ,180,13);
		move_all(-120.0f,1000,200   ,25,1000,200   ,278,13);
		move_all(-50.0f,1000,200   ,130-70,1000,200   ,278,13);
		set_zhuashou_kai();
	}else if(task == 0x03){
		move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
		move_all(-48.0f,1000,200   ,25,1000,200   ,180,13);
		move_all(-120.0f,1000,200   ,25,1000,200   ,317,13);
		move_all(3.0f,1000,200   ,130-70,1000,200   ,317,13);
		set_zhuashou_kai();
	}
}

void na_green(void)
{	
	move_all(-10.0f,1000,200   ,130,1000,200   ,278,13);
	move_all(-50.0f,1000,200   ,130,1000,200   ,278,13);
	set_zhuashou_he();
	move_all(-50.0f,1000,200   ,50,1000,220   ,278,13);
	move_all(-48.0f,1000,200   ,25,1000,220   ,135,13);
	move_all(-48.0f,1000,200   ,33,1000,220   ,135,13);
	set_zhuashou_kai();
}

void na_blue(void)
{
	move_all(3.0f,1000,200   ,110,1000,200   ,317,13);
	move_all(3.0f,1000,200   ,130,1000,200   ,317,13);
	set_zhuashou_he();
	move_all(8.0f,1000,200   ,25,1000,220   ,317,13);
	move_all(-48.0f,1000,200   ,25,1000,220   ,135,13);
	move_all(-48.0f,1000,200   ,33,1000,220   ,135,13);
	set_zhuashou_kai();
}

void na_red(void)
{
	move_all(3.0f,1000,200   ,130,1000,200   ,236,13);
	set_zhuashou_he();
	move_all(-50.0f,1000,200   ,50,1000,220   ,236,13);
	move_all(-48.0f,1000,200   ,25,1000,220   ,135,13);
	move_all(-48.0f,1000,200   ,33,1000,220   ,135,13);
	set_zhuashou_kai();
}

void na(uint8_t task)
{
	if(task == 0x01){
		na_red();
	}else if(task == 0x02){
		na_green();
	}else if(task == 0x03){
		na_blue();
	}
}

void yuantai_na(uint8_t task)
{
	if(task == 0x01){
		set_zhuashou_kai();  
		move_all(-100.0f,1000,220   ,50,2000,220   ,278,13);
		set_zhuashou_he();  
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();	 
	}else if(task == 0x02){
		set_zhuashou_kai();	
		move_all(63.0f,1000,200   ,48,2000,220   ,295,13);
		set_zhuashou_he();   
		move_all(20.0f,1000,200   ,25,2000,220   ,295,13);//**********多一个回收的动作
		move_all(-20.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();  
	}else if(task == 0x03){
		set_zhuashou_kai();  
		move_all(63.0f,1000,200   ,48,2000,220   ,261,13);
		set_zhuashou_he();  
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();  
	}
}

void yuantai_na1(uint8_t task)
{
	if(task == 0x01){
		set_zhuashou_kai();
		move_all(40.0f,1000,200   ,0,2000,200   ,278,13);
	}else if(task == 0x02){
		set_zhuashou_kai();	
		move_all(63.0f,1000,200   ,18,2000,220   ,295,13);
	}else if(task == 0x03){
		set_zhuashou_kai();  
		move_all(63.0f,1000,200   ,18,2000,220   ,261,13);
	}
}

void yuantai_na2(uint8_t task)
{
	if(task == 0x01){
		move_all(-100.0f,1000,220   ,50,2000,220   ,278,13);
		set_zhuashou_he();   
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();	 
	}else if(task == 0x02){
		move_all(63.0f,1000,200   ,48,2000,220   ,295,13);
		set_zhuashou_he();   
		move_all(20.0f,1000,200   ,25,2000,220   ,295,13);//**********多一个回收的动作
		move_all(-20.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();  
	}else if(task == 0x03){
		move_all(63.0f,1000,200   ,48,2000,220   ,261,13);
		set_zhuashou_he();  
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();
	}
}


//if task_choose = 0x01,则为检测转盘处的物块圆心
//if task_choose = 0x02,则为检测码垛处的物块圆心
//distance 为物块的高度
//color 为选择定标到哪一个物块的位置 为0表示任何位置都可以
void transfer_position_wukuai(uint8_t task_choose,float distance,uint8_t color)
{
	x_origin = (Serial_RxPacket[0]-0x30)*1000+(Serial_RxPacket[1]-0x30)*100+(Serial_RxPacket[2]-0x30)*10+(Serial_RxPacket[3]-0x30);
	y_origin = (Serial_RxPacket[4]-0x30)*1000+(Serial_RxPacket[5]-0x30)*100+(Serial_RxPacket[6]-0x30)*10+(Serial_RxPacket[7]-0x30);
	float bilichi = 0;
	if(task_choose == 0x01){
		bilichi = -0.0035f*(distance+80.0f) + 1.1967f;
		x_origin = x_origin - 322;
		y_origin = y_origin - 165;
	}else if(task_choose == 0x02){
		bilichi = -0.0033f*(distance) + 1.1655f;
		x_origin = x_origin - 314;
		y_origin = y_origin - 238;
	}
	direct_x = x_origin * bilichi;   
	direct_y = y_origin * bilichi;
	//下面的代码仅仅和颜色有关
	if(color == 2){
		if(Serial_RxPacket[8] == 0x32){
		}else if(Serial_RxPacket[8] == 0x31){
				direct_x = direct_x + distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x33){
				direct_x = direct_x - distance_wukuai;
		}
	}else if(color == 1){
		if(Serial_RxPacket[8] == 0x31){
		}else if(Serial_RxPacket[8] == 0x32){
			direct_x = direct_x - distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x33){
				direct_x = direct_x - 2*distance_wukuai;
		}
	}else if(color == 3){
		if(Serial_RxPacket[8] == 0x33){
		}else if(Serial_RxPacket[8] == 0x32){
			direct_x = direct_x + distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x31){
				direct_x = direct_x + 2*distance_wukuai;
		}
	}
	if(Serial_RxPacket[8] == 0x34){
		direct_y = 5;direct_x = -5;
	}
}


//与任务码相关联，定位色环位置
//fenbian_choose
	//fenbian_choose == 0x01 则为小分辨率
	//fenbian_choose == 0x02 则为大分辨率
//task_num 
	//选择移动到对应颜色的位置 为零表示与颜色无关
void transfer_position_sehuan(uint8_t fenbian_choose,uint8_t task_num)
{   
	x_origin = (Serial_RxPacket[0]-0x30)*1000+(Serial_RxPacket[1]-0x30)*100+(Serial_RxPacket[2]-0x30)*10+(Serial_RxPacket[3]-0x30);
	y_origin = (Serial_RxPacket[4]-0x30)*1000+(Serial_RxPacket[5]-0x30)*100+(Serial_RxPacket[6]-0x30)*10+(Serial_RxPacket[7]-0x30);
	float x_center,y_center;//x加物块右移//y+物块下移
	if(fenbian_choose == 0x01){
		x_center =316;y_center =238;
		x_origin = x_origin - x_center;
		y_origin = y_origin - y_center;
		
		direct_x = x_origin * 1.1805555;    //实际尺寸系数
		direct_y = y_origin * 1.1805555;
	}else if(fenbian_choose == 0x02){
		if(Serial_RxPacket[8] == 0x31){
			x_center = x_center_red;y_center = y_center_red;
		}else if(Serial_RxPacket[8] == 0x32){
			x_center = x_center_green;y_center = y_center_green;
		}else if(Serial_RxPacket[8] == 0x33){
			x_center = x_center_blue;y_center = y_center_blue;
		}
		x_origin = x_origin - x_center;
		y_origin = y_origin - y_center;
	
		direct_x = x_origin * 0.29239766;    //实际尺寸系数
		direct_y = y_origin * 0.29239766;
	}
	
	if(task_num == 2){
		if(Serial_RxPacket[8] == 0x32){
		}else if(Serial_RxPacket[8] == 0x31){
				direct_x = direct_x + distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x33){
				direct_x = direct_x - distance_wukuai;
		}
	}else if(task_num == 1){
		if(Serial_RxPacket[8] == 0x31){
		}else if(Serial_RxPacket[8] == 0x32){
			direct_x = direct_x - distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x33){
				direct_x = direct_x - 2*distance_wukuai;
		}
	}else if(task_num == 3){
		if(Serial_RxPacket[8] == 0x33){
		}else if(Serial_RxPacket[8] == 0x32){
			direct_x = direct_x + distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x31){
				direct_x = direct_x + 2*distance_wukuai;
		}
	}
	if(Serial_RxPacket[8] == 0x34){
				direct_y = 3;direct_x = -3;
	}
}

void jixie_reset(void)
{	
	while(swith_out() == 1){		
		LED_Toggle2();
		delay_ms1(50);
	}
	
	set_zhuashou_kai();
	move_all(0.0f,1000,200   ,0,1000,200   ,68.4,13);
	set_wukuaipingtai_Angle(20.0f,300);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	TIM_Cmd(TIM5, ENABLE); 
	while(1){}
}

/*
	先发送命令，等待一段时间如果还没有返回
	则继续发送，直到收到信息，立刻退出
*/
void wait_send_shumeipai(uint8_t message,uint16_t time)
{	
	Serial_GetRxFlag();//清空标志位
	Serial_SendByte(message);
	while(1){
		if(Serial_GetRxFlag() == 1){
			break;
		}
	}
}

//定时显示OLED和LCD显示屏
void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{   
		imu_scan(fAcc, fGyro, fAngle);
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		OLED_ShowSignedNum(1,1,fAngle[2],3);
		OLED_ShowChar(1,5,'.');
		OLED_ShowNum(1,6,ABS((fAngle[2]-(int)fAngle[2])*1000),3);
		OLED_ShowSignedNum(2,1,angle,3);
		OLED_ShowChar(2,5,'.');
		OLED_ShowNum(2,6,ABS((angle-(int)angle)*1000),3);
		
	}
}

void clear_ckp(void)
{
	sprintf(tjcstr, "t0.txt=\"  \"");
			HMISends(tjcstr);
	        HMISendb(0xff);//在显示屏上面显示任务
}
