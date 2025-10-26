/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <string.h>   // strlen, strncmp
#include <stdlib.h>   // atoi
#include <ctype.h>    // toupper
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../PeripheralDriver/Inc/oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ---- ICM-20948 (Bank 0) ---- */
#define ICM_REG_WHO_AM_I       0x00  /* expect 0xEA */
#define ICM_REG_PWR_MGMT_1     0x06
#define ICM_REG_PWR_MGMT_2     0x07
#define ICM_REG_GYRO_XOUT_H    0x33  /* XH,XL,YH,YL,ZH,ZL (6 bytes) */
#define ICM_REG_BANK_SEL       0x7F  /* write (bank << 4) */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHEEL_DIAMETER_CM 6.50f
#define ENCODER_CPR 11.0f
#define GEAR_RATIO 30.0f
#define QUAD_MULT 4.0f
#define WHEEL_CIRC_CM (3.1415926f * WHEEL_DIAMETER_CM)// = 20.42 cm
#define COUNTS_PER_REV 1560.0f  // you can adjust
#define CM_PER_COUNT (WHEEL_CIRC_CM / COUNTS_PER_REV)

// TIM1/TIM4 PWM period from your MX init = 7199
#define PWM_MAX             7199.0f
#define PWM_MIN             0.0f
#define pwm_slow            2000
#define pwm_fast            4750

// RPS values from encoders
volatile float rpsA = 0.0f;
volatile float rpsD = 0.0f;

//Running totals (signed). Visible across tasks if you want to display elsewhere.
volatile int32_t total_counts_A = 0;
volatile int32_t total_counts_D = 0;
// Distance tracking
volatile float   distance_cm_A  = 0.0f;
volatile float   distance_cm_D  = 0.0f;

// Target distance and motion flag
volatile int targetdistance_cm = 150; // Set via UART or test variable
volatile int motionActive = 0;   // 1 = robot moving, 0 = idle
volatile int pwmA_val;
volatile int pwmD_val;
uint8_t  dir  = 0;

#define MC_PERIOD_MS   50       // control period
#define KP_DIFF        (25.0f * 60.0f)    // proportional [PWM per RPS]
#define KI_DIFF        (0.6f * 60.0f)     // integral [PWM per RPS per second]
#define KD_DIFF        (0.00f * 60.0f)   // derivative [PWM per RPS * second] (start at 0)

#define I_MAX          4000.0f  // integral clamp (PWM units)
#define PWM_MIN_CLAMP  2000
#define PWM_MAX_CLAMP  7199     // clamp to ARR (0..7199)

// wheel bias offsets (use to cancel static asymmetry; + makes that wheel faster)
static int biasA = 0;   // e.g., +150 if A is habitually slower
static int biasD = 0;
/* === Gyro state === */
volatile uint8_t  ICM_addr        = 0;    // 0x68 or 0x69 (7-bit)
static   uint8_t  ICM_whoami      = 0;    // expect 0xEA
static   float    _gyro_bias_lsb  = 0.0f; // bias in raw LSB
volatile float    yaw_rate_dps    = 0.0f; // filtered Z rate (°/s)
volatile float    yaw_angle_deg   = 0.0f; // integrated yaw (°)
volatile uint8_t  gyro_ready      = 0;    // set after bias calibration
static   uint32_t _gyro_last_ms   = 0;

static const float GYRO_SENS_LSB_PER_DPS = 131.0f;  // FS=±250/500/1000/2000dps -> adjust if needed
static const float GYRO_LP_ALPHA         = 0.67f;   // IIR LPF coefficient

/* ================= Steer control config ================= */
#define STEER_US_LEFT      900     // +36°
#define STEER_US_CENTER    1350    //  0°
#define STEER_US_RIGHT     2400    // -36°
#define STEER_MAX_DEG      36.0f
#define STEER_DB_DEG       2.0f    // stop tolerance
#define STEER_KP           0.65f   // proportional (deg -> deg cmd)
#define STEER_KD           0.08f   // derivative (dps -> deg cmd) ~damping
#define STEER_LOOP_HZ      50      // 50 Hz loop
#define STEER_CMD_TIMEOUT  8000U   // safety timeout per command (ms)

#define YAW_COARSE_THRESH_DEG  5.0f   // > this → bang-bang
#define YAW_FINE_DB_DEG         0.8f   // final tolerance to declare "done"
#define YAW_STOP_DB_DEG         1.0f   // leave as your STEER_DB_DEG or a bit larger

#define TURN_PWM_MAX           5000    // your existing run level
#define TURN_PWM_MIN           4250    // minimum that still spins both wheels
#define MICRO_NUDGE_PWM        4500    // quick corrective pulse
#define MICRO_NUDGE_MS         20      // 15–30ms works well

#define FINE_KP_STEER          0.85f   // deg → wheel deg
#define FINE_KD_RATE           0.10f   // dps → wheel deg


/* Public gyro (already in your file) */
extern volatile float yaw_angle_deg;  // from gyro_task
extern volatile float yaw_rate_dps;   // from gyro_task
extern volatile uint8_t gyro_ready;

/* Forward decl (servo/steer helpers & task) */
static inline uint16_t steer_deg_to_pulse(float wheel_deg);
static inline float    angle_wrap_180(float a);
static inline float    smallest_err_deg(float target, float current);
static void            steer_write_us(uint16_t usec);
static void            steer_center(void);
void                   ServoAlignTask(void *argument);

/* Simple command mailbox */
typedef struct {
  volatile uint8_t  pending;        // 1 when a new command is waiting
  volatile float    target_heading; // absolute heading (deg, -180..+180)
  volatile uint8_t  zero_yaw_after; // 1: reset yaw when done
  volatile uint8_t  busy;           // task is executing a command
  volatile uint8_t  success;        // last result

  // NEW (bang-bang turn)
  volatile uint8_t  bangbang;      // 1 = hard lock servo, no PID
  volatile uint16_t drive_pwm;     // PWM to use while driving forward
  // NEW: reverse drive for bang-bang turns
  volatile uint8_t  reverse_drive;   // 0 = forward (default), 1 = reverse
} steer_cmd_t;


/* --- Command Cooldown --- */
#define CMD_COOLDOWN_MS   750u   // tweak 150–500 ms as you like
static volatile uint32_t g_cmd_cooldown_until_ms = 0;

static inline void StartCooldown(uint32_t ms)
{
  g_cmd_cooldown_until_ms = HAL_GetTick() + ms;
}

static steer_cmd_t g_steer_cmd = {0};

/* UART line buffer (one command) */
static char uart3_line[32];
static volatile uint8_t uart3_idx = 0;

/* === Command FIFO (queue) ============================================= */
#define CMDQ_CAP 64
typedef struct { char s[32]; } cmd_item_t;
static volatile uint16_t cmdq_head = 0, cmdq_tail = 0;
static cmd_item_t cmdq[CMDQ_CAP];

static inline int cmdq_empty(void) { return cmdq_head == cmdq_tail; }
static int cmdq_push(const char *cmd)
{
  uint16_t next = (cmdq_head + 1) % CMDQ_CAP;
  if (next == cmdq_tail) return -1; // full
  strncpy((char*)cmdq[cmdq_head].s, cmd, sizeof(cmdq[0].s)-1);
  cmdq[cmdq_head].s[sizeof(cmdq[0].s)-1] = '\0';
  cmdq_head = next;
  return 0;
}
static int cmdq_pop(char *out, int maxlen)
{
  if (cmdq_empty()) return -1;
  strncpy(out, (const char*)cmdq[cmdq_tail].s, maxlen-1);
  out[maxlen-1] = '\0';
  cmdq_tail = (cmdq_tail + 1) % CMDQ_CAP;
  return 0;
}

/* single-byte RX buffer */
static uint8_t uart3_rx_ch;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ShowTask */
osThreadId_t ShowTaskHandle;
const osThreadAttr_t ShowTask_attributes = {
  .name = "ShowTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DistanceTask */
osThreadId_t DistanceTaskHandle;
const osThreadAttr_t DistanceTask_attributes = {
  .name = "DistanceTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ServoMotorTask */
osThreadId_t ServoMotorTaskHandle;
const osThreadAttr_t ServoMotorTask_attributes = {
  .name = "ServoMotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRTask */
osThreadId_t IRTaskHandle;
const osThreadAttr_t IRTask_attributes = {
  .name = "IRTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UltrasonicTask */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
uint8_t aRxBuffer[20];
volatile uint16_t g_ir_sample = 0;   // latest ADC sample (0..4095)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void motor(void *argument);
void encoder(void *argument);
void distance(void *argument);
void imu(void *argument);
void servomotor(void *argument);
void ir(void *argument);
void ultrasonic(void *argument);

/* USER CODE BEGIN PFP */
/* ICM helpers */
static HAL_StatusTypeDef icm_write(uint8_t addr7, uint8_t reg, uint8_t val);
static HAL_StatusTypeDef icm_read (uint8_t addr7, uint8_t reg, uint8_t *val);
static HAL_StatusTypeDef icm_select_bank(uint8_t addr7, uint8_t bank);
static int               icm_probe(void);
static void              icm_wake_enable_gyro(void);
static HAL_StatusTypeDef icm_read_gyro_xyz(int16_t *gx, int16_t *gy, int16_t *gz);

/* Helper to drive a dual-input H-bridge using two PWM channels */
static inline void Motor_Set(TIM_HandleTypeDef *htim,
                             uint32_t ch_fwd, uint32_t ch_rev,
                             uint16_t duty, uint8_t dir)
{
  /* dir: 0 = CW, 1 = CCW */
  if (dir == 0) {
    __HAL_TIM_SET_COMPARE(htim, ch_fwd, duty);
    __HAL_TIM_SET_COMPARE(htim, ch_rev, 0);
  } else {
    __HAL_TIM_SET_COMPARE(htim, ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(htim, ch_rev, duty);
  }
}
/* Request an absolute heading turn. Always resets yaw when done. */
static inline int Servo_RequestTurnTo(float target_heading_deg)
{
  if (g_steer_cmd.busy) return 1; // still handling previous command
  g_steer_cmd.target_heading = angle_wrap_180(target_heading_deg);
  g_steer_cmd.zero_yaw_after = 1;     // <-- always reset
  g_steer_cmd.pending        = 1;
  return 0;
}

static inline int Servo_RequestBangBangTurn(float delta_deg, uint16_t pwm)
{
  if (g_steer_cmd.busy) return 1;

  float tgt = angle_wrap_180(yaw_angle_deg + delta_deg);
  g_steer_cmd.target_heading = tgt;
  g_steer_cmd.zero_yaw_after = 1;       // re-zero yaw after success (convenient chaining)
  g_steer_cmd.drive_pwm      = pwm ? pwm : 3500;   // default if 0
  g_steer_cmd.bangbang       = 1;
  g_steer_cmd.pending        = 1;
  return 0;
}

static inline int Servo_RequestBangBangTurnRev(float delta_deg, uint16_t pwm)
{
  if (g_steer_cmd.busy) return 1;

  float tgt = angle_wrap_180(yaw_angle_deg + delta_deg);
  g_steer_cmd.target_heading = tgt;
  g_steer_cmd.zero_yaw_after = 1;
  g_steer_cmd.drive_pwm      = pwm ? pwm : 3500;
  g_steer_cmd.bangbang       = 1;
  g_steer_cmd.reverse_drive  = 1;      // <<< reverse!
  g_steer_cmd.pending        = 1;
  return 0;
}


/* Convenience: relative delta (deg). +left / -right. Always resets yaw. */
static inline int Servo_RequestTurnBy(float delta_deg)
{
  float tgt = angle_wrap_180(yaw_angle_deg + delta_deg);
  return Servo_RequestTurnTo(tgt);
}

/* === Motion helpers ========================================================= */
#define DIR_FWD   0
#define DIR_BACK  1

static inline void AllStop(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

#define E_BRAKE_PWM   4500   // tweak 4000–5500
#define E_BRAKE_MS    80     // tweak 60–120 ms

static inline void ActiveBrakeStopForward(void)
{
  // During FL/FR we were driving "forward" as:
  //   Motor A (TIM4) dir=1, Motor D (TIM1) dir=0.
  // To brake, apply the reverse briefly:
  Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, E_BRAKE_PWM, 0); // A reverse of forward
  Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, E_BRAKE_PWM, 1); // D reverse of forward
  osDelay(E_BRAKE_MS);
  AllStop();
}

static inline void ActiveBrakeStopBackward(void)
{
  // During BL/BR we were driving "backward" as:
  //   Motor A dir=0, Motor D dir=1 (the reverse of your forward wiring)
  // To brake, apply the forward briefly:
  Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, E_BRAKE_PWM, 1); // A forward of backward
  Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, E_BRAKE_PWM, 0); // D forward of backward
  osDelay(E_BRAKE_MS);
  AllStop();
}

static inline void DriveForwardPWM(int pwmA, int pwmD)
{
  // A forward: TIM4 CH4 high
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwmA);
  // D forward: TIM1 CH3 high
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwmD);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

static inline void DriveBackwardPWM(int pwmA, int pwmD)
{
  // A backward: TIM4 CH3 high
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwmA);
  // D backward: TIM1 CH4 high
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwmD);
}

static inline void ResetDistanceCounts(void)
{
  taskENTER_CRITICAL();
  total_counts_A = 0;
  total_counts_D = 0;
  distance_cm_A  = 0.0f;
  distance_cm_D  = 0.0f;
  taskEXIT_CRITICAL();
}

static inline void StartMoveCM(int dist_cm, uint8_t dir_cmd)
{
  if (dist_cm < 0) dist_cm = -dist_cm;
  ResetDistanceCounts();          // relative move
  targetdistance_cm = dist_cm - 5; //Change to fine tune distance
  dir               = (dir_cmd == DIR_BACK) ? DIR_BACK : DIR_FWD;
  motionActive      = 1;
}

static inline uint16_t scale_pwm_from_err(float abs_err)
{
  if (abs_err >= YAW_COARSE_THRESH_DEG) return TURN_PWM_MAX;
  // linear map abs_err ∈ [0, YAW_COARSE_THRESH] → [TURN_PWM_MIN, TURN_PWM_MAX]
  float t = abs_err / YAW_COARSE_THRESH_DEG;
  float pwm = TURN_PWM_MIN + t * (TURN_PWM_MAX - TURN_PWM_MIN);
  if (pwm < TURN_PWM_MIN) pwm = TURN_PWM_MIN;
  if (pwm > TURN_PWM_MAX) pwm = TURN_PWM_MAX;
  return (uint16_t)pwm;
}

static inline void turn(int need_left, uint16_t pwm, uint8_t rev_drive)
{
  if (!rev_drive) {
    // Forward mapping (your original, proven)
    if (need_left) {
      // A backward, D forward
      Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_slow, 1);
      Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_fast, 0);
    } else {
      // A forward, D backward
      Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_fast, 1);
      Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_slow, 0);
    }
  } else {
    // Reverse mapping (your fixed version)
    if (need_left) {
      // A forward, D backward
      Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_slow, 0);
      Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_fast, 1);
    } else {
      // A backward, D forward
      Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_fast, 0);
      Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_slow, 1);
    }
  }
}
static inline void spin_in_place(int need_left, uint16_t pwm, uint8_t rev_drive)
{
  if (!rev_drive) {
    // Forward mapping (your original, proven)
    if (need_left) {
      // A backward, D forward
      Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm, 0);
      Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm, 0);
    } else {
      // A forward, D backward
      Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm, 1);
      Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm, 1);
    }
  } else {
    // Reverse mapping (your fixed version)
    if (need_left) {
      // A forward, D backward
      Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm, 1);
      Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm, 1);
    } else {
      // A backward, D forward
      Motor_Set(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm, 0);
      Motor_Set(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm, 0);
    }
  }
}

/* Calibrated center IR: raw ADC count -> distance in mm */
int32_t IrConvert(int32_t nc)
{
    if (nc < 1)     nc = 1;      // avoid divide by zero / powf domain errors
    if (nc > 4095)  nc = 4095;   // ADC max

    float x = (float)nc;

    // New fitted power-law model: y = A * x^B
    const float A = 2328857.1639444f;
    const float B = -1.5603067800772f;

    // distance in cm
    float d_cm = (A * powf(x, B));

    if (d_cm < 0.0f) d_cm = 0.0f;   // no negative distance

    return (int32_t)(d_cm);  // round to nearest cm
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  // NEW:
  HAL_UART_Receive_IT(&huart3, &uart3_rx_ch, 1);

  /* ===== Start PWM/Encoder here too (idempotent) ===== */
  // PWM for both motors
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_MOE_ENABLE(&htim1);

  // Encoders
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ShowTask */
  //ShowTaskHandle = osThreadNew(show, NULL, &ShowTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motor, NULL, &MotorTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder, NULL, &EncoderTask_attributes);

  /* creation of DistanceTask */
  DistanceTaskHandle = osThreadNew(distance, NULL, &DistanceTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(imu, NULL, &IMUTask_attributes);

  /* creation of ServoMotorTask */
  ServoMotorTaskHandle = osThreadNew(servomotor, NULL, &ServoMotorTask_attributes);

  /* creation of IRTask */
  //IRTaskHandle = osThreadNew(ir, NULL, &IRTask_attributes);

  /* creation of UltrasonicTask */
  //UltrasonicTaskHandle = osThreadNew(ultrasonic, NULL, &UltrasonicTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 7199;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 71;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 19999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RST_Pin OLED_SDA_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_PB_Pin IMU_INT_Pin */
  GPIO_InitStruct.Pin = USER_PB_Pin|IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
/* ==================== ICM-20948 low-level helpers ==================== */
static HAL_StatusTypeDef icm_write(uint8_t addr7, uint8_t reg, uint8_t val)
{
  extern I2C_HandleTypeDef hi2c2;  // already defined in this file
  return HAL_I2C_Mem_Write(&hi2c2, (addr7 << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static HAL_StatusTypeDef icm_read(uint8_t addr7, uint8_t reg, uint8_t *val)
{
  extern I2C_HandleTypeDef hi2c2;
  return HAL_I2C_Mem_Read(&hi2c2, (addr7 << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, val, 1, 10);
}

static HAL_StatusTypeDef icm_select_bank(uint8_t addr7, uint8_t bank) // 0..3
{
  uint8_t b = (uint8_t)(bank << 4);
  return icm_write(addr7, ICM_REG_BANK_SEL, b);
}

/* Try 0x68 then 0x69; confirm WHO_AM_I == 0xEA */
static int icm_probe(void)
{
  const uint8_t addrs[2] = { 0x68, 0x69 };
  for (int i = 0; i < 2; ++i) {
    uint8_t who = 0;
    icm_select_bank(addrs[i], 0);
    if (icm_read(addrs[i], ICM_REG_WHO_AM_I, &who) == HAL_OK && who == 0xEA) {
      ICM_addr   = addrs[i];
      ICM_whoami = who;
      return 1;
    }
  }
  return 0;
}

/* Clear sleep/select clock; enable gyro+accel */
static void icm_wake_enable_gyro(void)
{
  icm_select_bank(ICM_addr, 0);
  /* PWR_MGMT_1: 0x01 -> auto clock, sleep=0 */
  icm_write(ICM_addr, ICM_REG_PWR_MGMT_1, 0x01);
  HAL_Delay(10);
  /* PWR_MGMT_2: 0x00 -> enable all accel+gyro axes */
  icm_write(ICM_addr, ICM_REG_PWR_MGMT_2, 0x00);
  HAL_Delay(10);
}

/* Burst read 6 bytes of gyro data */
static HAL_StatusTypeDef icm_read_gyro_xyz(int16_t *gx, int16_t *gy, int16_t *gz)
{
  extern I2C_HandleTypeDef hi2c2;
  uint8_t buf[6];

  if (icm_select_bank(ICM_addr, 0) != HAL_OK) return HAL_ERROR;

  if (HAL_I2C_Mem_Read(&hi2c2, (ICM_addr << 1), ICM_REG_GYRO_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, buf, 6, 100) != HAL_OK) {
    return HAL_ERROR;
  }

  *gx = (int16_t)((buf[0] << 8) | buf[1]);
  *gy = (int16_t)((buf[2] << 8) | buf[3]);
  *gz = (int16_t)((buf[4] << 8) | buf[5]);
  return HAL_OK;
}


/* Map wheel angle (deg, +36..-36) to microseconds (900..2100) */
static inline uint16_t steer_deg_to_pulse(float wheel_deg)
{
  if (wheel_deg >  STEER_MAX_DEG) wheel_deg =  STEER_MAX_DEG;
  if (wheel_deg < -STEER_MAX_DEG) wheel_deg = -STEER_MAX_DEG;
  /* 1500 - 16.6667*deg  (since +36° => 900us, -36° => 2100us) */
  float usec = 1500.0f - (wheel_deg * (600.0f/36.0f));
  if (usec < STEER_US_LEFT)  usec = STEER_US_LEFT;
  if (usec > STEER_US_RIGHT) usec = STEER_US_RIGHT;
  return (uint16_t)(usec + 0.5f);
}

/* Wrap any angle to [-180, 180) */
static inline float angle_wrap_180(float a)
{
  while (a >= 180.0f) a -= 360.0f;
  while (a <  -180.0f) a += 360.0f;
  return a;
}

/* Smallest signed error from current->target in degrees */
static inline float smallest_err_deg(float target, float current)
{
  return angle_wrap_180(target - current);
}

/* Raw servo write on TIM12 CH2 (expects 20ms period already set) */
static void steer_write_us(uint16_t usec)
{
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, usec);
}

/* Convenience: center wheels */
static void steer_center(void)
{
  steer_write_us(STEER_US_CENTER);
}

static void uart3_send(const char *s)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)s, (uint16_t)strlen(s), 0xFFFF);
}

static int clampi(int v, int lo, int hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    char ch = (char)uart3_rx_ch;

    // Optional: local echo while typing
    // HAL_UART_Transmit(&huart3, (uint8_t*)&uart3_rx_ch, 1, 0xFFFF);

    if (ch == '\n' || ch == '\r') {
      if (uart3_idx >= sizeof(uart3_line)) uart3_idx = sizeof(uart3_line) - 1;
      uart3_line[uart3_idx] = '\0';

      /* --- enqueue one uppercased command line --- */
      char *p = uart3_line;
      while (*p==' '||*p=='\t'||*p=='\r'||*p=='\n') ++p;

      char cmd[32];
      size_t i = 0;
      while (i < sizeof(cmd)-1 && p[i] && p[i] != '\r' && p[i] != '\n') {
        cmd[i] = (char)toupper((unsigned char)p[i]);
        ++i;
      }
      cmd[i] = '\0';

      if (cmd[0] != '\0') {
        if (cmdq_push(cmd) != 0) {
          const char *busy = "BUSY\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)busy, (uint16_t)strlen(busy), 0xFFFF);
        }
      }
      uart3_idx = 0;
    } else {
      if (uart3_idx < sizeof(uart3_line)-1) {
        uart3_line[uart3_idx++] = ch;
      }
    }
    HAL_UART_Receive_IT(&huart3, &uart3_rx_ch, 1);  // re-arm at end
  }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  uint8_t ch = 'A';
  for(;;)
  {
	HAL_UART_Transmit(&huart3, (uint8_t *) &ch, 1, 0xFFFF);
	if(ch < 'Z')
		ch++;
	else ch ='A';
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    osDelay(5000);
  }
  /* USER CODE END 5 */
}

static void CommandQueue_TryDispatch(void)
{
  if (motionActive || g_steer_cmd.busy) return;

  // NEW: respect cooldown window
  if (HAL_GetTick() < g_cmd_cooldown_until_ms) return;

  if (cmdq_empty()) return;

  char cmdline[32];
  if (cmdq_pop(cmdline, sizeof cmdline) != 0) return;

  char *s = cmdline;
  while (*s == ' ' || *s == '\t') s++;
  char c0 = (char)toupper((unsigned char)s[0]);
  char c1 = (char)toupper((unsigned char)s[1]);
  const char *p = s + 2;
  while (*p == ' ' || *p == '\t') p++;

  // FR/FL = 90° proper align
  if (c0=='F' && c1=='L' && isdigit((unsigned char)s[2])) {
    int deg = atoi(&s[2]);
    if (deg <= 0) { uart3_send("ERR FL0\r\n"); return; }
    if (Servo_RequestBangBangTurn(+ (float)deg, 4750) == 0)
      uart3_send("ACK FL\r\n");
    else
      uart3_send("BUSY\r\n");
    return;
  }
  if (c0=='F' && c1=='R' && isdigit((unsigned char)s[2])) {
    int deg = atoi(&s[2]);
    if (deg <= 0) { uart3_send("ERR FR0\r\n"); return; }
    if (Servo_RequestBangBangTurn(- (float)deg, 4750) == 0)
      uart3_send("ACK FR\r\n");
    else
      uart3_send("BUSY\r\n");
    return;
  }

  // --- Reverse arbitrary degrees ---
  // BL<deg>  => reverse + left  => negative delta
  if (c0=='B' && c1=='L' && isdigit((unsigned char)s[2])) {
    int deg = atoi(&s[2]);            // BL45, BL120, ...
    if (deg <= 0) { uart3_send("ERR BL0\r\n"); return; }
    if (Servo_RequestBangBangTurnRev(-(float)deg, 4750) == 0)
      uart3_send("ACK BL\r\n");
    else
      uart3_send("BUSY\r\n");
    return;
  }

  // BR<deg>  => reverse + right => positive delta
  if (c0=='B' && c1=='R' && isdigit((unsigned char)s[2])) {
    int deg = atoi(&s[2]);            // BR30, BR180, ...
    if (deg <= 0) { uart3_send("ERR BR0\r\n"); return; }
    if (Servo_RequestBangBangTurnRev(+ (float)deg, 4750) == 0)
      uart3_send("ACK BR\r\n");
    else
      uart3_send("BUSY\r\n");
    return;
  }

  // Optional generic Lnn/Rnn
  if (c0=='L' && isdigit((unsigned char)s[1])) { int deg=atoi(&s[1]); if (deg<=0){uart3_send("ERR L0\r\n");return;} if (Servo_RequestBangBangTurn(+ (float)deg,4500)==0) uart3_send("ACK L\r\n"); else uart3_send("BUSY\r\n"); return; }
  if (c0=='R' && isdigit((unsigned char)s[1])) { int deg=atoi(&s[1]); if (deg<=0){uart3_send("ERR R0\r\n");return;} if (Servo_RequestBangBangTurn(- (float)deg,4500)==0) uart3_send("ACK R\r\n"); else uart3_send("BUSY\r\n"); return; }

  // ABS
  if (c0=='A' && c1=='B' && toupper((unsigned char)s[2])=='S') { int absd=atoi(&s[3])%360; if (Servo_RequestTurnTo((float)absd)==0) uart3_send("ACK ABS\r\n"); else uart3_send("BUSY\r\n"); return; }

  // Distance FW/BW
  if (c0=='F' && c1=='W') { int cm=0; if (sscanf(p,"%d",&cm)==1 && cm>0){ StartMoveCM(cm, DIR_FWD); char b[32]; int n=snprintf(b,sizeof b,"ACK FW %d\r\n",cm); HAL_UART_Transmit(&huart3,(uint8_t*)b,(uint16_t)n,HAL_MAX_DELAY);} else uart3_send("ERR (use FW###)\r\n"); return; }
  if (c0=='B' && c1=='W') { int cm=0; if (sscanf(p,"%d",&cm)==1 && cm>0){ StartMoveCM(cm, DIR_BACK); char b[32]; int n=snprintf(b,sizeof b,"ACK BW %d\r\n",cm); HAL_UART_Transmit(&huart3,(uint8_t*)b,(uint16_t)n,HAL_MAX_DELAY);} else uart3_send("ERR (use BW###)\r\n"); return; }

  uart3_send("CMD?\r\n");
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the ShowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
  /* Infinite loop */
  uint8_t hello[20] = "Hello World!\0";
  for(;;)
  {
	sprintf(hello, "%s\0", aRxBuffer);
	OLED_ShowString(10, 10, hello);
	OLED_Refresh_Gram();
    osDelay(1000);
  }
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
  /* USER CODE BEGIN motor */
  const TickType_t period = pdMS_TO_TICKS(MC_PERIOD_MS);
  TickType_t tick = xTaskGetTickCount();

  float integ = 0.0f;
  float prevErr = 0.0f;

  // simple RPS smoothing (3-sample EMA)
  float rpsA_f = 0.0f, rpsD_f = 0.0f;
  const float alpha = 0.5f; // 0=no filter, 1=heavy filter
  const int pwmBase = 5000; // your feed-forward
  uint32_t last_ms = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil(&tick, period);
    // --- ADD THIS GUARD ---
    if (g_steer_cmd.busy) {
        AllStop();
        continue;   // skip rest of loop until steering done
    }
    // dt (s)
    uint32_t now_ms = HAL_GetTick();
    float dt = (float)(now_ms - last_ms) / 1000.0f;
    if (dt <= 0.0f) dt = (float)MC_PERIOD_MS / 1000.0f;
    last_ms = now_ms;

    // filtered RPS
    rpsA_f = alpha * rpsA_f + (1.0f - alpha) * rpsA;
    rpsD_f = alpha * rpsD_f + (1.0f - alpha) * rpsD;

    // error = A - D (want 0)
    float err = rpsA_f - rpsD_f;

    // dt-scaled PI(D)
    integ += err * (KI_DIFF * dt);           // integral in PWM units
    if (integ > I_MAX) integ = I_MAX;
    if (integ < -I_MAX) integ = -I_MAX;

    float p = KP_DIFF * err;
    float d = KD_DIFF * (err - prevErr) / dt; // 0 if KD=0
    float corr = p + integ + d;
    prevErr = err;

    // apply symmetric correction + biases
    pwmA_val = pwmBase - (int)corr + biasA;
    pwmD_val = pwmBase + (int)corr + biasD;

    // clamp
    if (pwmA_val < PWM_MIN_CLAMP) pwmA_val = PWM_MIN_CLAMP;
    if (pwmA_val > PWM_MAX_CLAMP) pwmA_val = PWM_MAX_CLAMP;
    if (pwmD_val < PWM_MIN_CLAMP) pwmD_val = PWM_MIN_CLAMP;
    if (pwmD_val > PWM_MAX_CLAMP) pwmD_val = PWM_MAX_CLAMP;

    if (motionActive)
    {
        // direction-aware drive
        if (dir == DIR_FWD) {
            DriveForwardPWM(pwmA_val, pwmD_val);
        } else {
            DriveBackwardPWM(pwmA_val, pwmD_val);
        }
    }
    else
    {
        integ = 0.0f; // reset integral when idle
        prevErr = 0.0f;
    }

    // optional: quick telemetry
    //char msg[64]; int n=sprintf(msg,"%d,%d,%.0f\r\n",rpsA_f,rpsD_f,err);
   // HAL_UART_Transmit_IT(&huart3,(uint8_t*)msg,(uint16_t)n);
  }
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_encoder */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
  uint32_t cntA_prev = __HAL_TIM_GET_COUNTER(&htim2);
  uint32_t cntD_prev = __HAL_TIM_GET_COUNTER(&htim5);
  const int32_t modA = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim2) + 1; // 65536
  const int32_t modD = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim5) + 1;

  TickType_t tick = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20);   // 50 Hz
  uint32_t last_ms = HAL_GetTick();

  for (;;)
  {
	  vTaskDelayUntil(&tick, period);

	  uint32_t now_ms = HAL_GetTick();
	  float dt_ms = (float)(now_ms - last_ms);
	  if (dt_ms <= 0) dt_ms = 1.0f;

	  uint32_t cntA = __HAL_TIM_GET_COUNTER(&htim2);
	  uint32_t cntD = __HAL_TIM_GET_COUNTER(&htim5);

	  int32_t dA = (int32_t)cntA - (int32_t)cntA_prev;
	  if      (dA >  (modA/2)) dA -= modA;
	  else if (dA < -(modA/2)) dA += modA;

	  int32_t dD = (int32_t)cntD - (int32_t)cntD_prev;
	  if      (dD >  (modD/2)) dD -= modD;
	  else if (dD < -(modD/2)) dD += modD;

	  taskENTER_CRITICAL();
	  total_counts_A += dA;
	  total_counts_D += dD;
	  distance_cm_A  = (float)total_counts_A * CM_PER_COUNT;
	  distance_cm_D  = (float)total_counts_D * CM_PER_COUNT;
	  taskEXIT_CRITICAL();

	  // RPS over dt
	  float cpsA = fabsf((float)dA) * (1000.0f / dt_ms);
	  float cpsD = fabsf((float)dD) * (1000.0f / dt_ms);
	  rpsA = ((cpsA) / (float)COUNTS_PER_REV);
	  rpsD = ((cpsD) / (float)COUNTS_PER_REV);

	  cntA_prev = cntA;
	  cntD_prev = cntD;
	  last_ms   = now_ms;
  }
  /* USER CODE END encoder */
}

/* USER CODE BEGIN Header_distance */
/**
* @brief Function implementing the DistanceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_distance */
void distance(void *argument)
{
  /* USER CODE BEGIN distance */
  OLED_Clear();
  OLED_Refresh_Gram();

  char line[24];
  const TickType_t period = pdMS_TO_TICKS(100);   // 10 Hz UI
  TickType_t tick = xTaskGetTickCount();

  /* Infinite loop */
  for (;;)
  {
    // 1) Pop & execute next command (only here; single consumer)
    CommandQueue_TryDispatch();

    // 2) Safety/e-brake when target distance reached
    float a   = fabsf(distance_cm_A);
    float d   = fabsf(distance_cm_D);
    float avg = 0.5f * (a + d);

    if (motionActive && avg >= (float)targetdistance_cm)
    {
      AllStop();
      motionActive       = 0;
      targetdistance_cm  = 0;

      // brief active brake opposite to motion
      if (dir == DIR_FWD) {
        DriveBackwardPWM(4250, 4250);
        osDelay(73);
        AllStop();
      } else {
        DriveForwardPWM(4250, 4250);
        osDelay(77);
        AllStop();
      }

      StartCooldown(CMD_COOLDOWN_MS);
      // Optional: notify PC
      // const char *done = "DONE\r\n";
      // HAL_UART_Transmit(&huart3, (uint8_t*)done, (uint16_t)strlen(done), HAL_MAX_DELAY);
    }

    // 3) Simple OLED status (optional; comment out if not needed)
    //OLED_ShowString(10, 10, "                     ");
    //OLED_ShowString(10, 30, "                     ");

    //snprintf(line, sizeof line, "A Dis:%6.1f cm", a);
    //OLED_ShowString(10, 10, line);

    //snprintf(line, sizeof line, "B Dis:%6.1f cm", d);
    //OLED_ShowString(10, 30, line);

    //if (!motionActive) OLED_ShowString(10, 50, "STOP               ");
    //OLED_Refresh_Gram();

    vTaskDelayUntil(&tick, period);
  }
  /* USER CODE END distance */
}

/* USER CODE BEGIN Header_imu */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu */

void imu(void *argument)
{
  /* USER CODE BEGIN imu */
  OLED_ShowString(0,0,"ICM20948 init");
  OLED_Refresh_Gram();

  if (!ICM_addr) {
    if (!icm_probe()) {
      OLED_ShowString(0,16,"Probe FAIL");
      OLED_Refresh_Gram();
      for(;;) osDelay(1000);
    }
  }

  icm_wake_enable_gyro();
  OLED_ShowString(0,16,"Wake OK");
  OLED_Refresh_Gram();

  /* ---- Bias calibration (keep robot absolutely still) ---- */
  int32_t sum = 0;
  const int N = 400;        // ~2 s at 5 ms
  int16_t gx, gy, gz;
  for (int i = 0; i < N; ++i) {
    if (icm_read_gyro_xyz(&gx,&gy,&gz) == HAL_OK) sum += gz;
    osDelay(5);
  }
  _gyro_bias_lsb = (float)sum / (float)N;
  yaw_rate_dps   = 0.0f;
  yaw_angle_deg  = 0.0f;
  _gyro_last_ms  = HAL_GetTick();
  gyro_ready     = 1;

  OLED_ShowString(0,32,"Bias OK");
  OLED_Refresh_Gram();

  uint32_t lastPrint = 0;
  /* Infinite loop */
  for (;;)
  {
    if (icm_read_gyro_xyz(&gx,&gy,&gz) == HAL_OK)
    {
      /* remove bias, convert to dps, low-pass */
      float z_lsb = (float)gz - _gyro_bias_lsb;
      float z_dps = z_lsb / GYRO_SENS_LSB_PER_DPS;

      yaw_rate_dps = yaw_rate_dps*(1.0f - GYRO_LP_ALPHA) + z_dps*GYRO_LP_ALPHA;

      /* integrate heading */
      uint32_t now = HAL_GetTick();
      float dt = (now - _gyro_last_ms) / 1000.0f;
      _gyro_last_ms = now;
      yaw_angle_deg += yaw_rate_dps * dt;

      /* OLED debug every 200 ms */
      if (now - lastPrint >= 200) {
        char line[24];
        OLED_Clear();
        OLED_ShowString(0,0,"Gyro Z (dps):");
        snprintf(line, sizeof(line), "%7.2f", yaw_rate_dps);
        OLED_ShowString(0,16,line);
        OLED_ShowString(0,32,"Yaw (deg):");
        snprintf(line, sizeof(line), "%7.2f", yaw_angle_deg);
        OLED_ShowString(0,48,line);
        OLED_Refresh_Gram();
        lastPrint = now;
      }
    }
    osDelay(10);
  }
  /* USER CODE END imu */
}
/* USER CODE BEGIN Header_servomotor */
/**
* @brief Function implementing the ServoMotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servomotor */
void servomotor(void *argument)
{
  /* USER CODE BEGIN servomotor */
  steer_center();

  const uint32_t loop_ms = (1000U/STEER_LOOP_HZ);
  TickType_t last = xTaskGetTickCount();

  /* Infinite loop */
  for (;;)
  {
    if (!g_steer_cmd.pending) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (!gyro_ready) {
      g_steer_cmd.success = 0;
      g_steer_cmd.pending = 0;
      steer_center();
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    /* Latch a new job */
    g_steer_cmd.pending = 0;
    g_steer_cmd.busy    = 1;
    g_steer_cmd.success = 0;

    const float   target     = g_steer_cmd.target_heading;
    const uint8_t zero_after = g_steer_cmd.zero_yaw_after;
    const uint8_t use_bang = g_steer_cmd.bangbang;
    const uint16_t run_pwm = g_steer_cmd.drive_pwm ? g_steer_cmd.drive_pwm : 5000;
    const uint8_t rev_drive = g_steer_cmd.reverse_drive;   // <<< NEW

    // Clear it so next command defaults to PID again
    g_steer_cmd.bangbang = 0;
    g_steer_cmd.reverse_drive = 0;
    uint32_t started_ms = HAL_GetTick();

    /* IMPORTANT: reset the periodic scheduler baseline here */
    last = xTaskGetTickCount();                          // <<< add this

    float prev_err = smallest_err_deg(target, yaw_angle_deg);  // before loop

    for (;;) {
      vTaskDelayUntil(&last, pdMS_TO_TICKS(loop_ms));

      float err = smallest_err_deg(target, yaw_angle_deg);
      float abs_err = fabsf(err);

      // Decide “left” based on forward/reverse semantics (your rule)
      int need_left = rev_drive ? (err < 0.0f) : (err > 0.0f);

      // --- Coarse vs Fine ---
      if (abs_err > YAW_COARSE_THRESH_DEG) {
        // COARSE: bang-bang + high PWM
        uint16_t pwm = scale_pwm_from_err(abs_err);  // gives TURN_PWM_MAX here
        // lock wheels hard to the side for fast yaw
        steer_write_us(need_left ? STEER_US_LEFT : STEER_US_RIGHT);
        turn(need_left, pwm, rev_drive);
      } else {
        // FINE: proportional steer + reduced PWM + rate damping
        // steering angle = Kp*err - Kd*yaw_rate
        float wheel_cmd_deg = FINE_KP_STEER*err - FINE_KD_RATE*yaw_rate_dps;
        uint16_t usec = steer_deg_to_pulse(wheel_cmd_deg);
        steer_write_us(usec);

        uint16_t pwm = scale_pwm_from_err(abs_err);  // ramps 2600..5000
        spin_in_place(need_left, pwm, rev_drive);
      }

      // --- Exit windows ---
      if (abs_err <= YAW_STOP_DB_DEG) {
        // brief stop and settle
        AllStop();
        osDelay(30);
        steer_center();
        osDelay(30);

        // verify and micro-nudge toward target if needed
        float check_err = fabsf(smallest_err_deg(target, yaw_angle_deg));
        if (check_err > YAW_FINE_DB_DEG) {
          // one tiny corrective pulse, then re-evaluate next loop
          int nudge_left = rev_drive ? ( (smallest_err_deg(target, yaw_angle_deg) < 0.0f) )
                                     : ( (smallest_err_deg(target, yaw_angle_deg) > 0.0f) );
          spin_in_place(nudge_left, MICRO_NUDGE_PWM, rev_drive);
          osDelay(MICRO_NUDGE_MS);
          AllStop();
          // do NOT break; let loop re-check error and either finish or do another pass
        } else {
          g_steer_cmd.success = 1;
          break;
        }
      }

      // Simple overshoot damper: sign flip with non-trivial magnitude
      if ((prev_err > 0 && err < 0) || (prev_err < 0 && err > 0)) {
        // quick, tiny counter-pulse to kill momentum
        AllStop();
        osDelay(10);
      }
      prev_err = err;

      if ((HAL_GetTick() - started_ms) > STEER_CMD_TIMEOUT) {
        AllStop();
        osDelay(30);
        steer_center();
        g_steer_cmd.success = 0;
        break;
      }
    }

    /* Wheels straight; keep the car at the new heading */
    steer_center();
    // After the for(;;) loop in ServoAlignTask
    AllStop();

    /* By design we re-zero yaw so “now” becomes 0°, but the car
       physically remains 90° (or whatever you commanded) from the
       original heading. This is convenient for chained commands. */
    if (g_steer_cmd.success && zero_after) {
      yaw_angle_deg = 0.0f;
    }
    StartCooldown(CMD_COOLDOWN_MS);   // NEW: pause before next command
    g_steer_cmd.busy = 0;
  }
  /* USER CODE END servomotor */
}

/* USER CODE BEGIN Header_ir */
/**
* @brief Function implementing the IRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ir */
void ir(void *argument)
{
  /* USER CODE BEGIN ir */
  const char *hdr = "t_ms,ir_raw,ir_inv,ir_cm\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)hdr, (uint16_t)strlen(hdr), 10);

  TickType_t tick = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(50); // 20 Hz

  /* Infinite loop */
  for(;;)
  {
	    // Start one conversion now
	    if (HAL_ADC_Start(&hadc1) == HAL_OK)
	    {
	      // Wait (briefly) for it to finish
	      if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
	      {
	        uint32_t raw = HAL_ADC_GetValue(&hadc1);  // 0..4095
	        uint32_t inv = 4095u - raw;

	        int32_t d_cm =IrConvert((int32_t)raw);

	        char line[64];
	        int n = snprintf(line, sizeof(line), "%lu,%lu,%lu,%ld\r\n",
	                         (unsigned long)HAL_GetTick(),
	                         (unsigned long)raw,
	                         (unsigned long)inv,
	                         (long)d_cm);
	        if (n > 0) HAL_UART_Transmit(&huart3, (uint8_t*)line, (uint16_t)n, 10);

	      }
	      HAL_ADC_Stop(&hadc1); // tidy up this conversion
	    }

	    vTaskDelayUntil(&tick, period);
  }
  /* USER CODE END ir */
}

/* USER CODE BEGIN Header_ultrasonic */
/**
* @brief Function implementing the UltrasonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ultrasonic */
void ultrasonic(void *argument)
{
  /* USER CODE BEGIN ultrasonic */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ultrasonic */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
