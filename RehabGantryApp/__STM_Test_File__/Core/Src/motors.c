/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c  (Motor Test — Standalone Replacement)
 * @brief          : Interactive motor speed test via UART commands
 ******************************************************************************
 *
 * HOW TO USE
 * ==========
 * 1. In STM32CubeIDE, replace the contents of main.c with this ENTIRE file
 * 2. Build & flash to your STM32F4
 * 3. Run motor_test.py on your PC  (or open any serial terminal at 115200)
 * 4. Send commands to drive each motor axis independently
 *
 * COMMAND PROTOCOL  (framed with < > like the real main.c)
 * ════════════════
 *   <X:50.0>    Set X motor to +50 mm/s  (positive = DIR pin HIGH)
 *   <X:-30.0>   Set X motor to -30 mm/s  (negative = DIR pin LOW)
 *   <X:0>       Stop X motor
 *   <Y:50.0>    Set Y motor speed
 *   <Z:50.0>    Set Z motor speed  (requires brake OFF first!)
 *   <B:0>       Unclamp Z-axis brake  (relay RESET)
 *   <B:1>       Clamp Z-axis brake    (relay SET) — also zeros Z speed
 *   <STOP>      Emergency stop all motors + clamp brake
 *
 * SAFETY
 * ══════
 * - Z motor will NOT drive while brake is clamped.  Send <B:0> first.
 * - Dead-man timeout: if no command received within DEADMAN_TIMEOUT_MS,
 *   all motors stop and brake clamps.  Set to 0 to disable.
 * - Speed is clamped to ±MAX_SPEED mm/s.
 * - No homing is performed — position is NOT tracked.
 * - Admittance physics (HAL_SYSTICK_Callback) is NOT active.
 *
 * LEDs
 * ════
 *   Green  (LD4) = System ready, accepting commands
 *   Orange (LD3) = Heartbeat — toggles with each telemetry print
 *   Blue   (LD6) = Z-axis brake is UNCLAMPED (motor can move)
 *   Red    (LD5) = Dead-man timeout triggered (all stopped)
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ============================================================================
//  ███╗   ███╗ ██████╗ ████████╗ ██████╗ ██████╗
//  ████████╗███████╗███████╗████████╗ ████╗
//  ████║██╔═══██╗╚══██╔══╝██╔═══██╗██╔══██╗ ╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝
//  ██╔████╔██║██║   ██║   ██║   ██║   ██║██████╔╝       ██║   █████╗  ███████╗
//  ██║ ██║╚██╔╝██║██║   ██║   ██║   ██║   ██║██╔══██╗       ██║   ██╔══╝
//  ╚════██║   ██║ ██║ ╚═╝ ██║╚██████╔╝   ██║   ╚██████╔╝██║  ██║       ██║
//  ███████╗███████║   ██║ ╚═╝     ╚═╝ ╚═════╝    ╚═╝    ╚═════╝ ╚═╝  ╚═╝ ╚═╝
//  ╚══════╝╚══════╝   ╚═╝
//                          CONFIG — TUNE THESE
// ============================================================================

// --- Motor Physics ---
#define PULSES_PER_MM 80.0f    // 800 pulses/rev ÷ 10 mm/rev lead screw
#define TIMER_CLOCK 1000000.0f // 1 MHz (APB prescaled)
#define MAX_SPEED 100.0f       // mm/s — absolute safety limit per axis

// --- Safety ---
// Dead-man timeout: auto-stop all motors if no UART command received
// within this period.  Set to 0 to disable.
#define DEADMAN_TIMEOUT_MS 10000 // 10 seconds (0 = disabled)

// --- Telemetry ---
#define TELEMETRY_RATE_MS 500 // Status print interval (~2 Hz)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// ============================================================================
//                    MOTOR TEST STATE
// ============================================================================

// --- UART Parsing (same pattern as real main.c) ---
uint8_t rx_char;
char rx_buffer[64];
volatile uint8_t rx_index = 0;
volatile uint8_t new_cmd_received = 0;

// --- UART Transmit ---
char tx_buf[256];

// --- Motor Speeds (mm/s, signed = direction) ---
float speed_X = 0.0f;
float speed_Y = 0.0f;
float speed_Z = 0.0f; // Only applied to hardware when brake is unclamped

// --- Brake State ---
uint8_t brake_clamped = 1; // 1 = clamped (safe default), 0 = unclamped

// --- Timing ---
uint32_t last_cmd_time = 0; // For dead-man timeout
uint32_t last_telemetry_time = 0;
uint8_t deadman_tripped = 0; // Latched until next valid command

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void Set_Motor_Speed(TIM_HandleTypeDef *htim, uint32_t channel,
                     GPIO_TypeDef *dir_port, uint16_t dir_pin,
                     float speed_mmps);
void Apply_Motor_X(void);
void Apply_Motor_Y(void);
void Apply_Motor_Z(void);
void Stop_All(void);
void UART_Print(const char *msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_HOST_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  // --- Arm UART interrupt receiver ---
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);

  // --- Start PWM timers with compare = 0 (motors stationary) ---
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // X
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Y
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Z

  // --- Brake starts CLAMPED (safe) ---
  HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin, GPIO_PIN_SET);
  brake_clamped = 1;

  // --- Startup Banner ---
  const char *banner = "\r\n"
                       "========================================\r\n"
                       "  MOTOR TEST CONSOLE  v1.0\r\n"
                       "========================================\r\n"
                       "  TIM3 CH1 → X motor  (PUL_X = PC6)\r\n"
                       "  TIM4 CH2 → Y motor  (PUL_Y = PB7)\r\n"
                       "  TIM2 CH2 → Z motor  (PUL_Z = PA1)\r\n"
                       "  DIR: PE10(X) PE11(Y) PE12(Z)\r\n"
                       "  Brake relay: PE13  (clamped at boot)\r\n"
                       "  UART: 115200 8N1\r\n"
                       "========================================\r\n"
                       "Commands:\r\n"
                       "  <X:speed>  <Y:speed>  <Z:speed>\r\n"
                       "  <B:0> unclamp  <B:1> clamp\r\n"
                       "  <STOP> emergency stop\r\n"
                       "========================================\r\n"
                       "[RDY] Waiting for commands...\r\n\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)banner, strlen(banner), 500);

  // --- Green LED → system ready ---
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);

  // Sync timers
  last_cmd_time = HAL_GetTick();
  last_telemetry_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    // ==========================================================
    //  COMMAND PROCESSING
    // ==========================================================
    if (new_cmd_received == 1) {
      float tmp_speed;
      int tmp_brake;

      // Reset dead-man timer and clear trip flag
      last_cmd_time = HAL_GetTick();
      deadman_tripped = 0;
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Red OFF

      // --- X axis ---
      if (sscanf(rx_buffer, "X:%f", &tmp_speed) == 1) {
        // Clamp
        if (tmp_speed > MAX_SPEED)
          tmp_speed = MAX_SPEED;
        if (tmp_speed < -MAX_SPEED)
          tmp_speed = -MAX_SPEED;

        speed_X = tmp_speed;
        Apply_Motor_X();

        snprintf(tx_buf, sizeof(tx_buf), "[ACK] X → %+.1f mm/s\r\n", speed_X);
        UART_Print(tx_buf);

        // --- Y axis ---
      } else if (sscanf(rx_buffer, "Y:%f", &tmp_speed) == 1) {
        if (tmp_speed > MAX_SPEED)
          tmp_speed = MAX_SPEED;
        if (tmp_speed < -MAX_SPEED)
          tmp_speed = -MAX_SPEED;

        speed_Y = tmp_speed;
        Apply_Motor_Y();

        snprintf(tx_buf, sizeof(tx_buf), "[ACK] Y → %+.1f mm/s\r\n", speed_Y);
        UART_Print(tx_buf);

        // --- Z axis ---
      } else if (sscanf(rx_buffer, "Z:%f", &tmp_speed) == 1) {
        if (tmp_speed > MAX_SPEED)
          tmp_speed = MAX_SPEED;
        if (tmp_speed < -MAX_SPEED)
          tmp_speed = -MAX_SPEED;

        speed_Z = tmp_speed;

        if (!brake_clamped) {
          Apply_Motor_Z();
          snprintf(tx_buf, sizeof(tx_buf), "[ACK] Z → %+.1f mm/s\r\n", speed_Z);
        } else {
          // Z speed stored but NOT applied — brake is on
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
          snprintf(tx_buf, sizeof(tx_buf),
                   "[WARN] Z speed stored (%+.1f) but brake is CLAMPED — send "
                   "<B:0> first\r\n",
                   speed_Z);
        }
        UART_Print(tx_buf);

        // --- Brake control ---
      } else if (sscanf(rx_buffer, "B:%d", &tmp_brake) == 1) {
        if (tmp_brake == 0) {
          // UNCLAMP
          brake_clamped = 0;
          HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin,
                            GPIO_PIN_RESET);
          HAL_Delay(50); // Let relay click open

          // Apply any stored Z speed now that brake is off
          Apply_Motor_Z();

          HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Blue ON

          snprintf(tx_buf, sizeof(tx_buf),
                   "[ACK] Brake → UNCLAMPED  (Z active: %+.1f mm/s)\r\n",
                   speed_Z);
          UART_Print(tx_buf);

        } else {
          // CLAMP — first stop Z motor, then engage brake
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
          speed_Z = 0.0f;
          HAL_Delay(50); // Let motor decelerate

          brake_clamped = 1;
          HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin,
                            GPIO_PIN_SET);

          HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Blue OFF

          UART_Print("[ACK] Brake → CLAMPED  (Z speed zeroed)\r\n");
        }

        // --- Emergency Stop ---
      } else if (strcmp(rx_buffer, "STOP") == 0) {
        Stop_All();
        UART_Print("[ACK] *** EMERGENCY STOP — all motors zeroed, brake "
                   "clamped ***\r\n");
      } else {
        snprintf(tx_buf, sizeof(tx_buf), "[ERR] Unknown command: \"%s\"\r\n",
                 rx_buffer);
        UART_Print(tx_buf);
      }

      // Clear flag to wait for next message
      new_cmd_received = 0;
    }

    // ==========================================================
    //  DEAD-MAN TIMEOUT
    // ==========================================================
#if DEADMAN_TIMEOUT_MS > 0
    if (!deadman_tripped &&
        (HAL_GetTick() - last_cmd_time > DEADMAN_TIMEOUT_MS)) {
      Stop_All();
      deadman_tripped = 1;
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); // Red ON

      snprintf(tx_buf, sizeof(tx_buf),
               "[SAFETY] Dead-man timeout (%d s) — all motors stopped. "
               "Send any command to resume.\r\n",
               DEADMAN_TIMEOUT_MS / 1000);
      UART_Print(tx_buf);
    }
#endif

    // ==========================================================
    //  PERIODIC TELEMETRY
    // ==========================================================
    if (HAL_GetTick() - last_telemetry_time >= TELEMETRY_RATE_MS) {
      last_telemetry_time += TELEMETRY_RATE_MS;

      // Toggle orange LED as heartbeat
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

      snprintf(tx_buf, sizeof(tx_buf),
               "[MOT] X:%+.1f Y:%+.1f Z:%+.1f | BRK:%s\r\n", speed_X, speed_Y,
               speed_Z, brake_clamped ? "CLAMPED" : "FREE");
      UART_Print(tx_buf);
    }

    HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

// ============================================================================
//                     MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief  Convert a signed speed (mm/s) into timer ARR + compare settings.
 *         Identical math to the admittance output stage in real main.c.
 *
 * @param  htim       Timer handle (TIM2/3/4)
 * @param  channel    Timer channel (TIM_CHANNEL_1 or TIM_CHANNEL_2)
 * @param  dir_port   Direction GPIO port
 * @param  dir_pin    Direction GPIO pin
 * @param  speed_mmps Signed speed in mm/s (+ = DIR HIGH, − = DIR LOW)
 */
void Set_Motor_Speed(TIM_HandleTypeDef *htim, uint32_t channel,
                     GPIO_TypeDef *dir_port, uint16_t dir_pin,
                     float speed_mmps) {
  if (speed_mmps == 0.0f) {
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
    return;
  }

  // Set direction
  HAL_GPIO_WritePin(dir_port, dir_pin,
                    speed_mmps > 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);

  float abs_speed = (speed_mmps > 0.0f) ? speed_mmps : -speed_mmps;
  float freq = abs_speed * PULSES_PER_MM;

  if (freq < 1.0f) {
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
    return;
  }

  uint32_t arr = (uint32_t)(TIMER_CLOCK / freq);
  __HAL_TIM_SET_AUTORELOAD(htim, arr);
  __HAL_TIM_SET_COMPARE(htim, channel, arr / 2); // 50 % duty
}

/** Apply current speed_X to X motor hardware */
void Apply_Motor_X(void) {
  Set_Motor_Speed(&htim3, TIM_CHANNEL_1, DIR_X_GPIO_Port, DIR_X_Pin, speed_X);
}

/** Apply current speed_Y to Y motor hardware */
void Apply_Motor_Y(void) {
  Set_Motor_Speed(&htim4, TIM_CHANNEL_2, DIR_Y_GPIO_Port, DIR_Y_Pin, speed_Y);
}

/** Apply current speed_Z to Z motor hardware (only if brake unclamped) */
void Apply_Motor_Z(void) {
  if (brake_clamped) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  } else {
    Set_Motor_Speed(&htim2, TIM_CHANNEL_2, DIR_Z_GPIO_Port, DIR_Z_Pin, speed_Z);
  }
}

/** Emergency stop: zero all motors, zero stored speeds, clamp brake */
void Stop_All(void) {
  speed_X = 0.0f;
  speed_Y = 0.0f;
  speed_Z = 0.0f;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  // Clamp brake
  brake_clamped = 1;
  HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Blue OFF
}

/** Blocking UART print helper */
void UART_Print(const char *msg) {
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}

// ============================================================================
//                     UART INTERRUPT CALLBACKS
//   (Same < > framing protocol as real main.c)
// ============================================================================

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (rx_char == '<') {
      // Start of a new message. Reset the buffer.
      rx_index = 0;
      new_cmd_received = 0;
    } else if (rx_char == '>') {
      // End of the message. Null-terminate and flag it.
      rx_buffer[rx_index] = '\0';
      new_cmd_received = 1;
    } else {
      // Standard character. Add it to the array safely.
      if (new_cmd_received == 0 && rx_index < 63 && rx_char != '\r' &&
          rx_char != '\n') {
        rx_buffer[rx_index] = (char)rx_char;
        rx_index++;
      }
    }

    // Re-arm the interrupt to catch the next character
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    // Clear all error flags and re-arm so UART never goes deaf
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  }
}

// ============================================================================
//              CubeIDE BOILERPLATE — DO NOT MODIFY
//   (Copied verbatim from your real main.c so the project builds)
// ============================================================================

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}



static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim4);
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable only the GPIO port clocks we actually need */
  __HAL_RCC_GPIOE_CLK_ENABLE();  // DIR pins + brake relay
  __HAL_RCC_GPIOD_CLK_ENABLE();  // LEDs
  __HAL_RCC_GPIOA_CLK_ENABLE();  // TIM2 CH2 (PUL_Z)
  __HAL_RCC_GPIOB_CLK_ENABLE();  // TIM4 CH2 (PUL_Y)
  __HAL_RCC_GPIOC_CLK_ENABLE();  // TIM3 CH1 (PUL_X)
  __HAL_RCC_GPIOH_CLK_ENABLE();  // HSE crystal

  /* --- Initial output levels --- */
  HAL_GPIO_WritePin(GPIOE, DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin, GPIO_PIN_SET); // Brake clamped
  HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);

  /* --- DIR_X, DIR_Y, DIR_Z → Push-pull outputs (GPIOE) --- */
  GPIO_InitStruct.Pin   = DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* --- RELAY_BRAKE → Open-drain output (GPIOE) --- */
  GPIO_InitStruct.Pin   = RELAY_BRAKE_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_BRAKE_GPIO_Port, &GPIO_InitStruct);

  /* --- LEDs: LD3, LD4, LD5, LD6 → Push-pull outputs (GPIOD) --- */
  GPIO_InitStruct.Pin   = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
}
#endif /* USE_FULL_ASSERT */
