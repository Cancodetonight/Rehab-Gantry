/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Travel limits (mm) — matched to physical gantry dimensions
#define POS_MIN_X 0.0f
#define POS_MAX_X 510.0f
#define POS_MIN_Y 0.0f
#define POS_MAX_Y 230.0f
#define POS_MIN_Z 0.0f
#define POS_MAX_Z 200.0f

// Maximum time (ms) to wait for a limit switch during homing
#define HOMING_TIMEOUT_MS 60000

// Homing speed (mm/s) — slow and safe
#define HOMING_SPEED_MMPS 8.0f

// Debounce: switch must stay triggered for this many consecutive reads
// to filter EMI noise from stepper motors
#define HOMING_DEBOUNCE_COUNT  5
#define HOMING_DEBOUNCE_MS    10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_char; // Stores the incoming single character temporarily

// --- UART Parsing Buffer ---
char rx_buffer[64];
volatile uint8_t rx_index = 0;
volatile uint8_t new_target_received = 0;

// --- Game Target Variables ---
volatile float target_X = 0.0f;
volatile float target_Y = 0.0f;
volatile float target_Z = 0.0f;
volatile float stiffness_K = 0.0f;
volatile int z_enable = 0; // NEW: 0 = Brake Clamped, 1 = Brake Unclamped

// --- Telemetry & State Variables ---
char tx_buffer[128];
volatile uint8_t tx_complete = 1; // 1 = ready to send, 0 = TX in progress
uint32_t last_telemetry_time = 0;

// These will eventually be controlled by your Admittance math
volatile float current_pos_X = 0.0f;
volatile float current_pos_Y = 0.0f;
volatile float current_pos_Z = 0.0f;

// ==========================================
//    4-CELL FORCE SENSOR CONFIGURATION
// ==========================================

// --- Tare Settings ---
#define TARE_SAMPLES     20
#define TARE_SETTLE_MS   2000

// --- Hardcoded Fallback Offsets ---
#define HARDCODED_OFFSET_1  0
#define HARDCODED_OFFSET_2  0
#define HARDCODED_OFFSET_3  0
#define HARDCODED_OFFSET_4  0

// --- Per-Cell Scale Factors (counts per Newton) ---
#define SCALE_1  50000.0f
#define SCALE_2  50000.0f
#define SCALE_3  50000.0f
#define SCALE_4  50000.0f

// --- 3x4 Decoupling Matrix ---
//   F_X = +C2 - C4           (right minus left)
//   F_Y = +C1 - C3           (forward minus backward)
//   F_Z = (+C1 - C2 - C3 - C4) / 4  (pull up = positive)
#define D_X1   0.0f
#define D_X2   1.0f
#define D_X3   0.0f
#define D_X4  -1.0f
#define D_Y1   1.0f
#define D_Y2   0.0f
#define D_Y3  -1.0f
#define D_Y4   0.0f
#define D_Z1   0.25f
#define D_Z2  -0.25f
#define D_Z3  -0.25f
#define D_Z4  -0.25f

// --- EMA Filter ---
#define EMA_ALPHA 0.3f

// --- Clamp threshold for 24-bit wrap detection ---
#define MAX_SANE_ZEROED 5000000

// --- 4-Cell State Variables ---
int32_t offset_1 = 0, offset_2 = 0, offset_3 = 0, offset_4 = 0;
int32_t raw_1 = 0, raw_2 = 0, raw_3 = 0, raw_4 = 0;
int32_t last_good_z1 = 0, last_good_z2 = 0, last_good_z3 = 0, last_good_z4 = 0;
float cell_scale_1 = SCALE_1, cell_scale_2 = SCALE_2;
float cell_scale_3 = SCALE_3, cell_scale_4 = SCALE_4;
float newton_1 = 0, newton_2 = 0, newton_3 = 0, newton_4 = 0;
float decouple[3][4] = {
    {D_X1, D_X2, D_X3, D_X4},
    {D_Y1, D_Y2, D_Y3, D_Y4},
    {D_Z1, D_Z2, D_Z3, D_Z4},
};
float ema_X = 0, ema_Y = 0, ema_Z = 0;
uint8_t ema_primed = 0;
uint8_t tare_done = 0;
uint8_t motor_test_mode = 0;  // When 1: forces zeroed, auto-pull only

// Final Force Data (Newtons) — read by SysTick for admittance math
volatile float force_X = 0.0f;
volatile float force_Y = 0.0f;
volatile float force_Z = 0.0f;

// ==========================================
//    MOTOR PHYSICS CONSTANTS
// ==========================================
#define MAX_VELOCITY 100.0f
#define PULSES_PER_MM 80.0f
#define TIMER_CLOCK 1000000.0f

volatile float velocity_X = 0.0f;
volatile float velocity_Y = 0.0f;
volatile float velocity_Z = 0.0f;

// Flag: prevents SysTick from overriding timer registers during homing
volatile uint8_t homing_active = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
uint8_t Try_Read_HX711(GPIO_TypeDef *dt_port, uint16_t dt_pin,
                       GPIO_TypeDef *sck_port, uint16_t sck_pin,
                       int32_t *out_data);
void Force_Tare(void);
void Force_Update(volatile float *fx, volatile float *fy, volatile float *fz);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  // Tell the STM32 to listen for exactly 1 byte of data into rx_char
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);

  // --- Pre-start all PWM timers with compare=0 (silent) ---
  // This ensures the timers are running before homing, so we can
  // control motors by just changing ARR/compare (same as gameplay).
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  // ==========================================
  //         THE HOMING RITUAL
  // ==========================================
  homing_active = 1;  // Tell SysTick to leave timers alone

  // --- Step 1: Home the Z-Axis (UPWARDS) ---
  // Unclamp the Z-axis brake so it can move
  HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin, GPIO_PIN_RESET);
  HAL_Delay(50); // Wait 50ms for the mechanical relay to click open

  // Set direction to move TOWARD the Z switch (TOP)
  HAL_GPIO_WritePin(DIR_Z_GPIO_Port, DIR_Z_Pin, GPIO_PIN_SET);

  // Set homing speed (timers are already running with compare=0)
  {
    float freq = HOMING_SPEED_MMPS * PULSES_PER_MM;
    uint32_t arr = (uint32_t)(TIMER_CLOCK / freq);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arr / 2);
  }

  // Wait until the Z-switch triggers (debounced) or timeout
  uint32_t home_start = HAL_GetTick();
  uint8_t z_debounce = 0;
  while (1) {
    if (HAL_GetTick() - home_start > HOMING_TIMEOUT_MS) {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
      Error_Handler();
    }
    if (HAL_GPIO_ReadPin(GPIOE, LIMIT_Z_Pin) == GPIO_PIN_SET) {
      z_debounce++;
      if (z_debounce >= HOMING_DEBOUNCE_COUNT) break; // Confirmed!
      HAL_Delay(HOMING_DEBOUNCE_MS);
    } else {
      z_debounce = 0; // Reset — was just noise
    }
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  current_pos_Z = 0.0f;

  // --- Step 2: Home the X and Y Axes (Simultaneously) ---
  HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_RESET); // X inverted to move Left toward switch
  HAL_GPIO_WritePin(DIR_Y_GPIO_Port, DIR_Y_Pin, GPIO_PIN_SET);

  // Set homing speed on X and Y (timers already running)
  {
    float freq = HOMING_SPEED_MMPS * PULSES_PER_MM;
    uint32_t arr = (uint32_t)(TIMER_CLOCK / freq);
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr / 2);
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, arr / 2);
  }

  uint8_t x_homed = 0;
  uint8_t y_homed = 0;
  uint8_t x_debounce = 0;
  uint8_t y_debounce = 0;

  // Loop until BOTH axes have confirmed switch hits (debounced) with timeout
  home_start = HAL_GetTick();
  while (x_homed == 0 || y_homed == 0) {
    if (HAL_GetTick() - home_start > HOMING_TIMEOUT_MS) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
      Error_Handler();
    }

    // --- X debounce ---
    if (x_homed == 0) {
      if (HAL_GPIO_ReadPin(GPIOE, LIMIT_X_Pin) == GPIO_PIN_SET) {
        x_debounce++;
        if (x_debounce >= HOMING_DEBOUNCE_COUNT) {
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
          current_pos_X = 0.0f;
          x_homed = 1;
        }
      } else {
        x_debounce = 0;
      }
    }

    // --- Y debounce ---
    if (y_homed == 0) {
      if (HAL_GPIO_ReadPin(GPIOE, LIMIT_Y_Pin) == GPIO_PIN_SET) {
        y_debounce++;
        if (y_debounce >= HOMING_DEBOUNCE_COUNT) {
          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
          current_pos_Y = 0.0f;
          y_homed = 1;
        }
      } else {
        y_debounce = 0;
      }
    }

    HAL_Delay(HOMING_DEBOUNCE_MS);
  }

  // ==========================================
  //        HOMING COMPLETE
  // ==========================================

  // Clamp the Z brake again until the Python game tells us to start
  HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin, GPIO_PIN_SET);

  // Ensure all motors are silent (timers already running from pre-start)
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  homing_active = 0;  // SysTick can now control motors

  // ==========================================
  //     FORCE SENSOR TARE (AUTO-ZERO)
  // ==========================================
  HAL_Delay(TARE_SETTLE_MS);
  Force_Tare();

  // Notify Python that homing is complete
  const char *homed_msg = "<HOMED>\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)homed_msg, strlen(homed_msg), 50);

  // Sync telemetry timer so it doesn't burst-fire after the homing delay
  last_telemetry_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    // --- Read Force Sensors ---
    // Motor test mode: ignore forces entirely, auto-pull handles movement
    // Normal mode: read forces when K > 0, zero when idle
    if (motor_test_mode) {
      Force_Update(&force_X, &force_Y, &force_Z);  // keep raw values fresh
      force_X = 0.0f;
      force_Y = 0.0f;
      force_Z = 0.0f;
    } else if (stiffness_K > 0.0f) {
      Force_Update(&force_X, &force_Y, &force_Z);
    } else {
      Force_Update(&force_X, &force_Y, &force_Z);
      force_X = 0.0f;
      force_Y = 0.0f;
      force_Z = 0.0f;
    }

    // --- Process Incoming Python Commands ---
    if (new_target_received == 1) {
      // Check for TARE command first
      if (strcmp(rx_buffer, "TARE") == 0) {
        HAL_Delay(500);
        Force_Tare();
        ema_primed = 0;
      } else if (strcmp(rx_buffer, "MTEST") == 0) {
        motor_test_mode = 1;  // Enable motor test (forces ignored)
      } else if (strcmp(rx_buffer, "MTEST_OFF") == 0) {
        motor_test_mode = 0;  // Disable motor test
      } else {
        // Parse target command: TGT:x,y,z|K:value|Z:enable
        float tmp_tx, tmp_ty, tmp_tz, tmp_k;
        int tmp_zen;

        int parsed = sscanf(rx_buffer, "TGT:%f,%f,%f|K:%f|Z:%d", &tmp_tx, &tmp_ty,
                            &tmp_tz, &tmp_k, &tmp_zen);

        if (parsed == 5) {
          __disable_irq();
          target_X = tmp_tx;
          target_Y = tmp_ty;
          target_Z = tmp_tz;
          stiffness_K = tmp_k;
          __enable_irq();

          z_enable = tmp_zen;

          if (z_enable == 1) {
            HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin,
                              GPIO_PIN_RESET);
          } else {
            HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin,
                              GPIO_PIN_SET);
          }
        }
      }

      new_target_received = 0;
    }

    // --- Transmit Telemetry to Python (~60 Hz) ---
    if (HAL_GetTick() - last_telemetry_time >= 16) {
      last_telemetry_time += 16; // Anchored increment prevents drift

      // Snapshot positions AND velocities atomically
      float tx_pX, tx_pY, tx_pZ, tx_vX, tx_vY, tx_vZ;
      __disable_irq();
      tx_pX = current_pos_X;
      tx_pY = current_pos_Y;
      tx_pZ = current_pos_Z;
      tx_vX = velocity_X;
      tx_vY = velocity_Y;
      tx_vZ = velocity_Z;
      __enable_irq();

      // Velocity reads keep debug symbols alive — suppress unused warnings
      (void)tx_vX;
      (void)tx_vY;
      (void)tx_vZ;

      // Format matches Python regex: <POS:x,y,z|FRC:fx,fy,fz>
      snprintf(tx_buffer, sizeof(tx_buffer),
               "<POS:%.2f,%.2f,%.2f|FRC:%.2f,%.2f,%.2f>\n", tx_pX, tx_pY,
               tx_pZ, force_X, -force_Y, force_Z);

      // Send over UART (blocking — proven reliable)
      HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 10);
    }

    // Tiny delay to make the debug viewer readable
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
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

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void) {
  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {
  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3 | DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HX711_4_SCK_Pin | HX711_3_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD,
                    HX711_2_SCK_Pin | HX711_1_SCK_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin |
                        Audio_RST_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 DIR_X_Pin DIR_Y_Pin DIR_Z_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_X_Pin LIMIT_Y_Pin LIMIT_Z_Pin */
  GPIO_InitStruct.Pin = LIMIT_X_Pin | LIMIT_Y_Pin | LIMIT_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 HX711_4_DT_Pin HX711_3_DT_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | HX711_4_DT_Pin | HX711_3_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_BRAKE_Pin */
  GPIO_InitStruct.Pin = RELAY_BRAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_BRAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HX711_4_SCK_Pin HX711_3_SCK_Pin */
  GPIO_InitStruct.Pin = HX711_4_SCK_Pin | HX711_3_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HX711_2_SCK_Pin HX711_1_SCK_Pin LD4_Pin LD3_Pin
                           LD5_Pin LD6_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = HX711_2_SCK_Pin | HX711_1_SCK_Pin | LD4_Pin | LD3_Pin
                          | LD5_Pin | LD6_Pin | Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : HX711_2_DT_Pin HX711_1_DT_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = HX711_2_DT_Pin | HX711_1_DT_Pin | OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR_Y_GPIO_Port, DIR_Y_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR_Z_GPIO_Port, DIR_Z_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ============================================================================
//               4-CELL FORCE MODULE
// ============================================================================

uint8_t Try_Read_HX711(GPIO_TypeDef *dt_port, uint16_t dt_pin,
                       GPIO_TypeDef *sck_port, uint16_t sck_pin,
                       int32_t *out_data) {
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(dt_port, dt_pin) == GPIO_PIN_SET) {
    if (HAL_GetTick() - t0 > 200) return 0;
  }

  int32_t count = 0;
  __disable_irq();
  for (int i = 0; i < 24; i++) {
    HAL_GPIO_WritePin(sck_port, sck_pin, GPIO_PIN_SET);
    count = count << 1;
    HAL_GPIO_WritePin(sck_port, sck_pin, GPIO_PIN_RESET);
    if (HAL_GPIO_ReadPin(dt_port, dt_pin) == GPIO_PIN_SET) {
      count++;
    }
  }
  HAL_GPIO_WritePin(sck_port, sck_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(sck_port, sck_pin, GPIO_PIN_RESET);
  __enable_irq();

  if (count & 0x800000) {
    count |= (int32_t)0xFF000000;
  }

  *out_data = count;
  return 1;
}

void Force_Tare(void) {
  int64_t sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;
  int good1 = 0, good2 = 0, good3 = 0, good4 = 0;
  int32_t val;

  for (int i = 0; i < TARE_SAMPLES; i++) {
    if (Try_Read_HX711(HX711_1_DT_GPIO_Port, HX711_1_DT_Pin,
                       HX711_1_SCK_GPIO_Port, HX711_1_SCK_Pin, &val)) {
      sum1 += val; good1++;
    }
    if (Try_Read_HX711(HX711_2_DT_GPIO_Port, HX711_2_DT_Pin,
                       HX711_2_SCK_GPIO_Port, HX711_2_SCK_Pin, &val)) {
      sum2 += val; good2++;
    }
    if (Try_Read_HX711(HX711_3_DT_GPIO_Port, HX711_3_DT_Pin,
                       HX711_3_SCK_GPIO_Port, HX711_3_SCK_Pin, &val)) {
      sum3 += val; good3++;
    }
    if (Try_Read_HX711(HX711_4_DT_GPIO_Port, HX711_4_DT_Pin,
                       HX711_4_SCK_GPIO_Port, HX711_4_SCK_Pin, &val)) {
      sum4 += val; good4++;
    }
    HAL_Delay(20);
  }

  offset_1 = (good1 > 0) ? (int32_t)(sum1 / good1) : HARDCODED_OFFSET_1;
  offset_2 = (good2 > 0) ? (int32_t)(sum2 / good2) : HARDCODED_OFFSET_2;
  offset_3 = (good3 > 0) ? (int32_t)(sum3 / good3) : HARDCODED_OFFSET_3;
  offset_4 = (good4 > 0) ? (int32_t)(sum4 / good4) : HARDCODED_OFFSET_4;

  last_good_z1 = 0; last_good_z2 = 0;
  last_good_z3 = 0; last_good_z4 = 0;
  tare_done = 1;
}

void Force_Update(volatile float *fx, volatile float *fy, volatile float *fz) {
  int32_t val;

  if (Try_Read_HX711(HX711_1_DT_GPIO_Port, HX711_1_DT_Pin,
                     HX711_1_SCK_GPIO_Port, HX711_1_SCK_Pin, &val))
    raw_1 = val;

  if (Try_Read_HX711(HX711_2_DT_GPIO_Port, HX711_2_DT_Pin,
                     HX711_2_SCK_GPIO_Port, HX711_2_SCK_Pin, &val))
    raw_2 = val;

  if (Try_Read_HX711(HX711_3_DT_GPIO_Port, HX711_3_DT_Pin,
                     HX711_3_SCK_GPIO_Port, HX711_3_SCK_Pin, &val))
    raw_3 = val;

  if (Try_Read_HX711(HX711_4_DT_GPIO_Port, HX711_4_DT_Pin,
                     HX711_4_SCK_GPIO_Port, HX711_4_SCK_Pin, &val))
    raw_4 = val;

  int32_t z1 = raw_1 - offset_1;
  int32_t z2 = raw_2 - offset_2;
  int32_t z3 = raw_3 - offset_3;
  int32_t z4 = raw_4 - offset_4;

  if (z1 > MAX_SANE_ZEROED || z1 < -MAX_SANE_ZEROED) z1 = last_good_z1;
  else last_good_z1 = z1;

  if (z2 > MAX_SANE_ZEROED || z2 < -MAX_SANE_ZEROED) z2 = last_good_z2;
  else last_good_z2 = z2;

  if (z3 > MAX_SANE_ZEROED || z3 < -MAX_SANE_ZEROED) z3 = last_good_z3;
  else last_good_z3 = z3;

  if (z4 > MAX_SANE_ZEROED || z4 < -MAX_SANE_ZEROED) z4 = last_good_z4;
  else last_good_z4 = z4;

  newton_1 = (float)z1 / cell_scale_1;
  newton_2 = (float)z2 / cell_scale_2;
  newton_3 = (float)z3 / cell_scale_3;
  newton_4 = (float)z4 / cell_scale_4;

  float raw_fx = decouple[0][0] * newton_1 + decouple[0][1] * newton_2 +
                 decouple[0][2] * newton_3 + decouple[0][3] * newton_4;
  float raw_fy = decouple[1][0] * newton_1 + decouple[1][1] * newton_2 +
                 decouple[1][2] * newton_3 + decouple[1][3] * newton_4;
  float raw_fz = decouple[2][0] * newton_1 + decouple[2][1] * newton_2 +
                 decouple[2][2] * newton_3 + decouple[2][3] * newton_4;

  if (!ema_primed) {
    ema_X = raw_fx;
    ema_Y = raw_fy;
    ema_Z = raw_fz;
    ema_primed = 1;
  } else {
    ema_X = EMA_ALPHA * raw_fx + (1.0f - EMA_ALPHA) * ema_X;
    ema_Y = EMA_ALPHA * raw_fy + (1.0f - EMA_ALPHA) * ema_Y;
    ema_Z = EMA_ALPHA * raw_fz + (1.0f - EMA_ALPHA) * ema_Z;
  }
  *fx = ema_X;
  *fy = ema_Y;
  *fz = ema_Z;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (rx_char == '<') {
      // Start of a new message. Reset the buffer.
      rx_index = 0;
      new_target_received = 0;
    } else if (rx_char == '>') {
      // End of the message. Null-terminate and flag it.
      rx_buffer[rx_index] = '\0';
      new_target_received = 1;
    } else {
      // Standard character. Add it to the array safely, ignoring \r and \n to
      // prevent null terminator overwrite
      if (new_target_received == 0 && rx_index < 63 && rx_char != '\r' &&
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
    // On STM32F4, reading SR then DR clears all error flags (ORE, NE, FE, PE)
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    // Re-arm the receiver so UART never goes permanently deaf
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    tx_complete = 1; // Buffer is free, allow next telemetry send
  }
}

// ==========================================
//     DIRECTION-AWARE VELOCITY CONTROLLER
// ==========================================
// Instead of spring-mass-damper admittance, this uses a simple rule:
//  - Correct-direction force ≥4N → move at 20 mm/s toward target
//  - Correct-direction force <4N → scale proportionally
//  - Near target (<20mm) → slow to 5 mm/s constant approach
//  - Wrong-direction force → slight compliance, then rigid
//  - No force → gentle auto-pull toward target (for MOVE_TO_START)

#define CTRL_MAX_SPEED    20.0f   // mm/s at full force
#define CTRL_FORCE_SAT    4.0f    // N at which max speed is reached
#define CTRL_NEAR_DIST    20.0f   // mm: switch to approach speed
#define CTRL_NEAR_SPEED   5.0f    // mm/s: constant approach speed near target
#define CTRL_ARRIVE_DIST  1.0f    // mm: close enough = arrived
#define CTRL_FORCE_DEAD   0.3f    // N: below this = no user input
#define CTRL_COMPLY_MAX   3.0f    // mm/s: max wrong-direction compliance speed
#define CTRL_AUTO_SPEED   20.0f   // mm/s: auto-pull toward target when no user force
#define CTRL_MAX_ACCEL    100.0f  // mm/s²: acceleration ramp limit

static inline float fabsf_safe(float x) { return x >= 0.0f ? x : -x; }

/**
 * @brief  Compute the desired velocity for one axis.
 * @param  force   Measured force on this axis (N, signed)
 * @param  pos     Current position (mm)
 * @param  target  Target position (mm)
 * @retval Desired velocity (mm/s, signed toward target)
 */
static float Compute_Desired_Velocity(float force, float pos, float target) {
  float error = target - pos;
  float dist = fabsf_safe(error);
  float dir = (error >= 0.0f) ? 1.0f : -1.0f;
  float force_mag = fabsf_safe(force);
  float force_toward = force * dir; // positive = helping

  // Arrived at target
  if (dist < CTRL_ARRIVE_DIST) {
    return 0.0f;
  }

  // --- Auto-pull: MOVE_TO_START (K>=40) or motor test mode ---
  // No force needed, gantry navigates autonomously to target
  if (motor_test_mode || stiffness_K >= 40.0f) {
    float speed = CTRL_AUTO_SPEED;
    if (dist < CTRL_NEAR_DIST) speed = CTRL_NEAR_SPEED;
    return speed * dir;
  }

  // --- Normal mode: force-driven only ---
  // Below deadband = no movement
  if (force_mag < CTRL_FORCE_DEAD) {
    return 0.0f;
  }

  // Force in correct direction — proportional speed
  if (force_toward > 0.0f) {
    float speed = (force_toward / CTRL_FORCE_SAT) * CTRL_MAX_SPEED;
    if (speed > CTRL_MAX_SPEED) speed = CTRL_MAX_SPEED;
    if (dist < CTRL_NEAR_DIST) {
      if (speed > CTRL_NEAR_SPEED) speed = CTRL_NEAR_SPEED;
    }
    return speed * dir;
  }

  // Force in wrong direction — slight compliance, capped
  float comply = force_mag * 1.0f;
  if (comply > CTRL_COMPLY_MAX) comply = CTRL_COMPLY_MAX;
  return (force >= 0.0f ? 1.0f : -1.0f) * comply;
}

/**
 * @brief  Apply velocity to a stepper motor (direction pin + PWM frequency).
 */
static void Apply_Motor(float vel, TIM_HandleTypeDef *htim, uint32_t ch,
                        GPIO_TypeDef *dir_port, uint16_t dir_pin, uint8_t invert_dir) {
  if (vel == 0.0f) {
    if (__HAL_TIM_GET_COMPARE(htim, ch) != 0) {
      __HAL_TIM_SET_COMPARE(htim, ch, 0);
    }
  } else {
    uint8_t pos_dir = invert_dir ? GPIO_PIN_SET : GPIO_PIN_RESET;
    uint8_t neg_dir = invert_dir ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(dir_port, dir_pin,
                      vel > 0.0f ? pos_dir : neg_dir);
    float freq = fabsf_safe(vel) * PULSES_PER_MM;
    if (freq < 1.0f) {
      if (__HAL_TIM_GET_COMPARE(htim, ch) != 0) {
        __HAL_TIM_SET_COMPARE(htim, ch, 0);
      }
    } else {
      uint32_t arr = (uint32_t)(TIMER_CLOCK / freq);
      // Only write to timer registers if the frequency changed or it was previously stopped
      if (__HAL_TIM_GET_AUTORELOAD(htim) != arr || __HAL_TIM_GET_COMPARE(htim, ch) == 0) {
        __HAL_TIM_SET_AUTORELOAD(htim, arr);
        __HAL_TIM_SET_COMPARE(htim, ch, arr / 2);
      }
    }
  }
}

void HAL_SYSTICK_Callback(void) {
  // During homing, the main() loop controls timers directly — don't interfere
  if (homing_active) return;

  float dt = 0.001f;

  // If no game active (K=0), hold position — all motors off
  if (stiffness_K <= 0.0f) {
    velocity_X = 0.0f;
    velocity_Y = 0.0f;
    velocity_Z = 0.0f;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    return;
  }

  // --- X-Axis ---
  {
    float desired = Compute_Desired_Velocity(force_X, current_pos_X, target_X);
    float dv = desired - velocity_X;
    float max_dv = CTRL_MAX_ACCEL * dt;
    if (dv > max_dv) dv = max_dv;
    if (dv < -max_dv) dv = -max_dv;
    velocity_X += dv;

    current_pos_X += velocity_X * dt;
    if (current_pos_X < POS_MIN_X) { current_pos_X = POS_MIN_X; velocity_X = 0.0f; }
    if (current_pos_X > POS_MAX_X) { current_pos_X = POS_MAX_X; velocity_X = 0.0f; }

    Apply_Motor(velocity_X, &htim3, TIM_CHANNEL_1, DIR_X_GPIO_Port, DIR_X_Pin, 1); // X is inverted
  }

  // --- Y-Axis ---
  {
    float desired = Compute_Desired_Velocity(force_Y, current_pos_Y, target_Y);
    float dv = desired - velocity_Y;
    float max_dv = CTRL_MAX_ACCEL * dt;
    if (dv > max_dv) dv = max_dv;
    if (dv < -max_dv) dv = -max_dv;
    velocity_Y += dv;

    current_pos_Y += velocity_Y * dt;
    if (current_pos_Y < POS_MIN_Y) { current_pos_Y = POS_MIN_Y; velocity_Y = 0.0f; }
    if (current_pos_Y > POS_MAX_Y) { current_pos_Y = POS_MAX_Y; velocity_Y = 0.0f; }

    Apply_Motor(velocity_Y, &htim4, TIM_CHANNEL_2, DIR_Y_GPIO_Port, DIR_Y_Pin, 0);
  }

  // --- Z-Axis (with brake interlock) ---
  if (z_enable == 1) {
    float desired = Compute_Desired_Velocity(-force_Z, current_pos_Z, target_Z);
    float dv = desired - velocity_Z;
    float max_dv = CTRL_MAX_ACCEL * dt;
    if (dv > max_dv) dv = max_dv;
    if (dv < -max_dv) dv = -max_dv;
    velocity_Z += dv;

    current_pos_Z += velocity_Z * dt;
    if (current_pos_Z < POS_MIN_Z) { current_pos_Z = POS_MIN_Z; velocity_Z = 0.0f; }
    if (current_pos_Z > POS_MAX_Z) { current_pos_Z = POS_MAX_Z; velocity_Z = 0.0f; }

    Apply_Motor(velocity_Z, &htim2, TIM_CHANNEL_2, DIR_Z_GPIO_Port, DIR_Z_Pin, 0);
  } else {
    // SAFETY INTERLOCK: Brake is clamped.
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    velocity_Z = 0.0f;
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
