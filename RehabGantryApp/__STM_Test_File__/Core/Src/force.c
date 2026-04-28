/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c  (Force Sensor Calibration Test — 4 Cell Cross)
 * @brief          : Standalone force calibration with 4 HX711 load cells
 ******************************************************************************
 *
 * HOW TO USE
 * ==========
 * 1. In STM32CubeIDE, replace contents of main.c with this ENTIRE file
 * 2. Build & flash to your STM32F4
 * 3. Run force_test.py on your PC  (or any serial terminal at 115200)
 * 4. Keep hands OFF the gantry during the ~3 s tare (auto-zero) sequence
 * 5. Observe raw, zeroed, and calibrated readings in the terminal
 * 6. Tune the constants in the CALIBRATION CONFIG section below
 *
 * SENSOR LAYOUT (cross pattern, viewed from above)
 * ════════════════════════════════════════════════
 *
 *                   3 (top)           ← +Y
 *              4 (left)   2 (right)   ← +X
 *                   1 (bottom)
 *
 *              Joystick at center
 *              Z = vertical (down into cells)
 *
 * FORCE COMPUTATION
 * ═════════════════
 *   1. Read 4 HX711 raw counts (with 24-bit unwrap)
 *   2. Subtract tare offsets  → zeroed counts
 *   3. Divide by per-cell scale  → per-cell Newtons (n1..n4)
 *   4. Multiply by 3×4 decoupling matrix  → F_X, F_Y, F_Z
 *   5. Apply EMA low-pass filter
 *
 * SAFETY
 * ══════
 * - Motors are NOT driven.  Timers initialised but PWM never started.
 * - Z-axis brake stays CLAMPED.
 * - LEDs:
 *       Green  (LD4) = Tare complete, monitoring active
 *       Orange (LD3) = Heartbeat — toggles every print cycle
 *       Red    (LD5) = A sensor timed out during tare
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

// ============================================================================
//                        4-CELL CROSS CONFIGURATION
// ============================================================================

// --- Tare (Auto-Zero) Settings ---
#define USE_AUTO_TARE    1
#define TARE_SAMPLES     20
#define TARE_TIMEOUT_MS  5000
#define TARE_SETTLE_MS   2000

// --- Hardcoded Fallback Offsets ---
#define HARDCODED_OFFSET_1  0
#define HARDCODED_OFFSET_2  0
#define HARDCODED_OFFSET_3  0
#define HARDCODED_OFFSET_4  0

// --- Per-Cell Scale Factors  (raw counts per Newton) ---
// HOW TO CALIBRATE:
//   1. Hang a known weight (e.g. 1 kg = 9.81 N) on ONE cell only
//   2. Read the ZEROED column for that cell
//   3. scale = zeroed_counts / known_force_in_N
//   4. Update the value below and re-flash
#define SCALE_1  50000.0f   // Cell 1 (bottom)
#define SCALE_2  50000.0f   // Cell 2 (right)
#define SCALE_3  50000.0f   // Cell 3 (top)
#define SCALE_4  50000.0f   // Cell 4 (left)

// --- 3×4 Decoupling Matrix ---
// From testing:
//   Push RIGHT  → C2 increases, C4 decreases  → F_X = +C2 − C4
//   Push FWD    → C1 increases, C3 decreases   → F_Y = +C1 − C3
//   Pull UP     → C1 increases, C2/C3/C4 decrease → F_Z = +C1 − C2 − C3 − C4
//
//                        cell_1   cell_2   cell_3   cell_4
#define D_X1   0.0f  //  F_X  =  0·n1 + 1·n2 + 0·n3 − 1·n4
#define D_X2   1.0f
#define D_X3   0.0f
#define D_X4  -1.0f
#define D_Y1   1.0f  //  F_Y  = +1·n1 + 0·n2 − 1·n3 + 0·n4
#define D_Y2   0.0f
#define D_Y3  -1.0f
#define D_Y4   0.0f
#define D_Z1   0.25f //  F_Z  = (+C1 − C2 − C3 − C4) / 4   (pull up = positive)
#define D_Z2  -0.25f
#define D_Z3  -0.25f
#define D_Z4  -0.25f

// --- EMA Filter ---
#define USE_EMA_FILTER   1
#define EMA_ALPHA        0.3f

// --- Print Rate ---
#define PRINT_INTERVAL_MS 200   // ~5 Hz

// --- Motor Physics (not used, but needed for timer init) ---
#define PULSES_PER_MM    80.0f
#define TIMER_CLOCK      1000000.0f
#define MAX_SPEED        100.0f
#define DEADMAN_TIMEOUT_MS 0
#define TELEMETRY_RATE_MS  500

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

// ============================================================================
//                  FORCE MODULE STATE
// ============================================================================

// Active calibration offsets (set by tare or hardcoded fallback)
int32_t offset_1 = 0, offset_2 = 0, offset_3 = 0, offset_4 = 0;

// Last raw HX711 readings (signed 24-bit, as-is from sensor)
int32_t raw_1 = 0, raw_2 = 0, raw_3 = 0, raw_4 = 0;

// Last known-good zeroed values (for clamping when C4 wraps)
int32_t last_good_z1 = 0, last_good_z2 = 0, last_good_z3 = 0, last_good_z4 = 0;

// Max reasonable zeroed counts — anything beyond is clamped to last-good.
// C3 swings ±1.5M on normal pushes (high sensitivity), so threshold
// must be above that.  C4 wraps produce jumps of 8M+, still caught.
// 5,000,000 / 50,000 = 100 N limit — way beyond any hand force.
#define MAX_SANE_ZEROED 5000000

// Per-cell Newtons (after scale, before matrix)
float newton_1 = 0.0f, newton_2 = 0.0f, newton_3 = 0.0f, newton_4 = 0.0f;

// Final decoupled + filtered forces (Newtons)
volatile float force_X = 0.0f;
volatile float force_Y = 0.0f;
volatile float force_Z = 0.0f;

// EMA filter state
float ema_X = 0.0f, ema_Y = 0.0f, ema_Z = 0.0f;
uint8_t ema_primed = 0;

// Decoupling matrix stored in RAM (debugger/runtime tunable)
float decouple[3][4] = {
    {D_X1, D_X2, D_X3, D_X4},
    {D_Y1, D_Y2, D_Y3, D_Y4},
    {D_Z1, D_Z2, D_Z3, D_Z4},
};

// Scale factors in RAM (debugger tunable)
float scale_1 = SCALE_1;
float scale_2 = SCALE_2;
float scale_3 = SCALE_3;
float scale_4 = SCALE_4;

// UART transmit buffer
char tx_buf[512];

// Tare diagnostic flags
uint8_t tare_fail_1 = 0, tare_fail_2 = 0, tare_fail_3 = 0, tare_fail_4 = 0;

// UART Rx (for Python commands)
uint8_t rx_char;
char rx_buffer[64];
volatile uint8_t rx_index = 0;
volatile uint8_t new_cmd_received = 0;

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
void UART_Print(const char *msg);
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // SAFETY: Do NOT start motor PWM timers. Brake stays clamped.

  // Arm UART interrupt receiver (for Python commands)
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);

  // --- Startup Banner ---
  const char *banner = "\r\n"
                       "========================================\r\n"
                       "  FORCE SENSOR CALIBRATION  v2.1\r\n"
                       "  4-Cell Cross — 24-bit unwrap fix\r\n"
                       "========================================\r\n"
                       "  Cell 1 (bottom): PD11/PD10\r\n"
                       "  Cell 2 (right):  PD9/PD8\r\n"
                       "  Cell 3 (top):    PB15/PB14\r\n"
                       "  Cell 4 (left):   PB13/PB12\r\n"
                       "  UART: 115200 8N1\r\n"
                       "========================================\r\n\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)banner, strlen(banner), 200);

  // --- Tare ---
#if USE_AUTO_TARE

  snprintf(tx_buf, sizeof(tx_buf),
           "[TARE] Keep hands OFF! Auto-zeroing in %d ms...\r\n",
           TARE_SETTLE_MS);
  HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, strlen(tx_buf), 100);

  HAL_Delay(TARE_SETTLE_MS);

  Force_Tare();

  snprintf(tx_buf, sizeof(tx_buf),
           "[TARE] Offsets:  1=%ld  2=%ld  3=%ld  4=%ld\r\n",
           (long)offset_1, (long)offset_2, (long)offset_3, (long)offset_4);
  HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, strlen(tx_buf), 100);

  if (tare_fail_1 || tare_fail_2 || tare_fail_3 || tare_fail_4) {
    snprintf(tx_buf, sizeof(tx_buf),
             "[TARE] WARNING — Sensor timeout:%s%s%s%s\r\n",
             tare_fail_1 ? " C1" : "", tare_fail_2 ? " C2" : "",
             tare_fail_3 ? " C3" : "", tare_fail_4 ? " C4" : "");
    HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, strlen(tx_buf), 100);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
  }

  const char *tare_ok = "[TARE] Complete!\r\n\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)tare_ok, strlen(tare_ok), 50);

#else

  offset_1 = HARDCODED_OFFSET_1;
  offset_2 = HARDCODED_OFFSET_2;
  offset_3 = HARDCODED_OFFSET_3;
  offset_4 = HARDCODED_OFFSET_4;

  snprintf(tx_buf, sizeof(tx_buf),
           "[HARDCODED] Offsets:  1=%ld  2=%ld  3=%ld  4=%ld\r\n\r\n",
           (long)offset_1, (long)offset_2, (long)offset_3, (long)offset_4);
  HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, strlen(tx_buf), 100);

#endif

  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);

  uint32_t last_print = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    // --- Read + Calibrate + Filter ---
    Force_Update(&force_X, &force_Y, &force_Z);

    // --- Handle Python commands ---
    if (new_cmd_received == 1) {
      if (strcmp(rx_buffer, "TARE") == 0) {
        UART_Print("[CMD] Re-taring...\r\n");
        HAL_Delay(500);
        Force_Tare();
        ema_primed = 0;
        snprintf(tx_buf, sizeof(tx_buf),
                 "[TARE] New offsets: 1=%ld 2=%ld 3=%ld 4=%ld\r\n",
                 (long)offset_1, (long)offset_2, (long)offset_3,
                 (long)offset_4);
        UART_Print(tx_buf);
      } else if (strcmp(rx_buffer, "SCALES") == 0) {
        snprintf(tx_buf, sizeof(tx_buf),
                 "[SCALES] 1=%.2f 2=%.2f 3=%.2f 4=%.2f\r\n",
                 scale_1, scale_2, scale_3, scale_4);
        UART_Print(tx_buf);
      } else if (strcmp(rx_buffer, "MATRIX") == 0) {
        snprintf(tx_buf, sizeof(tx_buf),
                 "[MATRIX]\r\n"
                 "  X: [%+.4f %+.4f %+.4f %+.4f]\r\n"
                 "  Y: [%+.4f %+.4f %+.4f %+.4f]\r\n"
                 "  Z: [%+.4f %+.4f %+.4f %+.4f]\r\n",
                 decouple[0][0], decouple[0][1], decouple[0][2], decouple[0][3],
                 decouple[1][0], decouple[1][1], decouple[1][2], decouple[1][3],
                 decouple[2][0], decouple[2][1], decouple[2][2], decouple[2][3]);
        UART_Print(tx_buf);
      } else {
        int cell_id;
        float sval;
        if (sscanf(rx_buffer, "S%d:%f", &cell_id, &sval) == 2) {
          if (cell_id >= 1 && cell_id <= 4 && sval != 0.0f) {
            switch (cell_id) {
              case 1: scale_1 = sval; break;
              case 2: scale_2 = sval; break;
              case 3: scale_3 = sval; break;
              case 4: scale_4 = sval; break;
            }
            snprintf(tx_buf, sizeof(tx_buf),
                     "[ACK] Scale %d = %.2f\r\n", cell_id, sval);
            UART_Print(tx_buf);
          }
        }
      }
      new_cmd_received = 0;
    }

    // --- Print telemetry ---
    if (HAL_GetTick() - last_print >= PRINT_INTERVAL_MS) {
      last_print += PRINT_INTERVAL_MS;

      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

      int32_t z1 = raw_1 - offset_1;
      int32_t z2 = raw_2 - offset_2;
      int32_t z3 = raw_3 - offset_3;
      int32_t z4 = raw_4 - offset_4;

      snprintf(
          tx_buf, sizeof(tx_buf),
          "R:%ld,%ld,%ld,%ld | Z:%ld,%ld,%ld,%ld | "
          "N:%.3f,%.3f,%.3f,%.3f | F:%.3f,%.3f,%.3f\r\n",
          (long)raw_1, (long)raw_2, (long)raw_3, (long)raw_4,
          (long)z1, (long)z2, (long)z3, (long)z4,
          newton_1, newton_2, newton_3, newton_4,
          force_X, force_Y, force_Z);
      HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, strlen(tx_buf), 50);
    }

    HAL_Delay(5);
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2S3_Init(void)
{
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim4);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|DIR_X_Pin|DIR_Y_Pin|DIR_Z_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY_BRAKE_GPIO_Port, RELAY_BRAKE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, HX711_4_SCK_Pin|HX711_3_SCK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, HX711_2_SCK_Pin|HX711_1_SCK_Pin|LD4_Pin|LD3_Pin
                          |LD5_Pin|LD6_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_3|DIR_X_Pin|DIR_Y_Pin|DIR_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LIMIT_X_Pin|LIMIT_Y_Pin|LIMIT_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2|HX711_4_DT_Pin|HX711_3_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RELAY_BRAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_BRAKE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HX711_4_SCK_Pin|HX711_3_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HX711_2_SCK_Pin|HX711_1_SCK_Pin|LD4_Pin|LD3_Pin
                          |LD5_Pin|LD6_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HX711_2_DT_Pin|HX711_1_DT_Pin|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

// ============================================================================
//               FORCE MODULE — 4 Cell Cross with 24-bit Unwrap
// ============================================================================

/**
 * @brief  Read one HX711 via bit-bang GPIO.
 * @retval 1 = success,  0 = timeout
 */
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
  // 25th pulse → gain 128 channel A
  HAL_GPIO_WritePin(sck_port, sck_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(sck_port, sck_pin, GPIO_PIN_RESET);
  __enable_irq();

  // Sign-extend 24-bit to 32-bit
  if (count & 0x800000) {
    count |= (int32_t)0xFF000000;
  }

  *out_data = count;
  return 1;
}

/**
 * @brief  Unwrap a 24-bit signed value relative to a previous reading.
 *
 * HX711 outputs signed 24-bit: range −8388608 to +8388607.
 * When the count crosses this boundary (e.g. C4 near +8.1M increasing
 * past +8.38M), the sign-extended value jumps by ~16.7M.
 *
 * This function detects the discontinuity in the delta between consecutive
 * readings and adds/subtracts 16777216 to make the transition smooth.
 * The returned "unwrapped" value may exceed the 24-bit range, but that's
 * fine in a 32-bit int and all downstream math (zeroing, scaling) works.
 */
int32_t Unwrap_24bit(int32_t current, int32_t previous) {
  int32_t delta = current - previous;

  // If delta jumped by ~+16M, the sensor wrapped negative→positive
  if (delta > 8388607) {
    delta -= 16777216;
  }
  // If delta jumped by ~−16M, the sensor wrapped positive→negative
  else if (delta < -8388608) {
    delta += 16777216;
  }

  return previous + delta;
}

/**
 * @brief  Tare all 4 cells by averaging TARE_SAMPLES readings.
 */
void Force_Tare(void) {
  int64_t sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;
  int good1 = 0, good2 = 0, good3 = 0, good4 = 0;
  int32_t val;

  tare_fail_1 = tare_fail_2 = tare_fail_3 = tare_fail_4 = 0;

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

  if (good1 == 0) tare_fail_1 = 1;
  if (good2 == 0) tare_fail_2 = 1;
  if (good3 == 0) tare_fail_3 = 1;
  if (good4 == 0) tare_fail_4 = 1;

  // Reset last-good zeroed values after re-tare
  last_good_z1 = 0;
  last_good_z2 = 0;
  last_good_z3 = 0;
  last_good_z4 = 0;
}

/**
 * @brief  Read all 4 cells, clamp, scale, apply matrix, filter.
 */
void Force_Update(volatile float *fx, volatile float *fy, volatile float *fz) {
  int32_t val;

  // --- Read all 4 HX711 sensors (raw, no unwrap) ---
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

  // --- Subtract offsets ---
  int32_t z1 = raw_1 - offset_1;
  int32_t z2 = raw_2 - offset_2;
  int32_t z3 = raw_3 - offset_3;
  int32_t z4 = raw_4 - offset_4;

  // --- Clamp: if any cell's zeroed value is unreasonably large,
  //     it means the raw reading wrapped around the 24-bit boundary
  //     (e.g. C4 resting near +8.1M crosses ±8,388,607).
  //     In that case, hold the last known-good value. ---
  if (z1 > MAX_SANE_ZEROED || z1 < -MAX_SANE_ZEROED) z1 = last_good_z1;
  else last_good_z1 = z1;

  if (z2 > MAX_SANE_ZEROED || z2 < -MAX_SANE_ZEROED) z2 = last_good_z2;
  else last_good_z2 = z2;

  if (z3 > MAX_SANE_ZEROED || z3 < -MAX_SANE_ZEROED) z3 = last_good_z3;
  else last_good_z3 = z3;

  if (z4 > MAX_SANE_ZEROED || z4 < -MAX_SANE_ZEROED) z4 = last_good_z4;
  else last_good_z4 = z4;

  // --- Scale to per-cell Newtons ---
  newton_1 = (float)z1 / scale_1;
  newton_2 = (float)z2 / scale_2;
  newton_3 = (float)z3 / scale_3;
  newton_4 = (float)z4 / scale_4;

  // --- 3×4 decoupling matrix ---
  float raw_fx = decouple[0][0] * newton_1 + decouple[0][1] * newton_2 +
                 decouple[0][2] * newton_3 + decouple[0][3] * newton_4;
  float raw_fy = decouple[1][0] * newton_1 + decouple[1][1] * newton_2 +
                 decouple[1][2] * newton_3 + decouple[1][3] * newton_4;
  float raw_fz = decouple[2][0] * newton_1 + decouple[2][1] * newton_2 +
                 decouple[2][2] * newton_3 + decouple[2][3] * newton_4;

  // --- EMA low-pass filter ---
#if USE_EMA_FILTER
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
#else
  *fx = raw_fx;
  *fy = raw_fy;
  *fz = raw_fz;
#endif
}

void UART_Print(const char *msg) {
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}

// ============================================================================
//                     UART INTERRUPT CALLBACKS
// ============================================================================

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (rx_char == '<') {
      rx_index = 0;
      new_cmd_received = 0;
    } else if (rx_char == '>') {
      rx_buffer[rx_index] = '\0';
      new_cmd_received = 1;
    } else {
      if (new_cmd_received == 0 && rx_index < 63 && rx_char != '\r' &&
          rx_char != '\n') {
        rx_buffer[rx_index] = (char)rx_char;
        rx_index++;
      }
    }
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
