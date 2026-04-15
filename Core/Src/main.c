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
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
volatile uint16_t vmin = 4095;
volatile uint16_t vmax = 0;
volatile uint16_t dbg_v = 0;

volatile uint16_t center = 0;     // counts
volatile float vrms_adc = 0.0f;   // Vrms at ADC_VAC
volatile float vrms_hot = 0.0f;   // Vrms at AC_HOT (estimated)
volatile float f_est = 0.0f;
volatile float k_used = 0.0f;

volatile uint16_t dbg_iac = 0;      // raw ADC code for PA1 (current channel)
volatile uint16_t iac_center = 0;   // center for current channel
volatile float vrms_iac_adc = 0.0f; // Vrms at ADC_IAC node (volts)
volatile float irms_loop = 0.0f;    // estimated loop current (Arms)
volatile float p_real = 0.0f;       // watts
volatile float s_apparent = 0.0f;   // VA
volatile float pf = 0.0f;           // unitless
static uint32_t warmup = 0;
volatile float pf_filt = 0.0f;      // filtered PF for stability

static int8_t i_polarity = -1;
static uint8_t neg_count = 0;
static uint32_t last_cross_ms = 0;
static uint8_t  armed_rise = 1;
static uint32_t win_t0 = 0;

volatile uint16_t cap_v[256];
volatile uint16_t cap_i[256];
volatile uint16_t cap_idx = 0;
volatile uint8_t  cap_done = 0;
volatile float pf_now = 0.0f;
volatile float pf_i1  = 0.0f;
volatile float pf_v1  = 0.0f;


//DC ADD Variables....4/1
volatile uint16_t dbg_idc = 0;      // raw ADC code for PA3 (DC current)
volatile uint16_t dbg_vdc = 0;      // raw ADC code for PA4 (DC voltage)

volatile float idc_adc_volts = 0.0f; // sensor voltage seen at PA3
volatile float vdc_adc_volts = 0.0f; // sensor voltage seen at PA4

volatile float dc_current = 0.0f;    // final DC current in amps
volatile float dc_voltage = 0.0f;    // final DC voltage in volts

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void adc_read_four(ADC_HandleTypeDef *hadc,
                          uint16_t *ch0, uint16_t *ch1,
                          uint16_t *ch2, uint16_t *ch3)
{
    if (HAL_ADC_Start(hadc) != HAL_OK) {
        *ch0 = *ch1 = *ch2 = *ch3 = 0xEEEF;
        return;
    }

    if (HAL_ADC_PollForConversion(hadc, 100) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        *ch0 = *ch1 = *ch2 = *ch3 = 0xFFFF;
        return;
    }
    *ch0 = (uint16_t)HAL_ADC_GetValue(hadc);   // Rank 1

    if (HAL_ADC_PollForConversion(hadc, 100) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        *ch0 = *ch1 = *ch2 = *ch3 = 0xFFFF;
        return;
    }
    *ch1 = (uint16_t)HAL_ADC_GetValue(hadc);   // Rank 2

    if (HAL_ADC_PollForConversion(hadc, 100) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        *ch0 = *ch1 = *ch2 = *ch3 = 0xFFFF;
        return;
    }
    *ch2 = (uint16_t)HAL_ADC_GetValue(hadc);   // Rank 3

    if (HAL_ADC_PollForConversion(hadc, 100) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        *ch0 = *ch1 = *ch2 = *ch3 = 0xFFFF;
        return;
    }
    *ch3 = (uint16_t)HAL_ADC_GetValue(hadc);   // Rank 4

    HAL_ADC_Stop(hadc);
}

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  center = 2048;
  iac_center = 2048;
  warmup = 0;
  win_t0 = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      // ============================================================
      // 0) CONSTANTS / SCALE FACTORS
      // ============================================================
      // ADC reference (measured), and conversion from 12-bit counts -> volts at ADC pin
      const float VREF = 3.319f;
      const float COUNTS_TO_VOLTS = VREF / 4095.0f;

      // Voltage divider gain to map ADC_VAC node back to "AC_HOT"
      const float DIV_GAIN = (1000000.0f + 15000.0f) / 15000.0f;

      // Empirical correction factors for 60 Hz vs 120 Hz behavior (calibration)
      const float K60  = 1.36f;
      const float K120 = 1.68f;

      // Current chain parameters:
      // RB = burden resistor (ohms)
      // CT_RATIO = CST-1005 is ~1000:1
      // N_TURNS = passes through CT window
      // K_I = empirical current calibration factor
      const float RB = 100.0f;
      const float CT_RATIO = 1000.0f;
      const float N_TURNS = 8.0f;
      const float K_I = 1.13f;

      // DC Stuff 4/1
      // DC sensing constants
       const float DC_ADC_REF = 2.66f;
       const float DC_COUNTS_TO_VOLTS = DC_ADC_REF / 4095.0f;

     // ACS712 calibration
       const float ACS712_ZERO_VOLTAGE = 2.55f;   // measured output at 0 A
       const float ACS712_SENSITIVITY  = 0.185f;  // 5A version = 185 mV/A
       const float VOLTAGE_RATIO = (100000.0f + 5600.0f) / 5600.0f;



      // Zero-cross deadband (counts) to avoid noise-triggered crossings
      const int32_t ZC_DB = 30;

      // ============================================================
      // 1) "WINDOW" ACCUMULATORS (STATIC = PERSIST ACROSS LOOPS)
      //    We integrate for ~1 second, then compute mean/RMS/power/PF
      // ============================================================
      static uint32_t n = 0;       // number of samples inside current window

      // Sums for mean and variance (in ADC counts domain)
      // sum_v, sum_i -> compute mean (center)
      // sum_v2, sum_i2 -> compute variance -> RMS
      // sum_vi -> compute covariance -> real power (after scaling)
      static double sum_v  = 0.0;
      static double sum_i  = 0.0;
      static double sum_v2 = 0.0;
      static double sum_i2 = 0.0;
      static double sum_vi = 0.0;

      // Cross-correlation variants to compensate small channel timing skew:
      // sum_vi_i1 uses i[n-1] with v[n]
      // sum_vi_v1 uses v[n-1] with i[n]
      // One of these often gives “cleaner” power/PF on a breadboard/2-sample ADC read.
      static double sum_vi_i1 = 0.0;   // v[n]   * i[n-1]
      static double sum_vi_v1 = 0.0;   // v[n-1] * i[n]

      // Previous samples needed for the i[n-1] and v[n-1] terms
      static int32_t prev_v_counts = 0;
      static int32_t prev_i_counts = 0;
      static uint8_t have_prev = 0;

      // 60/120 selection state
      static uint8_t use_k120 = 0;

      // PF filter state
      static uint8_t pf_filt_init = 0;

      // ============================================================
      // 2) SAMPLE ADC CHANNELS (PA0 = voltage, PA1 = current)
      // ============================================================
      // DC Update Adds 4/1
      adc_read_four(&hadc1, &dbg_v, &dbg_iac, &dbg_idc, &dbg_vdc);

      // DC update 4/1
      idc_adc_volts = ((float)dbg_idc * DC_ADC_REF) / 4095.0f;
      vdc_adc_volts = dbg_vdc * DC_COUNTS_TO_VOLTS;
      // ACS712 current calculation
      dc_current = (idc_adc_volts - ACS712_ZERO_VOLTAGE) / ACS712_SENSITIVITY;


      dc_voltage = vdc_adc_volts * VOLTAGE_RATIO;

      // Track min/max of voltage raw ADC (debug / sanity check)
      if (dbg_v < vmin) vmin = dbg_v;
      if (dbg_v > vmax) vmax = dbg_v;

      // ============================================================
      // 3) FREQUENCY ESTIMATE (RISING ZERO-CROSS FROM VOLTAGE CHANNEL)
      //    Uses center (mean) as the “zero” reference + deadband.
      // ============================================================
      int32_t x_v = (int32_t)dbg_v - (int32_t)center; // voltage sample relative to center

      // Arm the rising-cross detector only after we are confidently negative
      if (x_v < -ZC_DB)
      {
          armed_rise = 1;
      }

      // Rising crossing: from negative region to positive region beyond deadband
      if (armed_rise && x_v > ZC_DB)
      {
          uint32_t now_ms = HAL_GetTick();

          if (last_cross_ms != 0)
          {
              uint32_t dt_ms = now_ms - last_cross_ms;

              // dt_ms ~ 16.7 ms for 60 Hz, ~ 8.3 ms for 120 Hz
              // You gate to a reasonable band to reject false crossings
              if (dt_ms >= 12 && dt_ms <= 25)
              {
                  float f_new = 1000.0f / (float)dt_ms;

                  // Low-pass filter frequency estimate for stability
                  if (f_est <= 0.0f)
                      f_est = f_new;
                  else
                      f_est = 0.80f * f_est + 0.20f * f_new;
              }
          }

          last_cross_ms = now_ms;
          armed_rise = 0; // disarm until we go negative again
      }

      // ============================================================
      // 4) ACCUMULATE WINDOW STATS IN "COUNTS DOMAIN"
      //    We compute mean, variance, covariance later.
      // ============================================================
      sum_v  += (double)dbg_v;
      sum_i  += (double)dbg_iac;

      sum_v2 += (double)dbg_v   * (double)dbg_v;
      sum_i2 += (double)dbg_iac * (double)dbg_iac;

      // Raw product sum (counts*counts); covariance will be derived later
      sum_vi += (double)dbg_v * (double)dbg_iac;

      // Lagged product sums (for skew compensation)
      if (have_prev)
      {
          sum_vi_i1 += (double)dbg_v        * (double)prev_i_counts; // v[n]*i[n-1]
          sum_vi_v1 += (double)prev_v_counts * (double)dbg_iac;      // v[n-1]*i[n]
      }

      // Update previous sample memory
      prev_v_counts = (int32_t)dbg_v;
      prev_i_counts = (int32_t)dbg_iac;
      have_prev = 1;

      n++; // one more sample in this window

      // ============================================================
      // 5) ONCE PER ~1 SECOND: FINALIZE WINDOW, COMPUTE OUTPUTS,
      //    THEN RESET ACCUMULATORS
      // ============================================================
      if ((HAL_GetTick() - win_t0) >= 1000)
      {
          if (n == 0) n = 1; // safety

          // ----------------------------
          // 5a) Compute means (centers)
          // ----------------------------
          double mean_v = sum_v / (double)n;
          double mean_i = sum_i / (double)n;

          // Center counts for each channel
          center     = (uint16_t)(mean_v + 0.5);
          iac_center = (uint16_t)(mean_i + 0.5);

          // ----------------------------
          // 5b) Variance -> RMS at ADC pins
          // ----------------------------
          // variance = E[x^2] - (E[x])^2  (counts^2)
          double var_v = (sum_v2 / (double)n) - (mean_v * mean_v);
          double var_i = (sum_i2 / (double)n) - (mean_i * mean_i);

          // Covariance (counts^2) -> proportional to real power
          // covariance = E[vi] - E[v]E[i]
          double cov_vi    = (sum_vi / (double)n)    - (mean_v * mean_i);
          double cov_vi_i1 = (sum_vi_i1 / (double)n) - (mean_v * mean_i);
          double cov_vi_v1 = (sum_vi_v1 / (double)n) - (mean_v * mean_i);

          // Numerical guard against tiny negative due to floating rounding
          if (var_v < 0.0) var_v = 0.0;
          if (var_i < 0.0) var_i = 0.0;

          // Convert variance(counts^2) -> Vrms at ADC pin:
          // sqrt(var_counts) gives counts_rms; multiply by counts->volts
          vrms_adc     = sqrtf((float)var_v) * COUNTS_TO_VOLTS;
          vrms_iac_adc = sqrtf((float)var_i) * COUNTS_TO_VOLTS;

          // ----------------------------
          // 5c) Decide which voltage calibration to use (60 vs 120)
          // ----------------------------
          // f_est is in Hz-ish; your thresholding is tuned to your estimator behavior
          if (!use_k120) { if (f_est > 95.0f) use_k120 = 1; }
          else           { if (f_est < 75.0f) use_k120 = 0; }

          k_used = use_k120 ? K120 : K60;

          // Scale Vrms from ADC pin back to actual AC_HOT
          vrms_hot = vrms_adc * DIV_GAIN * k_used;

          // ----------------------------
          // 5d) Convert current channel voltage -> loop current
          // ----------------------------
          // Vrms across burden -> secondary current:
          float isec = vrms_iac_adc / RB;

          // CT ratio and turns map secondary to primary:
          // Iprimary ≈ Isec * CT_RATIO / N_TURNS
          float iloop_raw = (isec * CT_RATIO) / N_TURNS;

          // Empirical correction
          irms_loop = iloop_raw * K_I;

          // These “scale” constants convert counts-domain covariance into real-world watts
          const float V_SCALE = DIV_GAIN * k_used;               // ADC volts -> HOT volts
          const float I_SCALE = (CT_RATIO / N_TURNS) / RB * K_I; // ADC volts -> LOOP amps

          // Apparent power
          s_apparent = vrms_hot * irms_loop;

          // Reset “instant” outputs before computing (keeps debugger clean)
          p_real = 0.0f;
          pf     = 0.0f;
          pf_now = 0.0f;
          pf_i1  = 0.0f;
          pf_v1  = 0.0f;

          // ----------------------------
          // 5e) Warmup: ignore first couple windows until centers settle
          // ----------------------------
          if (warmup < 2)
          {
              warmup++;
          }
          else
          {
              // ----------------------------
              // 5f) Real power from covariance
              // ----------------------------
              // cov_vi is in counts^2. Multiply by (counts->volts)^2 to get volts^2 at ADC pins,
              // then scale to HOT volts and LOOP amps, and apply i_polarity sign.
              float p_now = (float)cov_vi * COUNTS_TO_VOLTS * COUNTS_TO_VOLTS
                          * V_SCALE * I_SCALE * (float)i_polarity;

              float p_i1 = (float)cov_vi_i1 * COUNTS_TO_VOLTS * COUNTS_TO_VOLTS
                         * V_SCALE * I_SCALE * (float)i_polarity;

              float p_v1 = (float)cov_vi_v1 * COUNTS_TO_VOLTS * COUNTS_TO_VOLTS
                         * V_SCALE * I_SCALE * (float)i_polarity;

              // Gate if signals are too small (avoid dividing noise)
              if (vrms_adc >= 0.005f && vrms_iac_adc >= 0.005f && s_apparent > 1e-6f)
              {
                  // PF candidates for three alignment choices
                  pf_now = p_now / s_apparent;
                  pf_i1  = p_i1  / s_apparent;
                  pf_v1  = p_v1  / s_apparent;

                  // Clamp candidates to [-1,1]
                  if (pf_now > 1.0f)  pf_now = 1.0f;
                  if (pf_now < -1.0f) pf_now = -1.0f;

                  if (pf_i1 > 1.0f)   pf_i1 = 1.0f;
                  if (pf_i1 < -1.0f)  pf_i1 = -1.0f;

                  if (pf_v1 > 1.0f)   pf_v1 = 1.0f;
                  if (pf_v1 < -1.0f)  pf_v1 = -1.0f;

                  // Current choice: use i[n-1] alignment as the official output
                  // (often best when voltage and current samples are slightly time-skewed)
                  p_real = p_i1;
                  pf     = pf_i1;
              }
          }

          // ----------------------------
          // 5g) PF low-pass filter (smooth display)
          // ----------------------------
          // If conditions are bad, reset filter to 0 and require re-init next good window.
          if (warmup < 2 || s_apparent < 1e-6f || vrms_adc < 0.005f || vrms_iac_adc < 0.005f)
          {
              pf_filt = 0.0f;
              pf_filt_init = 0;
          }
          else
          {
              // First good window: snap filter to PF (no long “creep”)
              if (!pf_filt_init)
              {
                  pf_filt = pf;
                  pf_filt_init = 1;
              }
              else
              {
                  // Then exponential smoothing
                  pf_filt = 0.90f * pf_filt + 0.10f * pf;
              }
          }

          // ----------------------------
          // 5h) Reset window accumulators for next ~1 second window
          // ----------------------------
          n = 0;

          sum_v  = 0.0;
          sum_i  = 0.0;
          sum_v2 = 0.0;
          sum_i2 = 0.0;
          sum_vi = 0.0;
          sum_vi_i1 = 0.0;
          sum_vi_v1 = 0.0;

          have_prev = 0;
          prev_v_counts = 0;
          prev_i_counts = 0;

          vmin = 4095;
          vmax = 0;

          // restart 1-second timer
          win_t0 = HAL_GetTick();
      }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

 // DC Update 4/1
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
