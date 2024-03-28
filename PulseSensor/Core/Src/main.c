/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_VOLT 3.2
#define BUF_LENGTH 10
#define SAMPLE_FREQ 50 		// 50 Hz sample frequency
#define SAMPLE_PERIOD 20	// 20 ms sample frequency

// for bool typedef
#define true  1
#define false 0

// other
#define DICROTIC_BUFFER IBI (IBI * 3) / 5
#define THRESH_DEFAULT MAX_VOLT / 2.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

// general control variables
typedef unsigned char bool;
static unsigned char new_signal = false;

static int BPM = 0;		// Beats Per Minute
static int IBI = 100;	// Inter-Beat Interval (ms)
static int rate[BUF_LENGTH];

static float signal;	// signal output by the pulse sensor

// variables used to determine BPM and IBI
/* initialize thresh, pulse and trough amplitudes to half range */
static float amp = MAX_VOLT / 10.0f;
static float thresh = MAX_VOLT / 2.0f;
static float peak_amp = MAX_VOLT / 2.0f;
static float trough_amp = MAX_VOLT / 2.0f;
static bool first_beat = true;		// first beat bool
static bool second_beat = false; 	// second beat bool
static bool pulse = false; 		// pulse recognized bool
static int samples_since_last_beat = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void read_ADC();
void get_pulse();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(new_signal) return;
	read_ADC();
	new_signal = true;
}

void read_ADC(void) {
	uint32_t val;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
	val = HAL_ADC_GetValue(&hadc1);
	signal = val * 3.3f / 4096.0f;
}

void get_pulse() {
	++samples_since_last_beat;
	int N = samples_since_last_beat * SAMPLE_PERIOD;
	if (signal < thresh && N > (IBI / 5) * 3) { // avoid dicrotic noise by waiting 3/5 of last IBI
	    if (signal < trough_amp) {                        // T is the trough
	      trough_amp = signal;                            // keep track of lowest point in pulse wave
	    }
	  }

	  if (signal > thresh && signal > peak_amp) {       // thresh condition helps avoid noise
		  peak_amp = signal;                              // P is the peak
	  }                                          // keep track of highest point in pulse wave

	  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
	  // signal surges up in value every time there is a pulse
	  if (N > 250) {                             // avoid high frequency noise
	    if ( (signal > thresh) && (pulse == false) && (N > ((IBI / 5) * 3)) ) {
	      pulse = true;                             // set the Pulse flag when we think there is a pulse
	      IBI = N;    // measure time between beats in mS
	      samples_since_last_beat = 0;

	      if (second_beat) {                      // if this is the second beat, if secondBeat == TRUE
	        second_beat = false;                    // clear secondBeat flag
	        for (int i = 0; i < BUF_LENGTH; i++) {       // seed the running total to get a realisitic BPM at startup
	          rate[i] = IBI;
	        }
	      }

	      if (first_beat) {                       // if it's the first time we found a beat, if firstBeat == TRUE
	    	first_beat = 0;                       // clear firstBeat flag
	    	second_beat = 1;                      // set the second beat flag
	        // IBI value is unreliable so discard it
	        return;
	      }


	      // keep a running total of the last 10 IBI values
	      int runningTotal = 0;                  // clear the runningTotal variable

	      for (int i = 0; i < BUF_LENGTH - 1; i++) {          // shift data in the rate array
	        rate[i] = rate[i + 1];                // and drop the oldest IBI value
	        runningTotal += rate[i];              // add up the 9 oldest IBI values
	      }

	      rate[BUF_LENGTH - 1] = IBI;                          // add the latest IBI to the rate array
	      runningTotal += rate[BUF_LENGTH - 1];                // add the latest IBI to runningTotal
	      runningTotal /= BUF_LENGTH;                     // average the last 10 IBI values
	      BPM = 60000 / runningTotal;             // how many beats can fit into a minute? that's BPM!
	      //fadeLevel = MAX_FADE_LEVEL;             // If we're fading, re-light that LED.
	    }
	  }

	  if (signal < thresh && pulse) {  // when the values are going down, the beat is over
	    pulse = false;                         // reset the Pulse flag so we can do it again
	    amp = peak_amp - trough_amp;                           // get amplitude of the pulse wave
	    thresh = amp / 2 + trough_amp;                  // set thresh at 50% of the amplitude
	    peak_amp = thresh;                            // reset these for next time
	    trough_amp = thresh;
	  }

	  if (N > 2500) {                          // if 2.5 seconds go by without a beat
	    thresh = THRESH_DEFAULT;                // set thresh default
	    peak_amp = THRESH_DEFAULT;                               // set P default
	    trough_amp = THRESH_DEFAULT;                               // set T default
	    samples_since_last_beat = 0;          // bring the lastBeatTime up to date
	    first_beat = true;                      // set these to avoid noise
	    second_beat = false;                    // when we get the heartbeat back
	    BPM = 0;
	    IBI = 600;                  // 600ms per beat = 100 Beats Per Minute (BPM)
	    pulse = false;
	    amp = 100;                  // beat amplitude 1/10 of input range.

	  }
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // start timer
  HAL_TIM_Base_Start_IT(&htim16);

  // wait for buffer to fill initially
  HAL_Delay(BUF_LENGTH * 20);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int bpm;
  float buf_avg;
  int num_peaks;
  while (1)
  {
    if (new_signal) {
    	get_pulse();
    	new_signal = false;
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 399;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 199;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
