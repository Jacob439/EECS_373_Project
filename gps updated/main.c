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
#include "gps.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PI 3.14159265358979323846
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t a[10] = {0};
uint8_t buf[128] = {0};
int j = 0;
int done =0;
int go = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart == &huart1)
//	GPS_UART_CallBack();
	//printf("%c", a[0]);
	if (go){
		buf[j++] = a[0];
		if (a[0] == '\n'){
			buf[j++] = a[0];
			j = 0;
			done = 1;
			go = 0;
			//printf("\n\r");
			return;
		}

	}
	else if (a[0] == '$' && j == 0) {
		buf[j++] = a[0];
		go = 1;

	}

	//HAL_UART_Receive_IT(&huart1, a, 1);

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
	if(1){

	}
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
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart1, a, 1);
  //GPS_Init();
  //HAL_UART_Receive_IT(&huart1, a, 1);
  GPS_t GPS2;
  double p2p_dist = 0;
  double velocity = 0;
  double x_cord = 0;
  double y_cord = 0;
  uint8_t temp[7];
  while (1)
  {

//	  HAL_UART_Receive(&huart1, a, 1, 5000);
//
//	  printf("%c\n", a[0]);
	  if (done){
		  done = 0;
			for (int i = 0; i < 128; ++i){
			  //printf("%c", buf[i]);
			}
			printf("\n");
			for (int i = 0; i < 6; ++i){
				temp[i] = buf[i];
			}
			temp[6] = '\0';

			// If we have string we want, disable interrupt, do calc
			if (!strncmp((char*)temp, "$GPGGA", 6)){
				printf("IN HERE\n\r");
				char lat[9]; //latitude string
				char lat_ns; //latitude north south
				char lon[9]; //same as above bit longitude
				char lon_ew;
				char time[2];//only take the seconds

				for (int i = 11; i < 13; ++i){
					time[i-11] = buf[i];

				}
				for (int i = 18; i < 27; i++){
					lat[i-18] = buf[i];
					if (lat[i-18] == ',' || lat[i-18] == 'N'){
					  p2p_dist = 0;
					  velocity = 0;
					}
				}
				lat_ns = buf[28];
				for (int i = 30; i < 40; i++){
					lon[i-30] = buf[i];
					if (lon[i-30] == ',' || lon[i-30] == 'N'){
					  p2p_dist = 0;
					  velocity = 0;

					}
				}
				lon_ew = buf[41];
				for (int i = 0; i < 128; ++i){
					buf[i] = 0;
				}
				GPS2.time_sec = strtof(time, NULL);
				int d_time = abs(GPS2.time_sec-GPS2.prev_time_sec);
				//printf("bruh %f\n\r", (double)strtof(lat, NULL));
				//printf("bruh2 %f\n\r", (double)strtof(lon, NULL));
				GPS2.dec_latitude = GPS_nmea_to_dec(strtof(lat, NULL), lat_ns)*(PI/180);
				GPS2.dec_longitude = GPS_nmea_to_dec(strtof(lon, NULL), lon_ew)*(PI/180);

				x_cord = (double)(GPS2.dec_longitude - GPS2.dec_longitude_prev)*
				  (cos(((double)GPS2.dec_latitude+(double)GPS2.dec_latitude_prev)/2));

				y_cord = (double)(GPS2.dec_latitude - GPS2.dec_latitude_prev);

				//printf("dec1 %f",x_cord);
				//printf("dec2 %f",y_cord);
				p2p_dist = sqrt(x_cord*x_cord + y_cord*y_cord)*6371000;
				velocity = p2p_dist/ d_time;// m/s, timer is set for 5 sec
				if (d_time == 0){
					velocity = 0;
				}
				printf("p2p: %f\nmeter/sec: %f\n\r", p2p_dist,velocity);
				//printf("Minute Per Mile: %f\n\r", .08333/(p2p_dist/1609.3));
				GPS2.dec_latitude_prev = GPS2.dec_latitude;
				GPS2.dec_longitude_prev = GPS2.dec_longitude;
				GPS2.prev_time_sec = GPS2.time_sec;
				//HAL_NVIC_DisableIRQ(USART1_IRQn);

		  		}
			for (int i = 0; i < 128; ++i){
				buf[i] = 0;
			}

		  }
	  else {
		  HAL_UART_Receive_IT(&huart1, a, 1);
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}


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
