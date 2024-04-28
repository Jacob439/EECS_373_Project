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
#include "stdio.h"
#include "math.h"
#include "kalman.h"
#include "gps.h"
#include "string.h"
#include "stdlib.h"
#include "PulseSensor.h"
#include "lora_sx1276.h"
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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PI 3.14159265358979323846
uint8_t a[10] = {0};
struct Kalman kx;
struct Kalman ky;
struct Kalman kz;
double inst_velx = 0;
double inst_vely = 0;
double inst_velz = 0;
double x, y, z;

int imu_timer = 0;
int gps_timer = 0;
int pulse_timer = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//gps_timer = 1;
//	if(a[0] == 10){
//		printf("\n");
//	}
//	else
//		printf("%c", a[0]);
	//HAL_UART_Receive_IT(&huart1, a, 1);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim == &htim7){
		gps_timer = 1;
//		int counter = 0;
//		while(1){
//
//			HAL_UART_Receive(&huart1, a, 1, 1);
//			if(a[0] == '$'){
//				printf("%c", a[0]);
//				while (a[0] != 10){
//					HAL_UART_Receive(&huart1, a, 1, 100);
//					printf("%c", a[0]);
//				}
//				counter++;
//			}
//			if(counter == 6){
//				printf("\n");
//				break;
//			}
//		}
	}
	else if (htim == &htim6){
		//do imu
//		x = (double)x_lin_acc_raw(&hi2c1)/20000;
//		y = (double)y_lin_acc_raw(&hi2c1)/100;
//		z = (double)z_lin_acc_raw(&hi2c1)/100;
		imu_timer = 1;
		pulse_timer = 1;

	}

	//HAL_UART_Receive_IT(&huart1, a, 1);
	//HAL_UART_Receive(&huart1, a, 1, 1000);
	//
	//	  printf("%c\n", a[0]);
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
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  init_IMU(&hi2c1);
  initPulseSensor(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);
  //HAL_UART_Receive_IT(&huart1, a, 1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	//double x, y, z, a, b, c;
	double vector; //acc vector
	double velocity;//vel vector
//	double inst_velx = 0;
//	double inst_vely = 0;
//	double inst_velz = 0;


	kx._current_estimate = 0;
	kx._last_estimate = 0;
	kx._kalman_gain = 0;
	ky._current_estimate = 0;
	ky._last_estimate = 0;
	ky._kalman_gain = 0;
	kz._current_estimate = 0;
	kz._last_estimate = 0;
	kz._kalman_gain = 0;

	kx._err_estimate = .1;
	kx._err_measure = .1;
	kx._q = .01;
	ky._err_estimate = .1;
	ky._err_measure = .1;
	ky._q = .01;
	kz._err_estimate = .1;
	kz._err_measure = .1;
	kz._q = .01;
//	SimpleKalmanFilter(.1, .1, 5, kx);
//	SimpleKalmanFilter(.1, .1, .1, ky);
//	SimpleKalmanFilter(.1, .1, .1, kz);

	//int counter = 0;
	double updatex = 0;
	double updatey = 0;
	double updatez = 0;
	uint8_t gps_buf[128];
	uint8_t gps_idx = 0;
	GPS_t GPS2;
	GPS2.nmea_latitude = 0;
	GPS2.nmea_longitude = 0;
	GPS2.speed_k = 0;
	GPS2.utc_time = 0;
	double p2p_dist = 0;
	double total_dist = 0;
	double x_cord = 0;
	double y_cord = 0;
	char temp[7];
	int fake_velocity = 0;
	int bpm = 0;
	int imu_ctr = 0;// 1000 means 5s and then reset vel;
	lora_sx1276 lora;

	uint8_t res = lora_init(&lora, &hspi1, GPIOB, GPIO_PIN_0, LORA_BASE_FREQUENCY_US+FREQ_OFFSET);
	if (res != LORA_OK) {
		printf("epic fail!");
		// Initialization failed
	 }
  while (1)
  {
	  ////////////////////////////////
	  //							//
	  // begin vibration simulation //
	  //							//
	  ////////////////////////////////
	  switch(fake_velocity){
	  	  case 0: {
			  TIM1->CCR1 = 0;//0
			  break;
		  }
	  	  case 1: {
	  		  TIM1->CCR1 = 16384;//25
	  		  break;
	  	  }
	  	  case 2: {
	  		  TIM1->CCR1 = 32768;//50
	  		  break;
	  	  }
	  	  case 3: {
	  		  TIM1->CCR1 = 49152;//75
	  		  break;
	  	  }
	  	  default: {
	  		  TIM1->CCR1 = 0;//off?
	  	  }
	  }
	  ////////////////////////////////
	  //							//
	  // end vibration simulation   //
	  //							//
	  ////////////////////////////////

	  //printf("x:%-10f y:%-10f z:%-10f\r\n", x, y, z);

	  ////////////////////////////////////////////
	  // begin gps data fetch       			//
	  //										//
	  //also does the velocity calc and stuff 	//
	  ////////////////////////////////////////////
	  if (gps_timer){
		  imu_ctr = 1;
		  int counter = 0;
		  int counter2 = 0;
		  while(1){
			  HAL_UART_Receive(&huart1, a, 1, 5000);
			  if(a[0] == '$'){
				  gps_buf[gps_idx++] = a[0];
				  //printf("%c", a[0]);
				  while (a[0] != 10){
					  HAL_UART_Receive(&huart1, a, 1, 5000);
					  gps_buf[gps_idx++] = a[0];
					  //printf("%c", a[0]);
				  }
				  for (int i = 0; i < 7; ++i){
					  temp[i] = gps_buf[i];
				  }
				  temp[6] = '\0';
				  for (int i = 0; i < 128; ++i){
					  //printf("%c", gps_buf[i]);////////
				  }


				  //printf("done\n");
				  counter++;
				  gps_idx = 0;


				  if (!strncmp((char*)temp, "$GPGGA", 6)){
					  char lat[9];
					  char lat_ns;
					  char lon[9];
					  char lon_ew;

					  for (int i = 18; i < 27; i++){
						  lat[i-18] = gps_buf[i];

					  }
					  lat_ns = gps_buf[28];
					  for (int i = 30; i < 40; i++){
						  lon[i-30] = gps_buf[i];
					  }
					  lon_ew = gps_buf[41];
					  for (int i = 0; i < 128; ++i){
						  gps_buf[i] = 0;
					  }
					  GPS2.dec_latitude = GPS_nmea_to_dec(strtof(lat, NULL), lat_ns)*(PI/180);
					  GPS2.dec_longitude = GPS_nmea_to_dec(strtof(lon, NULL), lon_ew)*(PI/180);

				  }
				  else if (!strncmp((char*)temp, "$GPRMC", 6)){
					//sscanf((char*)gps_buf, "$GPRMC,%f,%f,%c,%f,%c,%f,%f,%d", &GPS2.utc_time, &GPS2.nmea_latitude, &GPS2.ns, &GPS2.nmea_longitude, &GPS2.ew, &GPS2.speed_k, &GPS2.course_d, &GPS2.date);
					  GPS2.speed_k++;
				  }
				  for (int i = 0; i < 128; ++i){
					  gps_buf[i] = 0;
				  }
			  }

			  if(counter == 6){


//				  printf("\n");
//				  printf("long: %f, longdec: %f\n\r", GPS2.nmea_latitude, GPS2.dec_longitude);
//				  printf("lat: %f, latdec: %f\n\r", GPS2.nmea_longitude, GPS2.dec_latitude);
//				  printf("speed: %f\n\r", GPS2.speed_k);
//				  printf("date: %f\n\r", GPS2.utc_time);

				  if(counter2 == 1){
					  gps_timer = 0;
					  x_cord = (double)(GPS2.dec_longitude - GPS2.dec_longitude_prev)*
							  (cos(((double)GPS2.dec_longitude+(double)GPS2.dec_longitude_prev)/2));
					  y_cord = (double)(GPS2.dec_latitude - GPS2.dec_latitude_prev);
					  p2p_dist = sqrt(x_cord*x_cord + y_cord*y_cord)*6371000;
					  velocity = p2p_dist/5;// m/s, timer is set for 5 sec
					  //printf("p2p: %f\nmeter/sec: %f\n\r", p2p_dist,velocity);
					  //printf("Minute Per Mile: %f\n\r", .08333/(p2p_dist/1609.3));
					  GPS2.dec_latitude_prev = GPS2.dec_latitude;
					  GPS2.dec_longitude_prev = GPS2.dec_longitude;

					  fake_velocity = (fake_velocity+1)%4;
					  break;
				  }
				  counter2++;
				  counter = 0;


			  }
		  }


	  }
	  ////////////////////////////////
	  //							//
	  // end gps data fetch         //
	  //							//
	  ////////////////////////////////

	  ////////////////////////////////
	  //							//
	  // begin imu stuff	        //
	  //							//
	  ////////////////////////////////
	  if (imu_timer){
		  x = (double)x_lin_acc_raw(&hi2c1)/100;
		  y = (double)y_lin_acc_raw(&hi2c1)/100;
		  z = (double)z_lin_acc_raw(&hi2c1)/100;
		  //20000 is 100 from imu, plus 200 for the time frame
		  //printf("x: %f y: %f, z: %f\n\r", x, y, z);
		  if((x>-.1)&&(x<.1)|(x>50) | (x<-50)){
			  x = 0;
		  }
		  if((y>-.1)&&(y<.1)|(y>50) | (y<-50)){
			  y = 0;
		  }
		  if((z>-.45)&&(z<.45)|(z>50) | (z<-50)){
			  z = 0;
		  }


	//	  updatex += x;
	//	  updatey += y;
	//	  updatez += z;
	//	  if (counter == 100){
	//		  updatex = 0;
	//		  updatey = 0;
	//		  updatez = 0;
	//		  counter = 0;
	//	  }
//		  updatex = updateEstimate(x, kx);//x;
//		  updatey = updateEstimate(y, ky);
//		  updatez = updateEstimate(z, kz);
//
		  updatex = inst_velx;
		  updatey = inst_vely;
		  updatez = inst_velz;
		  inst_velx += x;
		  inst_vely += y;
		  inst_velz += z;

//		  inst_velx = updateEstimate(inst_velx, &kx);
//		  inst_vely = updateEstimate(inst_vely, &ky);
//		  inst_velz = updateEstimate(inst_velz, &kz);
//		  imu_timer = 0;
		  //printf("x_velocity:%-10f y_velocity:%-10f z_velocity:%-10f\r\n", inst_velx, inst_vely, inst_velz);

		  if(imu_ctr == 1){
			  imu_ctr = 0;
			  inst_velx = 0;
			  inst_vely = 0;
			  inst_velz = 0;
		  }
	  }
	  ////////////////////////////////
	  //							//
	  // end imu stuff	        	//
	  //							//
	  ////////////////////////////////

	  ////////////////////////////////
	  //							//
	  // begin pulse stuff	        //
	  //							//
	  ////////////////////////////////

	  if(pulse_timer){//goes off every imu timer (tim6)
		  updatePulseSensor();
		  bpm = get_BPM();
		  //printf("BPM: %d\n\r", bpm);
		  pulse_timer = 0;
	  }

	  ////////////////////////////////
	  //							//
	  // end pulse stuff	        //
	  //							//
	  ////////////////////////////////




	  //printf("x_velocity:%-10f y_velocity:%-10f z_velocity:%-10f\r\n", inst_velx, inst_vely, inst_velz);


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
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 39;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 399;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 50000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
