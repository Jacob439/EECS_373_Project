/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
// #include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

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

// UART_HandleTypeDef huart2;
// UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

//35.15
typedef struct __attribute__((__packed__)) {
    uint32_t iTOW;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t speed;
    uint32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t cAcc;
    uint8_t CK_A;
    uint8_t CK_B;
} GPS_VELOCITY_PACKET;

//35.13
typedef struct __attribute__((__packed__)) {
    uint32_t iTOW;
    uint32_t tAcc;
    int32_t nano;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint8_t CK_A;
    uint8_t CK_B;
} GPS_TIME_PACKET;

//35.7
typedef struct __attribute__((__packed__)) {
    uint32_t iTOW;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    uint8_t CK_A;
    uint8_t CK_B;
} GEO_POS_SOL_PACKET;

//35.10
typedef struct __attribute__((__packed__)) {
    uint32_t iTOW;
    uint8_t gpsFix;
    //below three bit values
    uint8_t flags;
    uint8_t fixStat;
    uint8_t flags2;
    uint32_t ttff;
    uint32_t msss;
    uint8_t CK_A;
    uint8_t CK_B;
} NAV_STATUS_PACKET;

typedef enum{
	GOTBYTE,
	GOTSIZE,
	GOTPAYLOAD
} serial_read_state;

typedef struct __attribute__((__packed__)) {
	uint8_t cls;
	uint8_t id;
	uint16_t len;
} BEGIN_PACKET;

GPS_VELOCITY_PACKET gps_velocity_packet;
GPS_TIME_PACKET gps_time_packet;
GEO_POS_SOL_PACKET geo_pos_sol_packet;
NAV_STATUS_PACKET nav_status_packet;
BEGIN_PACKET begin_packet;

// Below two lines shouldn't? be included in the class
uint16_t gps_speed;
uint16_t gps_lock;

// This should be included
serial_read_state gps_state = GOTBYTE;
uint16_t gps_code = 0;
uint8_t gps_code_byte = 0;
uint16_t counter_thing = 0;

//Added by Jacob
uint16_t gps_time = 0;//Need one for 35.7 and 35.10
unsigned char gps_fix = 0;
int32_t gps_sea_level = 0;
uint32_t msss = 0;// For testing stat
// More debugging info
uint16_t selection_wrong_debug = 0;


// 0 = debugs match
// 1 = no match

typedef struct __attribute__((__packed__)) {
  // beginpacket.id doesn't match any that are givens
  unsigned int bit1:1;
  // UNUSED
  unsigned int bit2:1;
  // UNUSED
  unsigned int bit3:1;
  // UNUSED
  unsigned int bit4:1;
  // Status
  unsigned int bit5:1;
  // Position
  unsigned int bit6:1;
  // Time
  unsigned int bit7:1;
  // Velocity
  unsigned int bit8:1;
} PACKED_BITS;

PACKED_BITS chk_debug;

typedef struct __attribute__((__packed__)) {
    uint16_t p_vel;
    uint16_t p_time;
    uint16_t p_pos;
    uint16_t p_stat;
    uint16_t f_vel;
	uint16_t f_time;
	uint16_t f_pos;
	uint16_t f_stat;
} PACKET_DEBUG;

PACKET_DEBUG packet_debug;



#define ARRAY_SIZE 11

const uint8_t num_commands = ARRAY_SIZE;

/**
 * Notes from what David said regarding below
 *
 * First 6 of these commands seem to be 31.9 CFG-MSG
 * 		They seem to be turning off all the default messages
 * Remaning commands have inline comments
 *
 *
 * Notes about update frequency:
 * I find it weird that the higher the frequency, the more evenly the values update
 * I would have expected an inverse relationship
 * Starting at 10hz, I moved to 20hz, and then 40 hz
 * At 10 hz, I would rarely recieve time data
 * at 20hz I would inconsistantly get time data
 * at 40hz I consistantly get all the data I need
 *
 */
const uint8_t gps_config_commands[ARRAY_SIZE][16] = {
    { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24 },
    { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B },
    { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 },
    { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 },
    { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40 },
    { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xD5 },
    { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x12, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x27, 0x3D }, //This enables velocity
    { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x16, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, 0x00, 0x00 }, // CFG-RATE measurement rate to 100ms NOW 50ms (0x32)
		{ 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x21, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x36, 0xA6 }, //This enables time
		{ 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x17, 0xCD }, //This enables lat/long
		{ 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x18, 0xD4 }, //This enables sat stat
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_USART2_UART_Init(void);
// static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  // HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  // SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  // MX_GPIO_Init();
  // MX_USART2_UART_Init();
  // MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  // Receive GPS data
    HAL_UART_Receive_IT(&huart2, (uint8_t *) &gps_code_byte, 1);
    uint32_t gps_startup = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t loop_timer = 0;
  uint8_t test = 0;
  //Probably would be best to find a way to do this async
  while (1)
  {
	  if (!test){
		  printf("GOT HERE ONCE\r\n");
		  test++;
	  }
	  if (gps_startup < num_commands )
	        {
	            HAL_UART_Transmit(&huart2, gps_config_commands[gps_startup], 16, 100);//500 was 1000
	            gps_startup++;
	        }
	  if (gps_startup == num_commands){
		  printf("COMPLETED SENDING AT TIME: %ld\r\n", HAL_GetTick());
		  gps_startup++;
	  }
	  // This might have been the problem. Giving the GPS 7 seconds to send the commands now
//	  if (HAL_GetTick() > 6000) {
	  if (gps_startup > num_commands) {
			while (HAL_GetTick() - loop_timer < 1000); //1000, was 500
//	        while (HAL_GetTick() - loop_timer < 500);
	        loop_timer = HAL_GetTick();
	        printf("Sel: %d debug: %d %d %d %d %d %d %d %d Vel, time, sea, fix: %d %d %d %d\r\n", selection_wrong_debug, packet_debug.p_vel, packet_debug.p_time,
           packet_debug.p_pos, packet_debug.p_stat, packet_debug.f_vel, packet_debug.f_time, packet_debug.f_pos, packet_debug.f_stat, gps_speed, gps_time, gps_sea_level, msss);
	  }else{
		  while (HAL_GetTick() - loop_timer < 200);//was 500, then 300
		  loop_timer = HAL_GetTick();
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



/* USER CODE BEGIN 4 */
void checksum(uint8_t selection) {
	uint8_t GPS_A = 0, GPS_B = 0;
//	struct GPS_VELOCITY_PACKET *payload_packet;
	struct GEO_POS_SOL_PACKET *payload_packet;// Both velocity and time result in just velocity passing checksums
	uint16_t *begin_struct = &begin_packet;
	unsigned char *ptr_chk_debug = &chk_debug;
	uint16_t *second_debug_ptr = &packet_debug;

	//
	unsigned char chk_bit = 1;
	for (int i = 0; i < 7 - selection; i++) {
		chk_bit *= 2;
	}
//	unsigned char chk_bit1, chk_bit2;

	switch (selection) {
	case 0:
		GPS_A = gps_velocity_packet.CK_A;
		GPS_B = gps_velocity_packet.CK_B;
		payload_packet = &gps_velocity_packet;
//		chk_bit1 = 0b00000010;
//		chk_bit1 = 0b00000001;
		break;
	case 1:
		GPS_A = gps_time_packet.CK_A;
		GPS_B = gps_time_packet.CK_B;
		payload_packet = &gps_time_packet;
//		printf("Got time though");
//		chk_bit1 = 0b00001000;
//		chk_bit1 = 0b00000100;
		break;
	case 2:
		GPS_A = geo_pos_sol_packet.CK_A;
		GPS_B = geo_pos_sol_packet.CK_B;
		payload_packet = &geo_pos_sol_packet;
//		chk_bit1 = 0b00000010;
//		chk_bit1 = 0b00000001;
		break;
	case 3:
		GPS_A = nav_status_packet.CK_A;
		GPS_B = nav_status_packet.CK_B;
		payload_packet = &nav_status_packet;
//		chk_bit1 = 0b00000010;
//		chk_bit1 = 0b00000001;
		break;
	default:
		chk_debug.bit1 = 1;
		selection_wrong_debug++;
		return;
	}
	chk_debug.bit1 = 0;
	uint8_t CK_A = 0, CK_B = 0;

	int i;
	for(i = 0; i < 4; i++) {
		CK_A += ((uint8_t*)begin_struct)[i];
		CK_B += CK_A;
	}
	for(i = 0; i < begin_packet.len; i++) {
		CK_A += ((uint8_t*)payload_packet)[i];
		CK_B += CK_A;
	}

	if (CK_A != GPS_A || CK_B != GPS_B) {
		*ptr_chk_debug |=chk_bit;
//		*(second_debug_ptr + (selection + 4) * 2) = *(second_debug_ptr + (selection + 4) * 2) + 1;
		*(second_debug_ptr + (selection + 4)) = *(second_debug_ptr + (selection + 4)) + 1;

	} else {
		*ptr_chk_debug =chk_bit & ~ *ptr_chk_debug;
//		*(second_debug_ptr + selection * 2) = *(second_debug_ptr + selection * 2) + 1;
		*(second_debug_ptr + selection) = *(second_debug_ptr + selection) + 1;

	}

}


//Using the UBX Protocol
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(gps_state == GOTBYTE) {
        gps_code = gps_code >> 8;
        gps_code |= (uint16_t)(gps_code_byte) << 8;

        if(gps_code == 0x62B5) {
        	//Receiving the first packets to determine payload id and size
            HAL_UART_Receive_IT(&huart2, (uint8_t *)&begin_packet, sizeof(begin_packet));
            gps_state = GOTSIZE;
        } else{
            HAL_UART_Receive_IT(&huart2, (uint8_t *)&gps_code_byte, 1);
            gps_state = GOTBYTE;
        }
    } else if (gps_state == GOTSIZE) {
    	//Receiving the payload using the size of the payload given by the begin_packet
    	gps_state = GOTPAYLOAD;
    	switch(begin_packet.id) {
		case 0x12 :
			HAL_UART_Receive_IT(&huart2, (uint8_t *)&gps_velocity_packet, begin_packet.len + 2);
			break;
		case 0x02 :
			HAL_UART_Receive_IT(&huart2, (uint8_t *)&geo_pos_sol_packet, begin_packet.len + 2);
			break;
		case 0x03 :
			HAL_UART_Receive_IT(&huart2, (uint8_t *)&nav_status_packet, begin_packet.len + 2);
			break;
		case 0x21 :
			HAL_UART_Receive_IT(&huart2, (uint8_t *)&gps_time_packet, begin_packet.len + 2);
			break;
		default :
			gps_state = GOTBYTE;
			HAL_UART_Receive_IT(&huart2, (uint8_t *) &gps_code_byte, 1);
		}
    }
    else{
    	gps_lock = 1;
    	counter_thing++;
    	uint8_t selection = 0;
    	switch(begin_packet.id) {
		case 0x12 :
			gps_speed = gps_velocity_packet.gSpeed;
			break;
		case 0x02 :
			gps_sea_level = geo_pos_sol_packet.hMSL;
			selection = 2;
			break;
		case 0x03 :
			gps_fix = nav_status_packet.fixStat;
			msss = nav_status_packet.msss;
			selection = 3;
			break;
		case 0x21 :
			gps_time = gps_time_packet.year;
			selection = 1;
			break;
		default :
			gps_lock = 0;
			counter_thing--;
			selection = 4;
		}
    	checksum(selection);

        HAL_UART_Receive_IT(&huart2, (uint8_t *)&gps_code_byte, 1);
        gps_state = GOTBYTE;
    }
}



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  *
  * NOTE FROM JACOB: DO I WANT TO REMOVE THE BELOW CODE? OR SHOULD I CHANGE Error_Handler?
  */
// void Error_Handler(void)
// {
//   /* USER CODE BEGIN Error_Handler_Debug */
//   /* User can add his own implementation to report the HAL error return state */
//   __disable_irq();
//   while (1)
//   {
//   }
//   /* USER CODE END Error_Handler_Debug */
// }

// #ifdef  USE_FULL_ASSERT
// /**
//   * @brief  Reports the name of the source file and the source line number
//   *         where the assert_param error has occurred.
//   * @param  file: pointer to the source file name
//   * @param  line: assert_param error line source number
//   * @retval None
//   */
// void assert_failed(uint8_t *file, uint32_t line)
// {
//   /* USER CODE BEGIN 6 */
//   /* User can add his own implementation to report the file name and line number,
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//   /* USER CODE END 6 */
// }
// #endif /* USE_FULL_ASSERT */

//Mode cut code below
/**
  * @brief System Clock Configuration
  * @retval None
  */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Configure the main internal regulator output voltage
//   */
//   __HAL_RCC_PWR_CLK_ENABLE();
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// /**
//   * @brief USART2 Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_USART2_UART_Init(void)
// {

//   /* USER CODE BEGIN USART2_Init 0 */

//   /* USER CODE END USART2_Init 0 */

//   /* USER CODE BEGIN USART2_Init 1 */

//   /* USER CODE END USART2_Init 1 */
//   huart2.Instance = USART2;
//   huart2.Init.BaudRate = 9600;
//   huart2.Init.WordLength = UART_WORDLENGTH_8B;
//   huart2.Init.StopBits = UART_STOPBITS_1;
//   huart2.Init.Parity = UART_PARITY_NONE;
//   huart2.Init.Mode = UART_MODE_TX_RX;
//   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//   huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//   huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//   if (HAL_UART_Init(&huart2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN USART2_Init 2 */

//   /* USER CODE END USART2_Init 2 */

// }

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
// static void MX_USART6_UART_Init(void)
// {

//   /* USER CODE BEGIN USART6_Init 0 */

//   /* USER CODE END USART6_Init 0 */

//   /* USER CODE BEGIN USART6_Init 1 */

//   /* USER CODE END USART6_Init 1 */
//   huart6.Instance = USART6;
//   huart6.Init.BaudRate = 9600;
//   huart6.Init.WordLength = UART_WORDLENGTH_8B;
//   huart6.Init.StopBits = UART_STOPBITS_1;
//   huart6.Init.Parity = UART_PARITY_NONE;
//   huart6.Init.Mode = UART_MODE_TX_RX;
//   huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//   huart6.Init.OverSampling = UART_OVERSAMPLING_16;
//   huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//   huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//   if (HAL_UART_Init(&huart6) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN USART6_Init 2 */

//   /* USER CODE END USART6_Init 2 */

// }

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
// static void MX_GPIO_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_GPIOC_CLK_ENABLE();
//   __HAL_RCC_GPIOH_CLK_ENABLE();
//   __HAL_RCC_GPIOA_CLK_ENABLE();
//   __HAL_RCC_GPIOB_CLK_ENABLE();
//   __HAL_RCC_GPIOD_CLK_ENABLE();
//   __HAL_RCC_GPIOG_CLK_ENABLE();

//   /*Configure GPIO pin Output Level */
//   HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

//   /*Configure GPIO pin Output Level */
//   HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

//   /*Configure GPIO pin : USER_Btn_Pin */
//   GPIO_InitStruct.Pin = USER_Btn_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

//   /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
//   GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
//   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//   /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
//   GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
//   GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//   /*Configure GPIO pin : RMII_TXD1_Pin */
//   GPIO_InitStruct.Pin = RMII_TXD1_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
//   HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

//   /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
//   GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

//   /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
//   GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

//   /*Configure GPIO pin : USB_OverCurrent_Pin */
//   GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

//   /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
//   GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /*Configure GPIO pin : USB_VBUS_Pin */
//   GPIO_InitStruct.Pin = USB_VBUS_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

//   /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
//   GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
//   HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

// }