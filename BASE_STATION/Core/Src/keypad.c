/*
 * keypad.c
 *
 *  Created on: Apr 3, 2024
 *      Author: jbeels
 */


// Checks every row while a single column is pulled down
#include "keypad.h"
#include "stm32l4xx_hal.h"
#include "lcd.h"

// Checks every row while a single column is pulled down
int RowChecker() {
    int val = 0;
    val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET ? 1 : val;
    val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET ? 2 : val;
    val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET ? 3 : val;
    val = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET ? 4 : val;
    //	  	return row_return;
    return val;
  }

//Checks to see if 'D' was pressed
//	int KeyPadSelect(){
//		int val = 0;
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);
//		val = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET ? 4 : 0;
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);
//		if(val != 0){
//			return 1;
//		}
//		return 0;
//	}


void keypad_init(){
	// Setting all the pins to high impedence
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 1);
}


	//Processes the row (val) and col values to get the number associated with that row and col
	//weightSel is used to determine if Weight (1) or Age (0) is being input
  uint8_t KeyPadReturn(int row, int col, int weightSel) {
	  // Count of currently input characters
	static uint8_t weightCounter = 0;
    if (row == 0) {
      return 0;
    }
    HAL_Delay(10);
    // After delay, only exits while loop once key is released
    while (row == RowChecker()) {
    }
    HAL_Delay(10);
    uint8_t ASCII_Value = ASCII_Keypad_Lookup[row - 1][col - 1];
    // Check if '#' is pressed
    if (ASCII_Value == 0x23) {
    	// Go to main display
      if (weightCounter == 0) {
        // If there is no value for weight, just return
        return 0;
      }
      /* print the weight or age
      /printf("End ASCII value: ");
      for (uint8_t i = 0; i < weightCounter; i++) {
        // print the values here
        //printf("%x ", ASCII_Weight[i]);
      }
      printf("\n");*/
      if(weightSel){
          	ASCII_Weight[weightCounter] = '\0';
      }
      else{
    	  ASCII_Age[weightCounter] = '\0';
      }
      weightCounter = 0;
      // Wipe screen
      //LCD_Fill(50, 56, 50 + 26*3, 50+28, C_BLACK);
      return 1;
    } else if (ASCII_Value == 0x2A || weightCounter == max_digits - 2) {
    	// Wipe screen
    	if(weightSel){
    		LCD_Fill(105, 5, 170, 5+28, C_BLACK);
    	}
    	else{
    		LCD_Fill(80, 5, 170, 5+28, C_BLACK);
    	}
      // Reset if '*' is the input
      // Other if statement:
      // -2: there is a ++ at the end, and need a spot for #
      // Reset if max digits have been reached
      weightCounter = 0;
      return 0;
    }
    //printf("ASCII value: %x\n", ASCII_Value);


    if(weightSel){
    	ASCII_Weight[weightCounter] = ASCII_Value;
    	LCD_PutChar(105 + weightCounter*20, 5, ASCII_Value, DEFAULT_FONT, C_GREEN, C_BLACK);
    	UG_FontSetTransparency(1);
    }
    else{
    	ASCII_Age[weightCounter] = ASCII_Value;
    	LCD_PutChar(80 + weightCounter*20, 5, ASCII_Value, DEFAULT_FONT, C_GREEN, C_BLACK);
    	UG_FontSetTransparency(1);
    }

    weightCounter++;
    return 0;
  }



  void running(){
	  uint8_t finished = 0;
	  int val = 0;
	  //Gathers Weight data
	  LCD_PutStr(5, 5, "Weight: ", DEFAULT_FONT, C_GREEN, C_BLACK);
  while (!finished) {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);
      val = RowChecker();
      finished = KeyPadReturn(val, 4, 1);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);
      if (finished) break;

      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
      val = RowChecker();
      finished = KeyPadReturn(val, 3, 1);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
      if (finished) break;

      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
      val = RowChecker();
      finished = KeyPadReturn(val, 2, 1);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);
      if (finished) break;

      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 0);
      val = RowChecker();
      finished = KeyPadReturn(val, 1, 1);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 1);

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    }
  	  finished = 0;
  	  LCD_Fill(5, 5, 170, 5+28, C_BLACK);
  	  //Gathers Age data
  	  LCD_PutStr(5, 5, "Age: ", DEFAULT_FONT, C_GREEN, C_BLACK);
  	while (!finished) {
  	      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);
  	      val = RowChecker();
  	      finished = KeyPadReturn(val, 4, 0);
  	      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);
  	      if (finished) return;

  	      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
  	      val = RowChecker();
  	      finished = KeyPadReturn(val, 3, 0);
  	      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
  	      if (finished) return;

  	      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
  	      val = RowChecker();
  	      finished = KeyPadReturn(val, 2, 0);
  	      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);
  	      if (finished) return;

  	      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 0);
  	      val = RowChecker();
  	      finished = KeyPadReturn(val, 1, 0);
  	      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 1);
  	}
  }


