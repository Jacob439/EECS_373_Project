/*
 * joystick.h
 *
 *  Created on: Apr 3, 2024
 *      Author: jbeels
 */


#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

	#include "stdint.h"
	#include "stm32l4xx_hal.h"
	#define JOYSTICK_ID  0,
	#define JOYSTICK_VERSION1 1
	#define JOYSTICK_VERSION2 2
	#define JOYSTICK_X_MSB 3
	#define JOYSTICK_X_LSB 4
	#define JOYSTICK_Y_MSB 5
	#define JOYSTICK_Y_LSB 6
	#define JOYSTICK_BUTTON 7
	#define JOYSTICK_STATUS 8 //1 - button clicked
	#define JOYSTICK_I2C_LOCK 9
	#define JOYSTICK_CHANGE_ADDRESS 10
	#define JOYSTICK_ADDRESS 0x40
	HAL_StatusTypeDef ret;
	uint8_t _deviceAddress;


	//Write a byte value to a spot in the Joystick
  int writeRegister(uint8_t reg, uint8_t val);

  //Change the I2C address of this address to newRegAddress
  int setI2CAddress(uint8_t newRegAddress);

  //Reads from a given location from the Joystick
  uint8_t readRegister(uint8_t reg);

  //Returns the 10-bit ADC value of the joystick horizontal position
  uint16_t getHorizontal();

  //Returns the 10-bit ADC value of the joystick vertical position
  uint16_t getVertical();

  //Returns 0 button is currently being pressed
  uint8_t getButton();

  //Returns 1 if button was pressed between reads of .getButton() or .checkButton()
  //the register is then cleared after read.
  //The joystick has a register where it saves if a button was pressed after the getbutton or checkbutton function was used
  uint8_t checkButton();

  // Returns the following
  // 1: Left
  // 2: Right
  // 3: Up
  // 4: Down
  // 0: Center
  // Range: 0 to 2044
  uint8_t threshold();

  void JOYSTICK_INIT(I2C_HandleTypeDef hi2c1;);
#endif /* INC_JOYSTICK_H_ */
