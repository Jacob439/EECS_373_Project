/*
 * joystick.c
 *
 *  Created on: Apr 3, 2024
 *      Author: jbeels
 */

#include "joystick.h"

	//Write a byte value to a spot in the Joystick
	I2C_HandleTypeDef Hi2c1;
	void JOYSTICK_INIT(I2C_HandleTypeDef hi2c1){
		Hi2c1 = hi2c1;
	}
  int writeRegister(uint8_t reg, uint8_t val)
  {
	  uint8_t buf[10] = {reg, val};
	  ret = HAL_I2C_Master_Transmit(&Hi2c1, JOYSTICK_ADDRESS, &buf[0], 2, 1000);


    if (ret != 0)
    {
      //Serial.println("No write ack!");
      return (0); //Device failed to ack
    }

    HAL_Delay(30); // allow EPROM time to store value

    return (1);
  }

  //Change the I2C address of this address to newRegAddress
  int setI2CAddress(uint8_t newRegAddress)
  {
      writeRegister(JOYSTICK_I2C_LOCK, 0x13);
      writeRegister(JOYSTICK_CHANGE_ADDRESS, newRegAddress);

      // #if !defined(ESP_PLATFORM) && !defined(ESP8266)
      //   _i2cPort->end();
      // #endif

      //Once the address is changed, we need to change it in the library
      _deviceAddress = newRegAddress;
      return(1);
  }

  //Reads from a given location from the Joystick
  uint8_t readRegister(uint8_t reg)
  {
	  uint8_t buf[10] = {reg};
	  uint8_t buf0[10] = {};
	  ret = HAL_I2C_Master_Transmit(&Hi2c1, JOYSTICK_ADDRESS, &buf[0], 1, 1000);
	  ret = HAL_I2C_Master_Receive(&Hi2c1, JOYSTICK_ADDRESS, &buf0[0], 1, 1000);

    if (ret != 0)
    {
      //Serial.println("No ack!");
      return (0); //Device failed to ack
    }
    return buf0[0];
  }
  //Returns the 10-bit ADC value of the joystick horizontal position
  uint16_t getHorizontal()
  {
    uint16_t X_MSB = readRegister(JOYSTICK_X_MSB);
    uint16_t X_LSB = readRegister(JOYSTICK_X_LSB);
    return ((X_MSB<<8) | X_LSB)>>6; //MSB has the 8 MSB bits and LSB only has 2 bits
  }
  //Returns the 10-bit ADC value of the joystick vertical position
  uint16_t getVertical()
  {
    uint16_t Y_MSB = readRegister(JOYSTICK_Y_MSB);
    uint16_t Y_LSB = readRegister(JOYSTICK_Y_LSB);
    return ((Y_MSB<<8) | Y_LSB)>>6; //MSB has the 8 MSB bits and LSB only has 2 bits
  }
  //Returns 0 button is currently being pressed
  uint8_t getButton()
  {
	  uint8_t button = readRegister(JOYSTICK_BUTTON);
    //boolean pressed = status & (1<<statusButtonPressedBit);

    return(button);
  }

  //Returns 1 if button was pressed between reads of .getButton() or .checkButton()
  //the register is then cleared after read.
  //The joystick has a register where it saves if a button was pressed after the getbutton or checkbutton function was used
  uint8_t checkButton()
  {
	  uint8_t status = readRegister(JOYSTICK_STATUS);

    writeRegister(JOYSTICK_STATUS, 0x00); //We've read this status bit, now clear it

    return(status);
  }

  // Returns the following
  // 1: Left
  // 2: Right
  // 3: Up
  // 4: Down
  // 0: Center
  // Range: 0 to 2044
  uint8_t threshold(){
	  uint16_t horizontal = getHorizontal();
	  uint16_t vertical = getVertical();

	  // Perspective: Pin connections closest to you
	  // (0,0) is upper left, (1023, 1023) is lower right
	  // (0, 514) is left
	  // (512, 1023) is down
	  uint8_t lr = vertical > 250 && vertical < 750 ? 1 : 0;
	  if (horizontal < 250 && lr) {
		  return 1;
	  } else if (horizontal > 750 && lr){
		  return 2;
	  }
	  return 0;
  }
