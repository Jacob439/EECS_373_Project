/*
 * keypad.h
 *
 *  Created on: Apr 3, 2024
 *      Author: jbeels
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#include "stdint.h"
#include "stdio.h"

// Checks every row while a single column is pulled down
int RowChecker();


uint8_t ASCII_Keypad_Lookup[4][4] = {{0x31, 0x32, 0x33, 0x41},
                                       {0x34, 0x35, 0x36, 0x42},
                                       {0x37, 0x38, 0x39, 0x43},
                                       {0x2A, 0x30, 0x23, 0x44}};
  const uint8_t max_digits = 5;
  uint8_t ASCII_Weight[5];
//  uint8_t weight_int;
  uint8_t ASCII_Age[5];
  uint8_t age;
  // Count of currently input characters

  // uint8_t ASCII_Height [5];
  // uint8_t heightCounter = 0; implement later

  uint8_t KeyPadReturn(int row, int col, int weightSel);

//  int KeyPadSelect();
  void running();
  void keypad_init();


#endif /* INC_KEYPAD_H_ */
