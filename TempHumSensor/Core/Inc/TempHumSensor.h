/*
 * TempHumSensor.h
 *
 *  Created on: Apr 3, 2024
 *      Author: masondev
 */

#include "main.h"

/* MACROS */
#ifndef INC_TEMPHUMSENSOR_H_
#define INC_TEMPHUMSENSOR_H_

// device addr: 0x44
#define SAD_W 0x88
#define SAD_R 0x89

/* COMMANDS */

// clock stretch:
//	- enable(0x2C)
//	- disable(0x24)
#define CMD_MSB 0x2C

// repeatability:
//	- low (0x10)
// 	- med (0x0D)
//	- high(0x06)
#define CMD_LSB 0x0D

/* local variables and structs */

typedef struct {
	uint16_t temp;
	uint16_t hum;
} TempHumRaw_t;

typedef struct {
	float temp;
	float hum;
} TempHum_t;

void initTempHumSensor(I2C_HandleTypeDef*);

// send command to sensor
void req_measurements();

// send read request to sensor
// sensor will pull clock down until measurements are done
TempHumRaw_t read_raw_measurements();

// get converted values
// all you need to call in main to get measurements
TempHum_t get_temp_hum();


#endif /* INC_TEMPHUMSENSOR_H_ */
