/*
 * TempHumSensor.c
 *
 *  Created on: Apr 3, 2024
 *      Author: masondev
 */
#include "main.h"
#include "TempHumSensor.h"

// i2c handler
static I2C_HandleTypeDef* hi2c;

void initTempHumSensor(I2C_HandleTypeDef* hi2c_in) {
	hi2c = hi2c_in;
}

// send command to sensor
void req_measurements() {
	uint8_t buf[2] = {CMD_MSB, CMD_LSB};
	HAL_I2C_Master_Transmit(hi2c, SAD_W, buf, 2, 1000);
}

// send read request to sensor
// sensor will pull clock down until measurements are done
TempHumRaw_t read_raw_measurements() {
	TempHumRaw_t data;
	uint8_t buf[4];
	HAL_I2C_Master_Receive(hi2c, SAD_R, buf, 4, 1000);
	data.temp = buf[0] << 8 | buf[1];
	data.hum = buf[2] << 8 | buf[3];
	return data;
}

// get converted values
TempHum_t get_temp_hum() {
	TempHumRaw_t raw_data;
	TempHum_t conv_data;
	req_measurements();
	raw_data = read_raw_measurements();
	int32_t casted_temp = (uint32_t)(raw_data.temp);
	int32_t casted_hum = (uint32_t)(raw_data.hum);
	conv_data.temp = -45.0f + 315.0f * ((float)casted_temp/65535.0f);
	conv_data.hum = 100.0f * ((float)casted_hum/65535.0f);
	return conv_data;
}
