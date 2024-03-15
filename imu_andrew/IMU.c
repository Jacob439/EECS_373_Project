/*
 * IMU.c
 *
 *  Created on: Mar 13, 2024
 *      Author: masondev
 */

#include "IMU.h"
#include "stdio.h"
#include "main.h"

#define SAD_W 0x50
#define SAD_R 0x51

uint8_t check_ret(HAL_StatusTypeDef ret) {
	if (ret == HAL_OK) {
		return 1;
	} else {
		// TODO:
		// add more descriptive error
		printf("i2c error!\n\r");
		return 0;
	}
}

uint8_t transmit_buf(I2C_HandleTypeDef *hi2c, uint8_t *buf, uint16_t bytes) {
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, SAD_W, buf, bytes, 1000);
	return check_ret(ret);
}

uint8_t read_to_buf(I2C_HandleTypeDef *hi2c, uint8_t subAddr, uint8_t *buf, uint16_t bytes){
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, SAD_W, &subAddr, 1, 1000);
	if (!check_ret(ret)) return 0;
	ret = HAL_I2C_Master_Receive(hi2c, SAD_R, buf, bytes, 1000);
	return check_ret(ret);
}

void startup_IMU(I2C_HandleTypeDef *hi2c){
	uint8_t buf[10];

	// verify i2c is working properly by reading chip id
	read_to_buf(hi2c, 0x00, buf, 1);
	if(buf[0] != 0xA0) {
		printf("chip error, wrong/no chip ID returned\n\r");
		return;
	}

	// put chip in configuration mode
	buf[0] = 0x3D;
	buf[1] = 0x00;
	transmit_buf(hi2c, buf, 2);
}

void init_IMU(I2C_HandleTypeDef *hi2c){
	uint8_t buf[10];

	startup_IMU(hi2c);

	// put chip in IMU mode
	buf[0] = 0x3D;
	buf[1] = 0x08;
	transmit_buf(hi2c, buf, 2);
}

void init_IMU_custom(I2C_HandleTypeDef *hi2c, uint8_t mode, uint8_t range, uint8_t unit){
	uint8_t buf[10];

	startup_IMU(hi2c);

	// put chip in desired mode
	buf[0] = 0x3D;
	buf[1] = mode;
	transmit_buf(hi2c, buf, 2);
	HAL_Delay(10);

	// set acceleration range
	read_to_buf(hi2c, 0x8, buf, 1);
	buf[0] &= ~(range | 0xFC);
	buf[0] |= range & 0x03;
	buf[1] = buf[0];
	buf[0] = 0x8;
	transmit_buf(hi2c, buf, 2);

	// set unit output
	read_to_buf(hi2c, 0x3B, buf, 1);
	buf[0] &= ~(unit | 0xFE);
	buf[0] |= unit & 0x1;
	buf[1] = buf[0];
	buf[0] = 0x3B;
	transmit_buf(hi2c, buf, 2);
}






void lin_acc_vec_raw(I2C_HandleTypeDef *hi2c, int16_t* vec) {
	uint8_t buf[6];
	read_to_buf(hi2c, 0x28, buf, 6);
	vec[0] = buf[0] | ((0xFF & buf[1]) << 8);
	vec[1] = buf[2] | ((0xFF & buf[3]) << 8);
	vec[2] = buf[4] | ((0xFF & buf[5]) << 8);
	return;
}

int16_t x_lin_acc_raw(I2C_HandleTypeDef *hi2c){
	uint8_t buf[2];
	read_to_buf(hi2c, 0x28, buf, 2);
	return buf[0] | ((0xFF & buf[1]) << 8);
}

int16_t y_lin_acc_raw(I2C_HandleTypeDef *hi2c){
	uint8_t buf[2];
	read_to_buf(hi2c, 0x2A, buf, 2);
	return buf[0] | ((0xFF & buf[1]) << 8);
}

int16_t z_lin_acc_raw(I2C_HandleTypeDef *hi2c){
	uint8_t buf[2];
	read_to_buf(hi2c, 0x2C, buf, 2);
	return buf[0] | ((0xFF & buf[1]) << 8);
}

void lin_acc_vec(I2C_HandleTypeDef *hi2c, double* vec){
	vec[0] = (double)x_lin_acc_raw(hi2c) /100;
	vec[1] = (double)y_lin_acc_raw(hi2c) /100;
	vec[2] = (double)z_lin_acc_raw(hi2c) /100;
	return;
}

double x_lin_acc(I2C_HandleTypeDef *hi2c){
	return (double)x_lin_acc_raw(hi2c) /100;
}

double y_lin_acc(I2C_HandleTypeDef *hi2c){
	return (double)y_lin_acc_raw(hi2c) /100;
}

double z_lin_acc(I2C_HandleTypeDef *hi2c){
	return (double)z_lin_acc_raw(hi2c) /100;
}


void grav_vec_raw(I2C_HandleTypeDef *hi2c, int16_t* vec) {
	uint8_t buf[6];
	read_to_buf(hi2c, 0x2E, buf, 6);
	vec[0] = buf[0] | ((0xFF & buf[1]) << 8);
	vec[1] = buf[2] | ((0xFF & buf[3]) << 8);
	vec[2] = buf[4] | ((0xFF & buf[5]) << 8);
	return;
}

int16_t x_grav_raw(I2C_HandleTypeDef *hi2c){
	uint8_t buf[2];
	read_to_buf(hi2c, 0x2E, buf, 2);

	// MSB LOSS HACK
	int16_t ret;
	ret = buf[0] | ((0xFF & buf[1]) << 8);
	return (ret > 1000) ? ret |= 0x8000 : ret;
}

int16_t y_grav_raw(I2C_HandleTypeDef *hi2c){
	uint8_t buf[2];
	read_to_buf(hi2c, 0x30, buf, 2);

	// MSB LOSS HACK
	int16_t ret;
	ret = buf[0] | ((0xFF & buf[1]) << 8);
	return (ret > 1000) ? ret |= 0x8000 : ret;
}

int16_t z_grav_raw(I2C_HandleTypeDef *hi2c){
	uint8_t buf[2];
	read_to_buf(hi2c, 0x32, buf, 2);

	// MSB LOSS HACK
	int16_t ret;
	ret = buf[0] | ((0xFF & buf[1]) << 8);
	return (ret > 1000) ? ret |= 0x8000 : ret;
}

void grav_vec(I2C_HandleTypeDef *hi2c, float* vec) {
	vec[0] = x_grav_raw(hi2c) / 100.0f;
	vec[1] = y_grav_raw(hi2c) / 100.0f;
	vec[2] = z_grav_raw(hi2c) / 100.0f;
	return;
}

float x_grav(I2C_HandleTypeDef *hi2c){
	return x_grav_raw(hi2c) / 100.0f;
}

float y_grav(I2C_HandleTypeDef *hi2c){
	return y_grav_raw(hi2c) / 100.0f;
}

float z_grav(I2C_HandleTypeDef *hi2c){
	return z_grav_raw(hi2c) / 100.0f;
}


