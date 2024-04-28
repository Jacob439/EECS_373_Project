///*
// * speed.h
// *
// *  Created on: Apr 8, 2024
// *      Author: masondevries
// */
//
///** NOTES
// *    - may want to speed up input_acc() for use in
// *   	interrupt callback
// *   	  - if so, put all complicated work in get_inst_speed
// *   	  	and get_simple_speed() function, using component
// *   	  	buffers to hold data
// */
//
//#ifndef INC_SPEED_H_
//#define INC_SPEED_H_
//
//#define SPEED_UPDATE_FREQ 200		// Hertz
//#define SPEED_UPDATE_PERIOD 0.005f	// seconds
//#define SPEED_BUF_LENGTH 1000	// 5 seconds of data, to match GPS frequency
//
//typedef struct {
//	float x;
//	float y;
//	float z;
//} vec_t;
//
//typedef struct {
//	float mag;
//	float x;
//	float y;
//	float z;
//} norm_vec_t;
//
//// simply does a reimann sum of acceleration
//void input_acc(vec_t, vec_t);
//
//// returns inst speed static variable
//// data not useful until speed buffer is full (one second)
//float get_inst_speed(void);
//
//// checks and balances homie
//// mitigates error by matching accelerometer speed
//// with GPS speed
//float update_from_gps(float);
//
//
//#endif /* INC_SPEED_H_ */
