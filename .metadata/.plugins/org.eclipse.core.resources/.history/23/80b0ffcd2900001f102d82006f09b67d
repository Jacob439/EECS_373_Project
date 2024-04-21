///*
// * speed.c
// *
// *  Created on: Apr 8, 2024
// *      Author: masondevries
// */
//
//#include <speed.h>
//#include "math.h"
//
//
///* private function prototypes */
//
//float get_vec_mag(speed_vec_t);
//
//
///* static variables */
//
//static float x_speed = 0;
//static float y_speed = 0;
//static float z_speed = 0;
//static speed_vec_t curr_speed;
//
//// treated as circular buffer
//static speed_vec_t speed_buf[SPEED_BUF_LENGTH];
//static int speed_buf_idx = 0;
//static int wrapped_around = 0;
//
//
///* public functions */
//
//void input_acc(vec_t grav, vec_t acc){
//	// update component magnitudes
//	x_speed += x_acc * SPEED_UPDATE_PERIOD;
//	y_speed += y_acc * SPEED_UPDATE_PERIOD;
//	y_speed += y_acc * SPEED_UPDATE_PERIOD;
//
//	// update speed buffer and current speed
//	curr_speed.x = x_speed;
//	curr_speed.y = y_speed;
//	curr_speed.z = z_speed;
//	speed_buf[speed_buf_idx] = curr_speed;
//
//	// update next buffer index
//	speed_buf_idx = (speed_buf_idx + 1) % SPEED_BUF_LENGTH;
//	if (!wrapped_around && speed_buf_idx == 0) {
//		wrapped_around = 1;
//	}
//}
//
//float get_inst_speed(void){
//	return get_vec_mag(curr_speed);
//}
//
//float get_simple_speed() {
//	speed_vec_t accumulator = 0;
//
//	// not enough data
//	if (!wrapped_around) return 0;
//
//	// average buffers data
//	for (int i = 0; i < SPEED_BUF_LENGTH; ++i) {
//		accumulator.x += speed_buf[i].x;
//		accumulator.y += speed_buf[i].y;
//		accumulator.z += speed_buf[i].z;
//	}
//	accumulator.x /= SPEED_BUF_LENGTH;
//	accumulator.y /= SPEED_BUF_LENGTH;
//	accumulator.z /= SPEED_BUF_LENGTH;
//
//	return get_vec_mag(accumulator);
//}
//
//void update_from_gps(float gps_speed){
//	float avg_spd = get_inst_speed();
//
//}
//
//
///* private functions */
//
//norm_vec_t get_norm_vec(vec_t vec) {
//	vec_t ret;
//	float squared_speed = vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
//	ret.mag = sqrtf(squared_speed);
//	ret.x = vec.x/ret.mag;
//	ret.y = vec.y/ret.mag;
//	ret.z = vec.z/ret.mag;
//	return ret;
//}
