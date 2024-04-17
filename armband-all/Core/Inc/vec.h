/*
 * vec.h
 *
 *  Created on: Apr 9, 2024
 *      Author: masondevries
 */

#ifndef INC_VEC_H_
#define INC_VEC_H_

typedef struct {
	float x;
	float y;
	float z;
} vec_t;

// lora data
// float + int + int = 3 * 4B data = 12B data
typedef struct {
	float speed;		// m/s
	float distance;		// meters
	int heart_rate;		// bpm
	int steps;
} lora_data_t;


#endif /* INC_VEC_H_ */
