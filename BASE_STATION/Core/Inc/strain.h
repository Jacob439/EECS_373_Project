// analytics header file
// by mason

/**
 * USAGE:
 *    set input period to whatever period you are sending data to analytics
 *    and call input_data(bpm, speed) at that frequency
 *    
 *    call get_strain_factor() all you want, but it
 *    wont return anything but -1 until useful data
 *    is available (baselines are calculated)
*/

#include "main.h"

// data buffer can hold 10 minutes of data
#define INPUT_PERIOD 5   // seconds
#define BASELINE_LENGTH_MINUTES	2
#define DATA_BUFFER_LENGTH BASELINE_LENGTH_MINUTES * 60/INPUT_PERIOD
#define STRAIN_BUF_LENGTH 6

// maybe
#define SPEED_THRESHOLD 1.0f // speed threshold for start of workout?

typedef enum {
  k_pre_init = 0,           // must init in order to start analyzing
  k_post_init = 1,          // after baseline but before exercise
  k_exercise_baseline = 2,  // in first 2 minutes of exercise
  k_exercise = 3            // indefinite state of exercise
} State_t;


/* public functions (for use from main) */

// init analytics with age for heart rate threshold value
void init_analytics(int age);

// from main, simply call this function with relevant user
// data (TODO) and this file will handle it based on how much
// time is passed calculated by the number of data sent
// assuming it is sent at constant known intervals
// - this is the heart of the analytics file
void input_data(int bpm, float speed);

// call this function from main to get current strain factor
// returns -1 if strain factor is not relevant (baselines have
// not been calculated yet)
float get_strain_factor(void);

// self explanatory
uint8_t heartRateHigh(int bpm, int age);
uint8_t heartRateLow(int bpm, int age);

/* private function prototypes (for use in analytics.c) */

void update_data(int, float);
float get_strain(int, float);

// this function is called once by this file
// to calculate standard baseline (no strain)
// based on the first two minutes of metrics
void calculate_base_strain(void);

// this function is called once by this file once
// bpm or speed indicates exercise has started and two
// minutes have passed, then calculates strain baseline
void calculate_exercise_strain(void);
