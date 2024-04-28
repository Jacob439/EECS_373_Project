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
#define INPUT_PERIOD 10   // seconds
#define DATA_BUFFER_LENGTH 600/INPUT_PERIOD
#define TWO_MIN_NUM_DATAPOINTS 180/INPUT_PERIOD

// maybe
#define SPEED_THRESHOLD 1.5f // speed threshold for start of workout?

typedef enum {
  k_pre_init,           // must init in order to start analyzing
  k_init_baseline,      // in first 2 minutes (standard baseline)
  k_post_init,          // after baseline but before exercise
  k_exercise_baseline,  // in first 2 minutes of exercise
  k_exercise            // indefinite state of exercise
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


/* private function prototypes (for use in analytics.c) */

void update_data(int, float);
inline float get_strain(int, float);

// this function is called once by this file
// to calculate standard baseline (no strain)
// based on the first two minutes of metrics
void calculate_base_strain(void);

// this function is called once by this file once
// bpm or speed indicates exercise has started and two
// minutes have passed, then calculates strain baseline
void calculate_exercise_strain(void);