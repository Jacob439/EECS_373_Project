// analytics source file
// by mason

/** NOTES
 *   - add safeguards
 *      - only update strain when velocity above threshold
 *        (avoid divisino by zero / non-exercising data)
 * 
 */

#include "strain.h"
#include "limits.h"     // for INT_MAX

/* static variables */
static float data[DATA_BUFFER_LENGTH];  // vector of strain values
static unsigned int data_index = 0;
static float current_strain;   

// state variable
static State_t state = k_pre_init;   

// strain variables
static float exercise_base_strain = -1;
static float current_strain = -1;

// heartbeat threshold variables
// (calculated from exercise baseline)
// ~80% of minimum optimal heart rate 0.8 * 0.64*(220-age)
static int age = 0;
static int heart_threshold = INT_MAX;

void init_analytics(int age) {
  heart_threshold = (220-age)*0.60;
  state = k_post_init;
}

// Heart Rate too High
uint8_t heartRateHigh(int bpm, int age) {
	int maxHR = 220 - age;
	if (bpm > maxHR*0.80) return 1;
	return 0;
}

// Heart Rate too Low
uint8_t heartRateLow(int bpm, int age) {
	int maxHR = 220 - age;
	if (bpm < maxHR*0.60) return 1;
	return 0;
}

// Requires: speed in meters/s
void input_data(int bpm, float speed) {
  switch (state) {
    case k_pre_init:
      /* idle until init is called */
      break;
    case k_post_init:
      // if athlete starts moving or heart indicates exercise
      if (speed > SPEED_THRESHOLD || bpm > heart_threshold) {
        state = k_exercise_baseline;
      }
      break;
    case k_exercise_baseline:
      update_data(bpm, speed);
      if (data_index == DATA_BUFFER_LENGTH) {
        calculate_exercise_strain();
        data_index = 0;
        state = k_exercise;
      }
      break;
    case k_exercise:
      current_strain = get_strain(bpm, speed);
      break;
  }
}

float get_strain_factor() {
  if (state != k_exercise || current_strain == -1) return -1;
  return (exercise_base_strain / current_strain);
}

inline
float get_strain(int bpm, float speed) {
	// -1 if divide by zero, will not end up being pushed
	// to data array or updating curr_strain
  return (speed == 0) ? -1 : bpm / (speed * 60.0f);
}

void update_data(int bpm, float speed) {
  current_strain = get_strain(bpm, speed);
  if (current_strain == -1) return;
  data[data_index] = current_strain;
  ++data_index;
}

void calculate_exercise_strain(void) {
  exercise_base_strain = 0;
  for (int i = 0; i < DATA_BUFFER_LENGTH; ++i) {
    exercise_base_strain += data[i];
  }
  exercise_base_strain /= DATA_BUFFER_LENGTH;
}
