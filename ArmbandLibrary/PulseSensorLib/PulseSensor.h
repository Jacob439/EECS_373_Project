#ifndef PULSESENSOR_H
#define PULSESENSOR_H

#include "stdbool.h"
#include "stdlib.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"

/* Macros */
#define MAX_VOLT 3.2
#define BUF_LENGTH 10
#define SAMPLE_FREQ 50 		// 50 Hz sample frequency
#define SAMPLE_PERIOD 20	// 20 ms sample frequency

#define DICROTIC_BUFFER IBI (IBI * 3) / 5
#define THRESH_DEFAULT MAX_VOLT / 2.0f

/* Static Variables */
// ADC handler
ADC_HandleTypeDef* hadc;

// variable used to 
static unsigned char new_signal = false;

// metric variables
static int BPM = 0;		// Beats Per Minute
static int IBI = 100;	// Inter-Beat Interval (ms)

// IBI buffer
static int rate[BUF_LENGTH];

// value read from ADC
static float signal;	// signal output by the pulse sensor

// variables used to determine BPM and IBI
/* initialize thresh, pulse and trough amplitudes to half range */
static float amp = MAX_VOLT / 10.0f;
static float thresh = MAX_VOLT / 2.0f;
static float peak_amp = MAX_VOLT / 2.0f;
static float trough_amp = MAX_VOLT / 2.0f;
static bool first_beat = true;		// first beat bool
static bool second_beat = false; 	// second beat bool
static bool pulse = false; 		// pulse recognized bool
static int samples_since_last_beat = 0;

/* Function Prototypes*/

// init PulseSensor with adc it will use
// probably don't need timer handle since that
// will likely be handled in main, but maybe
// calls read_adc() and get_pulse()
void initPulseSensor(ADC_HandleTypeDef*);

// Called when the timer overflows
// can be used with polling or with 
// timer callback
void updatePulseSensor(void);

// sets signal with converted value
void read_ADC(void);

// uses static signal and rate buffer
// to update IBI and BPM
void get_pulse(void);

// simply returns value held in BPM var
int get_BPM() {
  return BPM;
}

// simply returns value held in IBI var
int get_IBI() {
  return IBI;
}

#endif