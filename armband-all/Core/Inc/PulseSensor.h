#ifndef PULSESENSOR_H
#define PULSESENSOR_H

#include "stdbool.h"
#include "stdlib.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"

/* Macros */
#define MAX_VOLT 3.2
#define BUF_LENGTH 10
#define SAMPLE_FREQ 200 		// 50 Hz sample frequency
#define SAMPLE_PERIOD 5	// 20 ms sample frequency

#define DICROTIC_BUFFER IBI (IBI * 3) / 5
#define THRESH_DEFAULT MAX_VOLT / 2.0f

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
int get_BPM(void);

// simply returns value held in IBI var
int get_IBI(void);

#endif
