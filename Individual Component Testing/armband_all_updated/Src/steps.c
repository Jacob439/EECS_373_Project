///*
// * steps.c
// *
// *  Created on: Apr 9, 2024
// *      Author: masondevries
// */
//
//#include "steps.h"
//#include "stdbool.h"
//#include "vec.h"
//#include "stdio.h"
//
//
//static float curr_step_mag = 0;
//static int samples_since_last_step = 0;
//static float amp = 5;
//static float thresh = STEP_THRESH_DEFAULT;
//static float peak_amp = STEP_THRESH_DEFAULT;
//static float min_peak_amp = STEP_THRESH_DEFAULT;
//static float trough_amp = -STEP_THRESH_DEFAULT;
//// metric variables
//static int steps = 0;
//
//// variables used to determine BPM and IBI
///* initialize thresh, pulse and trough amplitudes to half range */
//
//static bool first_step = true;		// first beat bool
//static bool second_step = false; 	// second beat bool
//static bool step = false; 		// pulse recognized bool
//
//void input_step_data(vec_t gravity_vec, vec_t acc_vec) {
//	curr_step_mag = 0;
//
//	// normalize gravity vector
//	gravity_vec.x /= 9.8;
//	gravity_vec.y /= 9.8;
//	gravity_vec.z /= 9.8;
//
//	// dot product
//	// negative to make upwards acceleration positive
//	curr_step_mag -= acc_vec.x * gravity_vec.x;
//	curr_step_mag -= acc_vec.y * gravity_vec.y;
//	curr_step_mag -= acc_vec.z * gravity_vec.z;
//
//	printf("stepsignal: %f\n\r", curr_step_mag);
//	get_step();
//	return steps;
//}
//
//void get_step() {
//	++samples_since_last_step;
//	int N = samples_since_last_step * STEP_SAMPLE_PERIOD;
//
//	if (curr_step_mag < thresh && curr_step_mag < trough_amp) { // update trough
//		trough_amp = curr_step_mag;
//	}// if.. lower trough
//
//	if (curr_step_mag > thresh && curr_step_mag > peak_amp && curr_step_mag > min_peak_amp) {  // thresh condition helps avoid noise
//		peak_amp = curr_step_mag;
//	}// if.. higher peak
//
//	//  NOW IT'S TIME TO LOOK FOR THE step
//	// signal surges up in value every time there is a step
//	if (N > 100 && (curr_step_mag > thresh) && (step == false) ) {
//		step = true;                             // set the Pulse flag when we think there is a pulse
//	  	samples_since_last_step = 0;
//	  	++steps;
//	}// if.. new step
//
//	if (curr_step_mag < thresh && step) {  // when the values are going down, the beat is over
//		step = false;                         // reset the Pulse flag so we can do it again
//		amp = peak_amp - trough_amp;                           // get amplitude of the pulse wave
//		thresh = amp / 2 + trough_amp;                  // set thresh at 50% of the amplitude
//	    peak_amp = thresh;                            // reset these for next time
//	    trough_amp = thresh;
//	}// if.. step over
//
//	if (N > 2500) {                          // if 2.5 seconds go by without a beat
//	    thresh = STEP_THRESH_DEFAULT;                // set thresh default
//	    peak_amp = STEP_THRESH_DEFAULT;                               // set P default
//	    trough_amp = STEP_THRESH_DEFAULT;                               // set T default
//	    samples_since_last_step = 0;          // bring the lastBeatTime up to date
//	    step = false;
//	    amp = 20;
//	}// if.. N>2500
//}
//
//int get_step_count(void) {
//	return steps;
//}
//
