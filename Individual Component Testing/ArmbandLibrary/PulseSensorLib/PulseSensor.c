/**
 * Pulse Sensor Library 
 * by Mason
 * 
 * Usage:
 *  - TODO lol
*/

#include "PulseSensor.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"

void initPulseSensor(ADC_HandleTypeDef *hadc_in) {
	hadc = hadc_in;
}

void updatePulseSensor(void){
	read_ADC();
	get_pulse();
}

void read_ADC(void) {
	//uint32_t val;
	unsigned int val;

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 0xFFFFFFFF);
	val = HAL_ADC_GetValue(hadc);
	signal = val * 3.3f / 4096.0f;
}

void get_pulse() {
	++samples_since_last_beat;
	int N = samples_since_last_beat * SAMPLE_PERIOD;
	if (signal < thresh && N > (IBI / 5) * 3) { // avoid dicrotic noise by waiting 3/5 of last IBI
	    if (signal < trough_amp) {                        // T is the trough
	      trough_amp = signal;                            // keep track of lowest point in pulse wave
	    }
	  }

	  if (signal > thresh && signal > peak_amp) {       // thresh condition helps avoid noise
		  peak_amp = signal;                              // P is the peak
	  }                                          // keep track of highest point in pulse wave

	  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
	  // signal surges up in value every time there is a pulse
	  if (N > 250) {                             // avoid high frequency noise
	    if ( (signal > thresh) && (pulse == false) && (N > ((IBI / 5) * 3)) ) {
	      pulse = true;                             // set the Pulse flag when we think there is a pulse
	      IBI = N;    // measure time between beats in mS
	      samples_since_last_beat = 0;

	      if (second_beat) {                      // if this is the second beat, if secondBeat == TRUE
	        second_beat = false;                    // clear secondBeat flag
	        for (int i = 0; i < BUF_LENGTH; i++) {       // seed the running total to get a realisitic BPM at startup
	          rate[i] = IBI;
	        }
	      }

	      if (first_beat) {                       // if it's the first time we found a beat, if firstBeat == TRUE
	    	first_beat = 0;                       // clear firstBeat flag
	    	second_beat = 1;                      // set the second beat flag
	        // IBI value is unreliable so discard it
	        return;
	      }


	      // keep a running total of the last 10 IBI values
	      int runningTotal = 0;                  // clear the runningTotal variable

	      for (int i = 0; i < BUF_LENGTH - 1; i++) {          // shift data in the rate array
	        rate[i] = rate[i + 1];                // and drop the oldest IBI value
	        runningTotal += rate[i];              // add up the 9 oldest IBI values
	      }

	      rate[BUF_LENGTH - 1] = IBI;                          // add the latest IBI to the rate array
	      runningTotal += rate[BUF_LENGTH - 1];                // add the latest IBI to runningTotal
	      runningTotal /= BUF_LENGTH;                     // average the last 10 IBI values
	      BPM = 60000 / runningTotal;             // how many beats can fit into a minute? that's BPM!
	      //fadeLevel = MAX_FADE_LEVEL;             // If we're fading, re-light that LED.
	    }
	  }

	  if (signal < thresh && pulse) {  // when the values are going down, the beat is over
	    pulse = false;                         // reset the Pulse flag so we can do it again
	    amp = peak_amp - trough_amp;                           // get amplitude of the pulse wave
	    thresh = amp / 2 + trough_amp;                  // set thresh at 50% of the amplitude
	    peak_amp = thresh;                            // reset these for next time
	    trough_amp = thresh;
	  }

	  if (N > 2500) {                          // if 2.5 seconds go by without a beat
	    thresh = THRESH_DEFAULT;                // set thresh default
	    peak_amp = THRESH_DEFAULT;                               // set P default
	    trough_amp = THRESH_DEFAULT;                               // set T default
	    samples_since_last_beat = 0;          // bring the lastBeatTime up to date
	    first_beat = true;                      // set these to avoid noise
	    second_beat = false;                    // when we get the heartbeat back
	    BPM = 0;
	    IBI = 600;                  // 600ms per beat = 100 Beats Per Minute (BPM)
	    pulse = false;
	    amp = 100;                  // beat amplitude 1/10 of input range.

	  }
}