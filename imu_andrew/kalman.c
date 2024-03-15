/*
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */

#include "kalman.h"
#include "main.h"
#include "math.h"

void SimpleKalmanFilter(float mea_e, float est_e, float q, struct Kalman k)
{
  k._err_measure=mea_e;
  k._err_estimate=est_e;
  k._q = q;
}

float updateEstimate(float mea, struct Kalman   k)
{
  k._kalman_gain = k._err_estimate/(k._err_estimate + k._err_measure);
  k._current_estimate = k._last_estimate + k._kalman_gain * (mea - k._last_estimate);
  k._err_estimate =  (1.0f - k._kalman_gain)*k._err_estimate + fabsf(k._last_estimate-k._current_estimate)*k._q;
  k._last_estimate=k._current_estimate;

  return k._current_estimate;
}

void setMeasurementError(float mea_e, struct Kalman   k) //pass in
{
	k._err_measure=mea_e;
}

void setEstimateError(float est_e, struct Kalman   k)
{
	k._err_estimate=est_e;
}

void setProcessNoise(float q, struct Kalman   k)
{
	k._q=q;
}

float getKalmanGain(struct Kalman   k) {
  return k._kalman_gain;
}

float getEstimateError(struct Kalman   k) {
  return k._err_estimate;
}
