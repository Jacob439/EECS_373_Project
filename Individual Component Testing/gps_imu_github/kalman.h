/*
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */

#ifndef SimpleKalmanFilter_H_
#define SimpleKalmanFilter_H_
#include "main.h"

struct Kalman {

  float _err_measure;
  float _err_estimate;
  float _q;
  float _kalman_gain;
  float _current_estimate;
  float _last_estimate;

};

void SimpleKalmanFilter(float mea_e, float est_e, float q, struct Kalman   k);
float updateEstimate(float mea, struct Kalman*   k);
void setMeasurementError(float mea_e, struct Kalman*   k);
void setEstimateError(float est_e, struct Kalman   k);
void setProcessNoise(float q, struct Kalman   k);
float getKalmanGain(struct Kalman   k);
float getEstimateError(struct Kalman   k);



#endif
