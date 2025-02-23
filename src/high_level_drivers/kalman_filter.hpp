#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter
{
  public:
    KalmanFilter(float process_noise, float measurement_noise, float estimate_error);
    float update(float measurement);

  private:
    float x; // Estimated state (yaw rate)
    float P; // Error covariance
    float Q; // Process noise covariance
    float R; // Measurement noise covariance
};

#endif