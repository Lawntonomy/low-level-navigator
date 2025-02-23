#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(float process_noise, float measurement_noise, float estimate_error)
    : Q(process_noise), R(measurement_noise), P(estimate_error), x(0.0)
{
}

float KalmanFilter::update(float measurement)
{
    // Prediction step
    float P_pred = P + Q;

    // Kalman gain
    float K = P_pred / (P_pred + R);

    // Correction step
    x = x + K * (measurement - x);
    P = (1 - K) * P_pred;

    return x;
}
