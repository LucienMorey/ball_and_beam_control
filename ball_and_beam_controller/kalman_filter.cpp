#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(BLA::Matrix<4, 4> A, BLA::Matrix<4, 1> B,
                           BLA::Matrix<2, 4> C, BLA::Matrix<4, 4> Q,
                           BLA::Matrix<2, 2> R, BLA::Matrix<4, 1> x_hat_0,
                           BLA::Matrix<4, 4> P_0)
{
    A_ = A;
    B_ = B;
    C_ = C;

    Q_ = Q;
    R_ = R;

    P_k_ = P_0;
    x_hat_k_ = x_hat_0;
}

KalmanFilter::~KalmanFilter() {}

BLA::Matrix<4, 1> KalmanFilter::predict_state(double control_input)
{

    return A_ * x_hat_k_ + B_ * control_input;
}

BLA::Matrix<4, 4> KalmanFilter::predict_covariance(void)
{
    return A_ * P_k_ * ~A_ + Q_;
}

BLA::Matrix<4, 2> KalmanFilter::compute_kalman_gain(BLA::Matrix<4, 4> predicted_covariance)
{
    // Does this term have a name?
    BLA::Matrix<2, 2> thing = C_ * predicted_covariance * ~C_ + R_;
    // THis function modifies in place (matrix is parsed by reference)
    Invert(thing);
    return predicted_covariance * ~C_ * thing;
}

BLA::Matrix<2, 1> KalmanFilter::predict_measurement(BLA::Matrix<4, 1> predicted_state)
{
    return C_ * predicted_state;
}

BLA::Matrix<4, 1> KalmanFilter::compute_updated_state_estimate(BLA::Matrix<4, 1> predicted_state,
                                                               BLA::Matrix<4, 2> kalman_gain,
                                                               BLA::Matrix<2, 1> measurement,
                                                               BLA::Matrix<2, 1> predicted_measurement)

{
    last_innovation_ = measurement - predicted_measurement;

    return predicted_state + kalman_gain * last_innovation_;
}

BLA::Matrix<4, 4> KalmanFilter::compute_updated_covariance(BLA::Matrix<4, 2> kalman_gain, BLA::Matrix<4, 4> predicted_covariance)
{
    BLA::Matrix<4, 4> eye_4 = {1.0, 0.0, 0.0, 0.0,
                               0.0, 1.0, 0.0, 0.0,
                               0.0, 0.0, 1.0, 0.0,
                               0.0, 0.0, 0.0, 1.0};
    return (eye_4 - kalman_gain * C_) * predicted_covariance;
}

BLA::Matrix<2, 1> KalmanFilter::get_last_innovation(void)
{
    return last_innovation_;
}

BLA::Matrix<4, 1> KalmanFilter::filter(double control_input, BLA::Matrix<2, 1> measurement)
{
    // Prediction
    BLA::Matrix<4, 1> predicted_state = predict_state(control_input);
    BLA::Matrix<4, 4> predicted_covariance = predict_covariance();

    // update
    BLA::Matrix<4, 2> kalman_gain = compute_kalman_gain(predicted_covariance);
    BLA::Matrix<2, 1> predicted_measurement = predict_measurement(predicted_state);

    BLA::Matrix<4, 1> estimated_state = compute_updated_state_estimate(predicted_state, kalman_gain, measurement, predicted_measurement);
    BLA::Matrix<4, 4> updated_covariance = compute_updated_covariance(kalman_gain, predicted_covariance);

    x_hat_k_ = estimated_state;
    P_k_ = updated_covariance;

    return x_hat_k_;
}