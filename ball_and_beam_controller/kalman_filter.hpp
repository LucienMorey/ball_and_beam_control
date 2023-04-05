
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
/**
 * @file kalman_filter.hpp
 * @author Lucien Morey
 * @brief Kalman filter class definition
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <BasicLinearAlgebra.h>

class KalmanFilter
{
public:
    KalmanFilter(BLA::Matrix<4, 4> A, BLA::Matrix<4, 1> B,
                 BLA::Matrix<2, 4> C, BLA::Matrix<4, 4> Q,
                 BLA::Matrix<2, 2> R, BLA::Matrix<4, 1> x_hat_0,
                 BLA::Matrix<4, 4> P_0);
    ~KalmanFilter();

    BLA::Matrix<4, 1> filter(double control_input, BLA::Matrix<2, 1> measurement);
    BLA::Matrix<2, 1> get_last_innovation(void);
    BLA::Matrix<4, 2> get_last_kalman_gain(void);

private:
    BLA::Matrix<4, 1> predict_state(double control_input);
    BLA::Matrix<4, 4> predict_covariance(void);

    BLA::Matrix<4, 2> compute_kalman_gain(BLA::Matrix<4, 4> predicted_covariance);
    BLA::Matrix<2, 1> predict_measurement(BLA::Matrix<4, 1> predicted_state);
    BLA::Matrix<4, 1> compute_updated_state_estimate(BLA::Matrix<4, 1> predicted_state,
                                                     BLA::Matrix<4, 2> kalman_gain,
                                                     BLA::Matrix<2, 1> measurement,
                                                     BLA::Matrix<2, 1> predicted_measurement);
    BLA::Matrix<4, 4> compute_updated_covariance(BLA::Matrix<4, 2> kalman_gain, BLA::Matrix<4, 4> predicted_covaraiance);

    BLA::Matrix<4, 4> A_;
    BLA::Matrix<4, 1> B_;
    BLA::Matrix<2, 4> C_;

    BLA::Matrix<4, 4> Q_;
    BLA::Matrix<2, 2> R_;

    BLA::Matrix<4, 4> P_k_;
    BLA::Matrix<4, 1> x_hat_k_;

    BLA::Matrix<2, 1> last_innovation_;
    BLA::Matrix<4, 2> kalman_gain_;
};

#endif