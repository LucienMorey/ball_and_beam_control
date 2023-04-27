#ifndef LUENBERGER_OBSERVER_H
#define LUENBERGER_OBSERVER_H

#include <ArduinoEigen.h>

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
class LuenbergerObserver
{
public:
    typedef Eigen::Matrix<double, state_dimension, state_dimension> state_matrix_t;
    typedef Eigen::Matrix<double, state_dimension, control_dimension> control_matrix_t;
    typedef Eigen::Matrix<double, output_dimension, state_dimension> output_matrix_t;

    typedef Eigen::Matrix<double, state_dimension, 1> state_vector_t;
    typedef Eigen::Matrix<double, control_dimension, 1> input_vector_t;
    typedef Eigen::Matrix<double, output_dimension, 1> observation_vector_t;

    typedef Eigen::Matrix<double, state_dimension, output_dimension> observation_gain_matrix_t;

    LuenbergerObserver(state_matrix_t A, control_matrix_t B,
                       output_matrix_t C, observation_gain_matrix_t L, state_vector_t x_hat_0)
    {
        A_ = A;
        B_ = B;
        C_ = C;

        L_ = L;

        x_hat_k_ = x_hat_0;
    }

    ~LuenbergerObserver() {}

    state_vector_t compute_observation(input_vector_t u_k, observation_vector_t z_k)
    {

        observation_vector_t predicted_output = C_ * x_hat_k_;
        x_hat_k_ = A_ * x_hat_k_ + B_ * u_k + L_ * (z_k - predicted_output);

        return x_hat_k_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    state_matrix_t A_;
    control_matrix_t B_;
    output_matrix_t C_;

    observation_gain_matrix_t L_;

    state_vector_t x_hat_k_;
};

#endif