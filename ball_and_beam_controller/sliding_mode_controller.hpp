#ifndef SLIDING_MODE_CONTROLLER_H
#define SLIDING_MODE_CONTROLLER_H

#include <ArduinoEigen.h>

template <size_t state_dimension, size_t control_dimension>
class SlidingModeController
{
    typedef Eigen::Matrix<double, state_dimension, state_dimension> state_matrix_t;
    typedef Eigen::Matrix<double, state_dimension, control_dimension> control_matrix_t;
    typedef Eigen::Matrix<double, control_dimension, state_dimension> surface_matrix_t;

    typedef Eigen::Matrix<double, state_dimension, 1> state_vector_t;
    typedef Eigen::Matrix<double, control_dimension, 1> input_vector_t;

    typedef Eigen::Matrix<double, control_dimension, state_dimension> gain_matrix_t;

public:
    SlidingModeController(state_matrix_t A, control_matrix_t B, surface_matrix_t Cs, double gamma)
    {
        A_ = A;
        B_ = B;
        Cs_ = Cs;

        K_ = (Cs * B).inverse() * Cs * A;

        state_dimension_ = state_dimension;
        control_dimension_ = control_dimension;

        gamma_ = gamma;
    }
    ~SlidingModeController() {}

    input_vector_t compute_control_input(input_vector_t u_ref, state_vector_t x_ref, state_vector_t x_k)
    {

        state_vector_t x_e = x_k - x_ref;
        Eigen::Matrix<double, control_dimension_, 1> Sx = Cs_ * x_e;
        input_vector_t ueq = -K_ * x_e;
        input_vector_t usw = -(Cs_ * B_).inverse() * gamma_ * Sx.cwiseSign();

        return ueq + usw;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    state_matrix_t A_;
    control_matrix_t B_;
    surface_matrix_t Cs_;
    double gamma_;

    gain_matrix_t K_;

    size_t state_dimension_;
    size_t control_dimension_;
};

#endif