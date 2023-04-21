#include "lqr_controller.hpp"

#include <limits.h>

template <size_t state_dimension, size_t control_dimension>
LqrController<state_dimension, control_dimension>::LqrController(state_matrix_t A, control_matrix_t B,
                                                                 state_penalty_matrix_t Q, control_penalty_matrix_t R, double max_error, uint32_t max_iterations)
{
    A_ = A;
    B_ = B;
    Q_ = Q;
    R_ = R;

    max_error_ = max_error;
    max_iterations_ = max_iterations;
    state_dimension_ = state_dimension;
    control_dimension_ = control_dimension;
}

template <size_t state_dimension, size_t control_dimension>
LqrController<state_dimension, control_dimension>::~LqrController()
{
}

template <size_t state_dimension, size_t control_dimension>
typename LqrController<state_dimension, control_dimension>::input_vector_t LqrController<state_dimension, control_dimension>::compute_control_input(input_vector_t u_ref, state_vector_t x_ref, state_vector_t x_k)
{
    Eigen::MatrixXd P = solve_dare(A_, B_, Q_, R_);
    K_ = (R_ + B_.transpose() * P * B_).inverse() * B_.transpose() * P * A_;

    input_vector_t control_input = -K_ * (x_k - x_ref) + u_ref;

    return control_input;
}

template <size_t state_dimension, size_t control_dimension>
typename LqrController<state_dimension, control_dimension>::state_matrix_t LqrController<state_dimension, control_dimension>::solve_dare(const state_matrix_t &A, const control_matrix_t &B, const state_penalty_matrix_t &Q, const control_penalty_matrix_t &R)
{
    state_matrix_t P_last = Eigen::MatrixXd::Identity(state_dimension_, state_dimension_);
    state_matrix_t P;
    uint32_t current_iteration = 0U;
    double current_error = std::numeric_limits<double>::max();

    while ((current_iteration < max_iterations_) && (current_error > max_error_))
    {
        current_iteration++;
        P = Q + A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

        current_error = (P_last - P).cwiseAbs().maxCoeff();
        P_last = P;
    }
    // TODO  how to handle failure?
    return P;
}

template <size_t state_dimension, size_t control_dimension>
typename LqrController<state_dimension, control_dimension>::gain_matrix_t LqrController<state_dimension, control_dimension>::get_last_gain_matrix()
{
    return K_;
}