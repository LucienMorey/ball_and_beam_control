#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <ArduinoEigen.h>
#include <limits.h>

template <size_t state_dimension, size_t control_dimension>
class LqrController
{

    typedef Eigen::Matrix<double, state_dimension, state_dimension> state_matrix_t;
    typedef Eigen::Matrix<double, state_dimension, control_dimension> control_matrix_t;
    typedef Eigen::Matrix<double, state_dimension, state_dimension> state_penalty_matrix_t;
    typedef Eigen::Matrix<double, control_dimension, control_dimension> control_penalty_matrix_t;
    typedef Eigen::Matrix<double, state_dimension, 1> state_vector_t;
    typedef Eigen::Matrix<double, control_dimension, 1> input_vector_t;
    typedef Eigen::Matrix<double, control_dimension, state_dimension> gain_matrix_t;

public:
    LqrController(state_matrix_t A, control_matrix_t B,
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

    ~LqrController() {}

    input_vector_t compute_control_input(input_vector_t u_ref, state_vector_t x_ref, state_vector_t x_k)
    {
        Eigen::MatrixXd P = solve_dare(A_, B_, Q_, R_);
        K_ = (R_ + B_.transpose() * P * B_).inverse() * B_.transpose() * P * A_;

        input_vector_t control_input = -K_ * (x_k - x_ref) + u_ref;

        return control_input;
    }

    gain_matrix_t get_last_gain_matrix()
    {
        return K_;
    }

private:
    state_matrix_t solve_dare(const state_matrix_t &A, const control_matrix_t &B, const state_penalty_matrix_t &Q, const control_penalty_matrix_t &R)
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

    state_matrix_t A_;
    control_matrix_t B_;

    state_penalty_matrix_t Q_;
    control_penalty_matrix_t R_;

    gain_matrix_t K_;

    double max_error_;
    double max_iterations_;

    size_t state_dimension_;
    size_t control_dimension_;
};

#endif