#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <ArduinoEigen.h>

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
                  state_penalty_matrix_t Q, control_penalty_matrix_t R, double max_error, uint32_t max_iterations);
    ~LqrController();

    input_vector_t compute_control_input(input_vector_t u_ref, state_vector_t x_ref, state_vector_t x_k);

    gain_matrix_t get_last_gain_matrix();

private:
    state_matrix_t solve_dare(const state_matrix_t &A, const control_matrix_t &B, const state_penalty_matrix_t &Q, const control_penalty_matrix_t &R);

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