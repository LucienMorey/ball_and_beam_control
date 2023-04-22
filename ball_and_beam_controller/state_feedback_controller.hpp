#ifndef STATE_FEEDBACK_CONTROLLER_H
#define STATE_FEEDBACK_CONTROLLER_H

#include <ArduinoEigen.h>

template <size_t state_dimension, size_t control_dimension>
class StateFeedbackController
{

    typedef Eigen::Matrix<double, state_dimension, 1> state_vector_t;
    typedef Eigen::Matrix<double, control_dimension, 1> input_vector_t;

    typedef Eigen::Matrix<double, control_dimension, state_dimension> controller_gain_matrix_t;

public:
    StateFeedbackController(controller_gain_matrix_t K)
    {
        K_ = K;
    }
    ~StateFeedbackController() {}

    input_vector_t compute_control_input(input_vector_t u_ref, state_vector_t x_ref, state_vector_t x_k)
    {
        input_vector_t control_input = -K_ * (x_k - x_ref) + u_ref;
        return control_input;
    }

private:
    controller_gain_matrix_t K_;
};

#endif