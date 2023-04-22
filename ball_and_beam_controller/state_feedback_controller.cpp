#include "state_feedback_controller.hpp"

// template <size_t state_dimension, size_t control_dimension>
// StateFeedbackController<state_dimension, control_dimension>::StateFeedbackController(controller_gain_matrix_t K)
// {
//     K_ = K;
// }

// template <size_t state_dimension, size_t control_dimension>
// StateFeedbackController<state_dimension,
//                         control_dimension>::~StateFeedbackController()
// {
// }

// template <size_t state_dimension, size_t control_dimension>
// typename StateFeedbackController<state_dimension, control_dimension>::input_vector_t StateFeedbackController<state_dimension,
//                                                                                                              control_dimension>::compute_control_input(input_vector_t u_ref, state_vector_t x_ref, state_vector_t x_k)
// {
//     input_vector_t control_input = -K_ * (x_k - x_ref) + u_ref;
//     return control_input;
// }
