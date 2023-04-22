#include "luenberger_observer.hpp"

// template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
// LuenbergerObserver<state_dimension, control_dimension, output_dimension>::LuenbergerObserver(state_matrix_t A, control_matrix_t B,
//                                                                                              output_matrix_t C, observation_gain_matrix_t L, state_vector_t x_hat_0)
// {
//     Serial.printf("fuck cunt fuck\n");
//     A_ = A;
//     B_ = B;
//     C_ = C;

//     L_ = L;

//     x_hat_k_ = x_hat_0;
//     Serial.printf("fuck cunt fuck 2\n");
// }

// template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
// LuenbergerObserver<state_dimension, control_dimension, output_dimension>::~LuenbergerObserver()
// {
// }

// template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
// typename LuenbergerObserver<state_dimension, control_dimension, output_dimension>::state_vector_t LuenbergerObserver<state_dimension, control_dimension, output_dimension>::compute_observation(input_vector_t u_k, observation_vector_t z_k)
// {

//     observation_vector_t predicted_output = C_ * x_hat_k_;
//     x_hat_k_ = A_ * x_hat_k_ + B_ * u_k + L_ * (z_k - predicted_output);

//     return x_hat_k_;
// }
