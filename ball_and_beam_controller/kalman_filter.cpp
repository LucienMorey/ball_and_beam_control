#include "kalman_filter.hpp"

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
KalmanFilter<state_dimension, control_dimension, output_dimension>::KalmanFilter(state_matrix_t A, control_matrix_t B,
                                                                                 output_matrix_t C, state_penalty_matrix_t Q,
                                                                                 observation_penalty_matrix_t R, state_vector_t x_hat_0,
                                                                                 state_matrix_t P_0)
{
    A_ = A;
    B_ = B;
    C_ = C;

    Q_ = Q;
    R_ = R;

    P_k_ = P_0;
    x_hat_k_ = x_hat_0;

    state_dimension_ = state_dimension;
    control_dimension_ = control_dimension;
    output_dimension_ = output_dimension;
}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
KalmanFilter<state_dimension, control_dimension, output_dimension>::~KalmanFilter() {}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
typename KalmanFilter<state_dimension, control_dimension, output_dimension>::state_vector_t KalmanFilter<state_dimension, control_dimension, output_dimension>::predict_state(input_vector_t control_input)
{

    return A_ * x_hat_k_ + B_ * control_input;
}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
typename KalmanFilter<state_dimension, control_dimension, output_dimension>::state_matrix_t KalmanFilter<state_dimension, control_dimension, output_dimension>::predict_covariance(void)
{
    return A_ * P_k_ * A_.transpose() + Q_;
}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
typename KalmanFilter<state_dimension, control_dimension, output_dimension>::observation_gain_matrix_t KalmanFilter<state_dimension, control_dimension, output_dimension>::compute_kalman_gain(state_matrix_t predicted_covariance)
{
    // THis function modifies in place (matrix is parsed by reference)
    return predicted_covariance * C_.transpose() * (C_ * predicted_covariance * C_.transpose() + R_).inverse();
}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
typename KalmanFilter<state_dimension, control_dimension, output_dimension>::observation_vector_t KalmanFilter<state_dimension, control_dimension, output_dimension>::predict_measurement(state_vector_t predicted_state)
{
    return C_ * predicted_state;
}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
typename KalmanFilter<state_dimension, control_dimension, output_dimension>::state_vector_t KalmanFilter<state_dimension, control_dimension, output_dimension>::compute_updated_state_estimate(state_vector_t predicted_state,
                                                                                                                                                                                               observation_gain_matrix_t kalman_gain,
                                                                                                                                                                                               observation_vector_t measurement,
                                                                                                                                                                                               observation_vector_t predicted_measurement)

{
    last_innovation_ = measurement - predicted_measurement;

    return predicted_state + kalman_gain * last_innovation_;
}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
typename KalmanFilter<state_dimension, control_dimension, output_dimension>::state_matrix_t KalmanFilter<state_dimension, control_dimension, output_dimension>::compute_updated_covariance(observation_gain_matrix_t kalman_gain, state_matrix_t predicted_covariance)
{
    return (Eigen::MatrixXd::Identity(state_dimension_, state_dimension_) - kalman_gain * C_) * predicted_covariance;
}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
typename KalmanFilter<state_dimension, control_dimension, output_dimension>::observation_vector_t KalmanFilter<state_dimension, control_dimension, output_dimension>::get_last_innovation(void)
{
    return last_innovation_;
}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
typename KalmanFilter<state_dimension, control_dimension, output_dimension>::observation_gain_matrix_t KalmanFilter<state_dimension, control_dimension, output_dimension>::get_last_kalman_gain(void)
{
    return kalman_gain_;
}

template <size_t state_dimension, size_t control_dimension, size_t output_dimension>
typename KalmanFilter<state_dimension, control_dimension, output_dimension>::state_vector_t KalmanFilter<state_dimension, control_dimension, output_dimension>::filter(input_vector_t control_input, observation_vector_t measurement)
{
    // Prediction
    state_vector_t predicted_state = predict_state(control_input);
    state_matrix_t predicted_covariance = predict_covariance();

    // update
    kalman_gain_ = compute_kalman_gain(predicted_covariance);
    observation_vector_t predicted_measurement = predict_measurement(predicted_state);

    state_vector_t estimated_state = compute_updated_state_estimate(predicted_state, kalman_gain_, measurement, predicted_measurement);
    state_matrix_t updated_covariance = compute_updated_covariance(kalman_gain_, predicted_covariance);

    x_hat_k_ = estimated_state;
    P_k_ = updated_covariance;

    return x_hat_k_;
}
