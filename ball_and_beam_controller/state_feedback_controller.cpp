#include "state_feedback_controller.hpp"

StateFeedbackController::StateFeedbackController(BLA::Matrix<1, 4> K)
{
    K_ = K;
}

StateFeedbackController::~StateFeedbackController()
{
}

double StateFeedbackController::compute_control_input(double u_ref, BLA::Matrix<4, 1> x_ref, BLA::Matrix<4, 1> x_k)
{
    auto control_input = -K_ * (x_k - x_ref) + u_ref;
    return control_input(0, 0);
}