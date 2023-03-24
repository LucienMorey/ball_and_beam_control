#ifndef STATE_FEEDBACK_CONTROLLER_H
#define STATE_FEEDBACK_CONTROLLER_H

#include <BasicLinearAlgebra.h>

class StateFeedbackController
{
public:
    StateFeedbackController(BLA::Matrix<1, 4> K);
    ~StateFeedbackController();

    double compute_control_input(double u_ref, BLA::Matrix<4, 1> x_ref, BLA::Matrix<4, 1> x_k);

private:
    BLA::Matrix<1, 4> K_;
};

#endif