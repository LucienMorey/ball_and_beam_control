#ifndef LUENBERGER_OBSERVER_H
#define LUENBERGER_OBSERVER_H

#include <BasicLinearAlgebra.h>

class LuenbergerObserver
{
public:
    LuenbergerObserver(BLA::Matrix<4, 4> A, BLA::Matrix<4, 1> B,
                       BLA::Matrix<2, 4> C, BLA::Matrix<4, 2> L, BLA::Matrix<4, 1> x_hat_0);
    ~LuenbergerObserver();

    BLA::Matrix<4, 1> compute_observation(double u_k, BLA::Matrix<2, 1> z_k);

private:
    BLA::Matrix<4, 4> A_;
    BLA::Matrix<4, 1> B_;
    BLA::Matrix<2, 4> C_;

    BLA::Matrix<4, 2> L_;

    BLA::Matrix<4, 1> x_hat_k_;
};

#endif