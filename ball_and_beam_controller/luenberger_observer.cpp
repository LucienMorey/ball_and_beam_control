#include "luenberger_observer.hpp"

LuenbergerObserver::LuenbergerObserver(BLA::Matrix<4, 4> A, BLA::Matrix<4, 1> B,
                                       BLA::Matrix<2, 4> C, BLA::Matrix<4, 2> L, BLA::Matrix<4, 1> x_hat_0)
{
    A_ = A;
    B_ = B;
    C_ = C;

    L_ = L;

    x_hat_k_ = x_hat_0;
}

LuenbergerObserver::~LuenbergerObserver()
{
}

BLA::Matrix<4, 1> LuenbergerObserver::compute_observation(double u_k, BLA::Matrix<2, 1> z_k)
{

    BLA::Matrix<2, 1> predicted_output = C_ * x_hat_k_;

    x_hat_k_ = A_ * x_hat_k_ + B_ * u_k + L_ * (z_k - predicted_output);

    return x_hat_k_;
}