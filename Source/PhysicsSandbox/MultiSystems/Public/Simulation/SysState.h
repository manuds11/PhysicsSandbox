#pragma once

#include <Eigen/Dense>

struct FSysState
{
    Eigen::VectorXd Q;
    Eigen::VectorXd QDot;
};