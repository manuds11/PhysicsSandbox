#pragma once

#include <Eigen/Dense>

struct FSysState
{
    Eigen::VectorXd Q_Full;
    Eigen::VectorXd QDot_Full;
};