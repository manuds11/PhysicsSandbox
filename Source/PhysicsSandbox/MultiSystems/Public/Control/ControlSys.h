#pragma once

#include "Simulation/SysState.h"

#include <Eigen/Dense>

class FFullSys;


class FControlSys
{
public:
	FControlSys() = default;

	bool ComputeStateJacobianColumn(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		int32 PerturbationIndexX,
		double Epsilon,
		Eigen::VectorXd& OutA_iColumn
	);
};