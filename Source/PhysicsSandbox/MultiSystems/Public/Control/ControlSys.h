#pragma once

#include "Simulation/SysState.h"

#include <Eigen/Dense>

class FFullSys;

class FControlSys
{
public:
	FControlSys() = default;

	bool ComputeAStateJacobian(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		double Epsilon,
		Eigen::MatrixXd& OutA_Full
	);

	bool ComputeAFreeRedundant(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		double Epsilon,
		Eigen::MatrixXd& OutA_FR
	);

	void BuildAFreeRedundantInFullDisplay(
		const FStateDOFIndexMap& IndexMap,
		const Eigen::MatrixXd& A_FreeRedundant,
		Eigen::MatrixXd& OutA_FullDisplay
	);

private:
	bool ComputeA_FullColumn(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		int32 PerturbationIndexX,
		double Epsilon,
		Eigen::VectorXd& OutA_iColumn
	);
};
