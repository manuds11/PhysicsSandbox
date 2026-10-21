#pragma once

#include "Simulation/SysState.h"

#include <Eigen/Dense>

class FFullSys;

class FControlSys
{
public:
	FControlSys() = default;

	bool BuildJandJDot_FreeRedundant(
		const FFullSys& FullSys,
		Eigen::MatrixXd& OutJ_FR,
		Eigen::MatrixXd& OutJDot_FR
	);

	bool ComputeA_FreeRedundant(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		double Epsilon,
		Eigen::MatrixXd& OutA_FR
	);

	void BuildA_FRInFullDisplay(
		const FStateDOFIndexMap& IndexMap,
		const Eigen::MatrixXd& A_FR,
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
