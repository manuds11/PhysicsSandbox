#pragma once

#include "Simulation/SysState.h"

#include <Eigen/Dense>

class FFullSys;

class FControlSys
{
public:
	FControlSys() = default;

	bool ComputeA_FR(
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

	bool BuildJandJDot_FR(
		const FFullSys& FullSys,
		Eigen::MatrixXd& OutJ_FR,
		Eigen::MatrixXd& OutJDot_FR
	);

	bool BuildH_FR(
		const Eigen::MatrixXd& J_FR,
		const Eigen::MatrixXd& JDot_FR,
		Eigen::MatrixXd& OutH_X_FR
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
