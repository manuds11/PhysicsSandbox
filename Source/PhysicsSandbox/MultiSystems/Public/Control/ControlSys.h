#pragma once

#include "Simulation/SysState.h"

#include <Eigen/Dense>

class FFullSys;

class FControlSys
{
public:
	FControlSys() = default;

	// Finite Difference
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

	// Kernel
	
	bool BuildKernelBasis_FR(
		const Eigen::MatrixXd& H_X_FR,
		Eigen::MatrixXd& OutT_kerToFR
	) const;
	
	bool BuildJandJDot_FR(
		const FFullSys& FullSys,
		Eigen::MatrixXd& OutJ_FR,
		Eigen::MatrixXd& OutJDot_FR
	) const;

	bool BuildH_FR(
		const Eigen::MatrixXd& J_FR,
		const Eigen::MatrixXd& JDot_FR,
		Eigen::MatrixXd& OutH_X_FR
	) const;

	// Vector Transformations

	Eigen::VectorXd TransformDeltaX_ker2FR(
		const Eigen::MatrixXd& T_kerToFR,
		const Eigen::VectorXd& DeltaX_ker
	) const;

	Eigen::VectorXd ExpandDeltaX_FRToFull(
		const FStateDOFIndexMap& IndexMap,
		const Eigen::VectorXd& DeltaX_FR
	) const;

private:
	bool ComputeA_FullColumn(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		int32 PerturbationIndexX,
		double Epsilon,
		Eigen::VectorXd& OutA_iColumn
	);
};
