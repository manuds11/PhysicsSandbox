#pragma once

#include "Simulation/SysState.h"

#include <Eigen/Dense>

class FFullSys;

struct FControlForceContribution
{
	int32 BodyApplicationIndex = INDEX_NONE;

	// Cartesian force produced by one unit of this control channel.
	FVector2D ForcePerUnitU = FVector2D::ZeroVector;
};

struct FLinearControlChannel
{
	FName Name = NAME_None;

	TArray<FControlForceContribution> ForceContributions;

	FLinearControlChannel() = default;

	explicit FLinearControlChannel(
		FName InName
	)
		: Name(InName)
	{
	}

	void AddForceContribution(
		int32 BodyApplicationIndex,
		const FVector2D& ForcePerUnitU
	);
};

class FControlSys
{
public:
	FControlSys() = default;

	// -----------------------------------------------------------------------------
	// Control Channels
	// -----------------------------------------------------------------------------

	int32 AddLinearControlChannel(
		FLinearControlChannel Channel
	);

	bool BuildControlAllocationG_FRMatrix(
		const FFullSys& FullSys
	);

	Eigen::VectorXd ComputeTau_FR(
		const Eigen::VectorXd& U
	) const;

	int32 GetNumControlChannels() const
	{
		return ControlChannels.Num();
	}

	const Eigen::MatrixXd& GetG_UToTau_FR() const
	{
		return G_UToTau_FR;
	}

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
	TArray<FLinearControlChannel> ControlChannels;

	Eigen::MatrixXd G_UToTau_FR;

	bool bControlAllocationBuilt = false;
	
	bool ComputeA_FullColumn(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		int32 PerturbationIndexX,
		double Epsilon,
		Eigen::VectorXd& OutA_iColumn
	);
};
