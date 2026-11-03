#pragma once

#include "Simulation/SysState.h"
#include "Simulation/FullSysTypes.h"

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

struct FControllabilityResult
{
	int32 StateDimension = 0;
	int32 ControlDimension = 0;

	int32 Rank = 0;
	bool bIsControllable = false;

	double RankTolerance = 0.0;
	double RelativeMinSingularValue = 0.0;

	Eigen::VectorXd SingularValues;
	Eigen::MatrixXd ControllabilityMatrix;
};

struct FDAREResult
{
	Eigen::MatrixXd P;

	int32 Iterations = 0;
	double Residual = 0.0;

	bool bConverged = false;
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
		const Eigen::VectorXd& U_Control
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

	bool ComputeA_FR_Continuous(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		const Eigen::VectorXd& NominalTau_FR,
		const TArray<FU_ExtForce>& NominalU_ExtForces,
		double Epsilon,
		Eigen::MatrixXd& OutA_FR
	);

	bool ComputeA_FR_Discrete(
		FFullSys& FullSys,
		double Dt,
		const FSysState& NominalX_Full,
		const Eigen::VectorXd& NominalTau_FR,
		const TArray<FU_ExtForce>& NominalU_ExtForces,
		double Epsilon,
		Eigen::MatrixXd& OutA_FR
	);

	void BuildA_FRInFullDisplay(
		const FStateDOFIndexMap& IndexMap,
		const Eigen::MatrixXd& A_FR,
		Eigen::MatrixXd& OutA_FullDisplay
	);

	bool ComputeB_FR_Continuous(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		const Eigen::VectorXd& NominalU_Control,
		const TArray<FU_ExtForce>& NominalU_ExtForces,
		double EpsilonU,
		Eigen::MatrixXd& OutB_FR
	);

	bool ComputeB_FR_Discrete(
		FFullSys& FullSys,
		double Dt,
		const FSysState& NominalX_Full,
		const Eigen::VectorXd& NominalU_Control,
		const TArray<FU_ExtForce>& NominalU_ExtForces,
		double EpsilonU,
		Eigen::MatrixXd& OutB_FR
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

	// Controlability

	bool AnalyzeControllability(
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& B,
		FControllabilityResult& OutResult
	) const;

	// SOLVERS

	bool SolveDARE(
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& B,
		const Eigen::MatrixXd& Q,
		const Eigen::MatrixXd& R,
		FDAREResult& OutResult
	) const;

	bool ComputeLQRGain(
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& B,
		const Eigen::MatrixXd& R,
		const Eigen::MatrixXd& P,
		Eigen::MatrixXd& OutK
	) const;

private:
	TArray<FLinearControlChannel> ControlChannels;

	Eigen::MatrixXd G_UToTau_FR;

	bool bControlAllocationBuilt = false;
	
	bool ComputeA_FullColumn_Continuous(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		const Eigen::VectorXd& NominalTau_FR,
		const TArray<FU_ExtForce>& NominalU_ExtForces,
		int32 PerturbationIndexX,
		double Epsilon,
		Eigen::VectorXd& OutA_iColumn
	);

	bool ComputeA_FullColumn_Discrete(
		FFullSys& FullSys,
		double Dt,
		const FSysState& NominalX_Full,
		const Eigen::VectorXd& NominalTau_FR,
		const TArray<FU_ExtForce>& NominalU_ExtForces,
		int32 PerturbationIndexX,
		double Epsilon,
		Eigen::VectorXd& OutA_iColumn
	);

	bool ComputeB_FullColumn_Continuous(
		FFullSys& FullSys,
		const FSysState& NominalX_Full,
		const Eigen::VectorXd& NominalU_Control,
		const TArray<FU_ExtForce>& NominalU_ExtForces,
		int32 PerturbationIndexU,
		double EpsilonU,
		Eigen::VectorXd& OutB_iColumn
	);

	bool ComputeB_FullColumn_Discrete(
		FFullSys& FullSys,
		double Dt,
		const FSysState& NominalX_Full,
		const Eigen::VectorXd& NominalU_Control,
		const TArray<FU_ExtForce>& NominalU_ExtForces,
		int32 PerturbationIndexU,
		double EpsilonU,
		Eigen::VectorXd& OutB_iColumn
	);

	bool BuildControllabilityMatrix(
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& B,
		Eigen::MatrixXd& OutControllabilityMatrix
	) const;

	bool ComputeMatrixRankSVD(
		const Eigen::MatrixXd& Matrix,
		int32& OutRank,
		double& OutRankTolerance,
		Eigen::VectorXd& OutSingularValues
	) const;
};
