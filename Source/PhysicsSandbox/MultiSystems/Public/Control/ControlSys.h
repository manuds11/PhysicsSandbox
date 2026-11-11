#pragma once

#include "Simulation/SysState.h"
#include "Simulation/FullSysTypes.h"

#include <Eigen/Dense>

class FFullSys;


// =============================================================================
// Control channel definitions
// =============================================================================

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

	explicit FLinearControlChannel(FName InName)
		: Name(InName)
	{
	}

	void AddForceContribution(int32 BodyApplicationIndex, const FVector2D& ForcePerUnitU);
};


// =============================================================================
// Controllability
// =============================================================================

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


// =============================================================================
// DARE
// =============================================================================

struct FDAREResult
{
	Eigen::MatrixXd P;

	int32 Iterations = 0;
	double Residual = 0.0;

	bool bConverged = false;
};


// =============================================================================
// LQR design
// =============================================================================

struct FLQRDesign
{
	double Dt = 0.0;

	FSysState NominalX_Full;
	Eigen::VectorXd NominalU;

	Eigen::MatrixXd T_kerToFR;

	Eigen::MatrixXd A_ker_Discrete;
	Eigen::MatrixXd B_ker_Discrete;

	FControllabilityResult Controllability;
	FDAREResult DARE;

	Eigen::MatrixXd K;

	// Debug visualization
	// -------------------------------------------------------------------------

	TArray<double> T_DebugData;
	TArray<double> A_DebugData;
	TArray<double> B_DebugData;
	TArray<double> P_DebugData;
	TArray<double> K_DebugData;

	void AdaptDataFromEigen2Debug()
	{
		auto CopyMatrixRowMajor = [](const Eigen::MatrixXd& Matrix, TArray<double>& OutDebugData)
			{
				OutDebugData.SetNum(static_cast<int32>(Matrix.rows() * Matrix.cols()));

				int32 DebugIndex = 0;

				for (Eigen::Index Row = 0; Row < Matrix.rows(); ++Row)
				{
					for (Eigen::Index Col = 0; Col < Matrix.cols(); ++Col)
					{
						OutDebugData[DebugIndex++] = Matrix(Row, Col);
					}
				}
			};

		CopyMatrixRowMajor(T_kerToFR, T_DebugData);
		CopyMatrixRowMajor(A_ker_Discrete, A_DebugData);
		CopyMatrixRowMajor(B_ker_Discrete, B_DebugData);
		CopyMatrixRowMajor(DARE.P, P_DebugData);
		CopyMatrixRowMajor(K, K_DebugData);
	}
};


// =============================================================================
// Control System
// =============================================================================

class FControlSys
{
public:
	FControlSys() = default;


	// =========================================================================
	// Configuration
	// =========================================================================

	int32 AddLinearControlChannel(FLinearControlChannel Channel);

	bool Initialize(const FFullSys& FullSys);

	int32 GetNumControlChannels() const
	{
		return ControlChannels.Num();
	}


	// =========================================================================
	// Reference
	// =========================================================================

	void SetReference(const FSysState& InX_Reference_Full, const Eigen::VectorXd& InU_Reference);


	// =========================================================================
	// LQR Initialization
	// =========================================================================

	bool InitializeLQRController(
		FFullSys& FullSys,
		double Dt,
		const TArray<FU_ExtForce>& U_ExtForces
	);


	// =========================================================================
	// Runtime Control
	// =========================================================================

	Eigen::VectorXd ComputeControlTau_FR(
		const FSysState& X_Full,
		const FStateDOFIndexMap& IndexMap
	);


	// =========================================================================
	// Debug Conversion
	// =========================================================================

	bool BuildDeltaX_FullFromZ_ker(
		const FFullSys& FullSys,
		const Eigen::VectorXd& Z_ker,
		Eigen::VectorXd& OutDeltaX_Full
	) const;


	// =========================================================================
	// Visualization Access
	// =========================================================================

	const TArray<FLinearControlChannel>& GetControlChannels() const
	{
		return ControlChannels;
	}

	const Eigen::VectorXd& GetUControl() const
	{
		return U_Control;
	}


private:

	// =========================================================================
	// Control State
	// =========================================================================

	TArray<FLinearControlChannel> ControlChannels;
	Eigen::MatrixXd G_UToTau_FR;

	FSysState X_Reference_Full;
	Eigen::VectorXd U_Reference;
	Eigen::VectorXd U_Control;

	FLQRDesign ActiveLQRModel;

	bool bControlAllocationBuilt = false;


	// =========================================================================
	// Control Allocation
	// =========================================================================

	bool BuildControlAllocationG_FRMatrix(const FFullSys& FullSys);

	Eigen::VectorXd ComputeTau_FR(const Eigen::VectorXd& InU_Control) const;


	// =========================================================================
	// LQR Design
	// =========================================================================

	bool DesignLQRController(
		FFullSys& FullSys,
		double Dt,
		const FSysState& NominalX_Full,
		const Eigen::VectorXd& NominalU,
		const TArray<FU_ExtForce>& NominalU_ExtForces,
		FLQRDesign& OutLQRModel
	);


	// =========================================================================
	// FR Linearization
	// =========================================================================

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


	// =========================================================================
	// Full-State Finite Difference Helpers
	// =========================================================================

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


	// =========================================================================
	// Constraint Tangent Space
	// =========================================================================

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

	bool BuildKernelBasis_FR(
		const Eigen::MatrixXd& H_X_FR,
		Eigen::MatrixXd& OutT_kerToFR
	) const;


	// =========================================================================
	// Controllability
	// =========================================================================

	bool AnalyzeControllability(
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& B,
		FControllabilityResult& OutResult
	) const;

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


	// =========================================================================
	// LQR Solvers
	// =========================================================================

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
};