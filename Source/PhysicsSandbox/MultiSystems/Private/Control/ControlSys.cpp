#include "Control/ControlSys.h"

#include "Simulation/FullSys.h"
#include "Elements/PendulumRod.h"

#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <limits>

// -----------------------------------------------------------------------------
// Control Channels
// -----------------------------------------------------------------------------

int32 FControlSys::AddLinearControlChannel(
	FLinearControlChannel Channel
)
{
	check(!bControlAllocationBuilt);

	check(
		!Channel.ForceContributions.IsEmpty()
	);

	return ControlChannels.Add(
		MoveTemp(Channel)
	);
}

void FLinearControlChannel::AddForceContribution(
	int32 BodyApplicationIndex,
	const FVector2D& ForcePerUnitU
)
{
	check(
		BodyApplicationIndex
		!= INDEX_NONE
	);

	check(
		FMath::IsFinite(ForcePerUnitU.X)
		&& FMath::IsFinite(ForcePerUnitU.Y)
	);

	check(
		!ForcePerUnitU.IsNearlyZero()
	);

	FControlForceContribution Contribution;

	Contribution.BodyApplicationIndex =
		BodyApplicationIndex;

	Contribution.ForcePerUnitU =
		ForcePerUnitU;

	ForceContributions.Add(
		Contribution
	);
}

bool FControlSys::BuildControlAllocationG_FRMatrix(
	const FFullSys& FullSys
)
{
	const FStateDOFIndexMap& IndexMap = FullSys.GetStateIndexMap();

	const TArray<FBody>& Bodies = FullSys.GetBodies();

	if (IndexMap.SizeQ_FR <= 0 || ControlChannels.Num() <= 0)
	{
		return false;
	}

	G_UToTau_FR = Eigen::MatrixXd::Zero(IndexMap.SizeQ_FR, ControlChannels.Num());

	for (
		int32 ChannelIndex = 0;
		ChannelIndex < ControlChannels.Num();
		++ChannelIndex
		)
	{
		const FLinearControlChannel& Channel =
			ControlChannels[ChannelIndex];

		if (Channel.ForceContributions.IsEmpty())
		{
			return false;
		}

		for (const FControlForceContribution& Contribution : Channel.ForceContributions)
		{
			if (!Bodies.IsValidIndex(Contribution.BodyApplicationIndex))
			{
				return false;
			}

			const int32 Q_FullChannelContribXIndex = 2 * Contribution.BodyApplicationIndex;
			const int32 Q_FullChannelContribYIndex = Q_FullChannelContribXIndex + 1;

			const int32 Q_FRChannelContribXIndex = IndexMap.FindQ_FRIndex(Q_FullChannelContribXIndex);
			const int32 Q_FRChannelContribYIndex = IndexMap.FindQ_FRIndex(Q_FullChannelContribYIndex);

			if (!FMath::IsNearlyZero(Contribution.ForcePerUnitU.X) && Q_FRChannelContribXIndex == INDEX_NONE)
			{
				return false;
			}

			if (!FMath::IsNearlyZero(Contribution.ForcePerUnitU.Y) && Q_FRChannelContribYIndex == INDEX_NONE)
			{
				return false;
			}

			if (Q_FRChannelContribXIndex != INDEX_NONE)
			{
				G_UToTau_FR(Q_FRChannelContribXIndex, ChannelIndex) += Contribution.ForcePerUnitU.X;
			}

			if (Q_FRChannelContribYIndex != INDEX_NONE)
			{
				G_UToTau_FR(Q_FRChannelContribYIndex, ChannelIndex) += Contribution.ForcePerUnitU.Y;
			}
		}
	}

	if (!G_UToTau_FR.allFinite())
	{
		return false;
	}

	bControlAllocationBuilt = true;

	return true;
}

Eigen::VectorXd FControlSys::ComputeTau_FR(
	const Eigen::VectorXd& U_Control
) const
{
	check(bControlAllocationBuilt);

	check(
		U_Control.size()
		== G_UToTau_FR.cols()
	);

	check(U_Control.allFinite());

	return G_UToTau_FR * U_Control;
}

// -------------------------------------------------------------------------
// Finite Difference 
// -------------------------------------------------------------------------

bool FControlSys::ComputeA_FR_Continuous(	// FR = Free-Redundant
	FFullSys& FullSys,
	const FSysState& NominalX_Full,
	const Eigen::VectorXd& NominalTau_FR,
	const TArray<FU_ExtForce>& NominalU_ExtForces,
	double Epsilon,
	Eigen::MatrixXd& OutA_FR
)
{
	const FStateDOFIndexMap& IndexMap = FullSys.GetStateIndexMap();

	const int32 SizeX_Full = static_cast<int32>(NominalX_Full.ToVector().size());
	const int32 SizeX_FR = IndexMap.X_FRToFullIndices.Num();

	check(Epsilon > 0.0);

	OutA_FR.resize(SizeX_FR, SizeX_FR);

	for (int32 ColumnA_FR = 0; ColumnA_FR < SizeX_FR; ++ColumnA_FR)
	{
		const int32 PerturbationIndexX_Full = IndexMap.X_FRToFullIndices[ColumnA_FR];

		Eigen::VectorXd A_iColumn_Full;

		if (!ComputeA_FullColumn_Continuous(
			FullSys,
			NominalX_Full,
			NominalTau_FR,
			NominalU_ExtForces,
			PerturbationIndexX_Full,
			Epsilon,
			A_iColumn_Full
		))
		{
			return false;
		}

		check(A_iColumn_Full.size() == SizeX_Full);

		for (int32 RowA_FR = 0; RowA_FR < SizeX_FR; ++RowA_FR)
		{
			const int32 RowIndex_Full = IndexMap.X_FRToFullIndices[RowA_FR];

			OutA_FR(RowA_FR, ColumnA_FR) = A_iColumn_Full(RowIndex_Full);
		}
	}

	return true;
}

bool FControlSys::ComputeA_FR_Discrete(
	FFullSys& FullSys,
	double Dt,
	const FSysState& NominalX_Full,
	const Eigen::VectorXd& NominalTau_FR,
	const TArray<FU_ExtForce>& NominalU_ExtForces,
	double Epsilon,
	Eigen::MatrixXd& OutA_FR
)
{
	const FStateDOFIndexMap& IndexMap = FullSys.GetStateIndexMap();

	const int32 SizeX_Full = static_cast<int32>(NominalX_Full.ToVector().size());
	const int32 SizeX_FR = IndexMap.X_FRToFullIndices.Num();

	check(Dt > 0.0);
	check(Epsilon > 0.0);

	OutA_FR.resize(SizeX_FR, SizeX_FR);

	for (int32 ColumnA_FR = 0; ColumnA_FR < SizeX_FR; ++ColumnA_FR)
	{
		const int32 PerturbationIndexX_Full = IndexMap.X_FRToFullIndices[ColumnA_FR];

		Eigen::VectorXd A_iColumn_Full;

		if (!ComputeA_FullColumn_Discrete(
			FullSys,
			Dt,
			NominalX_Full,
			NominalTau_FR,
			NominalU_ExtForces,
			PerturbationIndexX_Full,
			Epsilon,
			A_iColumn_Full
		))
		{
			return false;
		}

		check(A_iColumn_Full.size() == SizeX_Full);

		for (int32 RowA_FR = 0; RowA_FR < SizeX_FR; ++RowA_FR)
		{
			const int32 RowIndex_Full = IndexMap.X_FRToFullIndices[RowA_FR];
			OutA_FR(RowA_FR, ColumnA_FR) = A_iColumn_Full(RowIndex_Full);
		}
	}

	return true;
}

void FControlSys::BuildA_FRInFullDisplay(
	const FStateDOFIndexMap& IndexMap,
	const Eigen::MatrixXd& A_FR,
	Eigen::MatrixXd& OutA_FullDisplay
)
{
	const int32 SizeX_Full = 2 * IndexMap.SizeQ_Full;

	const int32 SizeX_FR = IndexMap.X_FRToFullIndices.Num();

	check(A_FR.rows() == SizeX_FR);

	check(A_FR.cols() == SizeX_FR);

	OutA_FullDisplay =
		Eigen::MatrixXd::Zero(
			SizeX_Full,
			SizeX_Full
		);

	for (
		int32 RowA_FR = 0;
		RowA_FR < SizeX_FR;
		++RowA_FR
		)
	{
		for (
			int32 ColumnA_FR = 0;
			ColumnA_FR < SizeX_FR;
			++ColumnA_FR
			)
		{
			OutA_FullDisplay(IndexMap.X_FRToFullIndices[RowA_FR], IndexMap.X_FRToFullIndices[ColumnA_FR]) =
				A_FR(RowA_FR, ColumnA_FR);
		}
	}
}

bool FControlSys::ComputeB_FR_Continuous(
	FFullSys& FullSys,
	const FSysState& NominalX_Full,
	const Eigen::VectorXd& NominalU_Control,
	const TArray<FU_ExtForce>& NominalU_ExtForces,
	double EpsilonU,
	Eigen::MatrixXd& OutB_FR
)
{
	if (
		EpsilonU <= 0.0
		|| NominalU_Control.size() != GetNumControlChannels()
		|| !NominalU_Control.allFinite()
		)
	{
		return false;
	}

	const FStateDOFIndexMap& IndexMap = FullSys.GetStateIndexMap();

	const int32 SizeX_Full =
		static_cast<int32>(NominalX_Full.ToVector().size());

	const int32 SizeX_FR =
		IndexMap.X_FRToFullIndices.Num();

	const int32 NumControlChannels =
		GetNumControlChannels();

	OutB_FR.resize(SizeX_FR, NumControlChannels);

	for (int32 ColumnB_FR = 0; ColumnB_FR < NumControlChannels; ++ColumnB_FR)
	{
		Eigen::VectorXd B_iColumn_Full;

		if (!ComputeB_FullColumn_Continuous(
			FullSys,
			NominalX_Full,
			NominalU_Control,
			NominalU_ExtForces,
			ColumnB_FR,
			EpsilonU,
			B_iColumn_Full
		))
		{
			return false;
		}

		if (B_iColumn_Full.size() != SizeX_Full)
		{
			return false;
		}

		for (int32 RowB_FR = 0; RowB_FR < SizeX_FR; ++RowB_FR)
		{
			const int32 RowIndex_Full =
				IndexMap.X_FRToFullIndices[RowB_FR];

			OutB_FR(RowB_FR, ColumnB_FR) =
				B_iColumn_Full(RowIndex_Full);
		}
	}

	return OutB_FR.allFinite();
}

bool FControlSys::ComputeB_FR_Discrete(
	FFullSys& FullSys,
	double Dt,
	const FSysState& NominalX_Full,
	const Eigen::VectorXd& NominalU_Control,
	const TArray<FU_ExtForce>& NominalU_ExtForces,
	double EpsilonU,
	Eigen::MatrixXd& OutB_FR
)
{
	const FStateDOFIndexMap& IndexMap = FullSys.GetStateIndexMap();

	const int32 SizeX_Full = static_cast<int32>(NominalX_Full.ToVector().size());
	const int32 SizeX_FR = IndexMap.X_FRToFullIndices.Num();
	const int32 NumControlChannels = GetNumControlChannels();

	check(Dt > 0.0);
	check(EpsilonU > 0.0);
	check(NominalU_Control.size() == NumControlChannels);

	OutB_FR.resize(SizeX_FR, NumControlChannels);

	for (int32 ColumnB_FR = 0; ColumnB_FR < NumControlChannels; ++ColumnB_FR)
	{
		Eigen::VectorXd B_iColumn_Full;

		if (!ComputeB_FullColumn_Discrete(
			FullSys,
			Dt,
			NominalX_Full,
			NominalU_Control,
			NominalU_ExtForces,
			ColumnB_FR,
			EpsilonU,
			B_iColumn_Full
		))
		{
			return false;
		}

		check(B_iColumn_Full.size() == SizeX_Full);

		for (int32 RowB_FR = 0; RowB_FR < SizeX_FR; ++RowB_FR)
		{
			const int32 RowIndex_Full = IndexMap.X_FRToFullIndices[RowB_FR];
			OutB_FR(RowB_FR, ColumnB_FR) = B_iColumn_Full(RowIndex_Full);
		}
	}

	return true;
}

// -------------------------------------------------------------------------
// Kernel operations
// -------------------------------------------------------------------------

bool FControlSys::BuildKernelBasis_FR(
	const Eigen::MatrixXd& H_X_FR,
	Eigen::MatrixXd& OutT_kerToFR
) const
{
	if (
		H_X_FR.rows() <= 0
		|| H_X_FR.cols() <= 0
		|| !H_X_FR.allFinite()
		)
	{
		OutT_kerToFR.resize(0, 0);
		return false;
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> SVD(H_X_FR, Eigen::ComputeFullV);

	const Eigen::VectorXd& SingularValues = SVD.singularValues();

	const double MaxSingularValue = SingularValues.maxCoeff();

	const double Tolerance =
		static_cast<double>(FMath::Max(H_X_FR.rows(), H_X_FR.cols()))
		* std::numeric_limits<double>::epsilon()
		* MaxSingularValue;

	int32 Rank = 0;

	for (Eigen::Index i = 0; i < SingularValues.size(); ++i)
	{
		if (SingularValues(i) > Tolerance)
		{
			++Rank;
		}
	}

	const int32 KernelDimension =
		static_cast<int32>(H_X_FR.cols()) - Rank;

	OutT_kerToFR =
		SVD.matrixV().rightCols(KernelDimension);

	return true;
}

bool FControlSys::BuildJandJDot_FR(
	const FFullSys& FullSys,
	Eigen::MatrixXd& OutJ_FR,
	Eigen::MatrixXd& OutJDot_FR
) const
{
	const TArray<FPendulumRod*>& Rods = FullSys.GetRodSystemArray();
	const FStateDOFIndexMap& IndexMap = FullSys.GetStateIndexMap();

	const int32 NumRods = Rods.Num();
	const int32 SizeQ_FR = IndexMap.Q_FRToFullIndices.Num();

	OutJ_FR = Eigen::MatrixXd::Zero(NumRods, SizeQ_FR);
	OutJDot_FR = Eigen::MatrixXd::Zero(NumRods, SizeQ_FR);

	for (int32 Row = 0; Row < NumRods; ++Row)
	{
		const FPendulumRod* Rod = Rods[Row];
		check(Rod);

		int32 PivotIndex = INDEX_NONE;
		int32 BobIndex = INDEX_NONE;
		Rod->GetConnectedBodies(PivotIndex, BobIndex);

		const FRodJacobian& Jacobian = Rod->GetRodJacobian();

		for (int32 Column_FR = 0; Column_FR < SizeQ_FR; ++Column_FR)
		{
			const int32 QIndex_Full = IndexMap.Q_FRToFullIndices[Column_FR];
			const int32 BodyIndex = QIndex_Full / 2; 
			const bool bXAxis = (QIndex_Full % 2) == 0;

			if (BodyIndex == PivotIndex)
			{
				OutJ_FR(Row, Column_FR) = bXAxis ? Jacobian.JPivot.X : Jacobian.JPivot.Y;
				OutJDot_FR(Row, Column_FR) = bXAxis ? Jacobian.JDotPivot.X : Jacobian.JDotPivot.Y;
			}
			else if (BodyIndex == BobIndex)
			{
				OutJ_FR(Row, Column_FR) = bXAxis ? Jacobian.JBob.X : Jacobian.JBob.Y;
				OutJDot_FR(Row, Column_FR) = bXAxis ? Jacobian.JDotBob.X : Jacobian.JDotBob.Y;
			}
		}
	}

	return true;
}

bool FControlSys::BuildH_FR(
	const Eigen::MatrixXd& J_FR,
	const Eigen::MatrixXd& JDot_FR,
	Eigen::MatrixXd& OutH_X_FR
) const
{
	if (
		J_FR.rows() != JDot_FR.rows()
		|| J_FR.cols() != JDot_FR.cols()
		|| !J_FR.allFinite()
		|| !JDot_FR.allFinite()
		)
	{
		OutH_X_FR.resize(0, 0);
		return false;
	}

	const Eigen::Index NumRods = J_FR.rows();
	const Eigen::Index SizeQ_FR = J_FR.cols();

	OutH_X_FR = Eigen::MatrixXd::Zero(
		2 * NumRods,
		2 * SizeQ_FR
	);

	OutH_X_FR.block(0, 0, NumRods, SizeQ_FR) = J_FR;
	OutH_X_FR.block(NumRods, 0, NumRods, SizeQ_FR) = JDot_FR;
	OutH_X_FR.block(NumRods, SizeQ_FR, NumRods, SizeQ_FR) = J_FR;

	return true;
}

// -------------------------------------------------------------------------
// Vector Transformations
// -------------------------------------------------------------------------

Eigen::VectorXd FControlSys::TransformDeltaX_ker2FR(
	const Eigen::MatrixXd& T_kerToFR,
	const Eigen::VectorXd& DeltaX_ker
) const
{
	check(T_kerToFR.cols() == DeltaX_ker.size());
	check(T_kerToFR.allFinite());
	check(DeltaX_ker.allFinite());

	return T_kerToFR * DeltaX_ker;
}

Eigen::VectorXd FControlSys::ExpandDeltaX_FRToFull(
	const FStateDOFIndexMap& IndexMap,
	const Eigen::VectorXd& DeltaX_FR
) const
{
	const int32 SizeX_FR = IndexMap.X_FRToFullIndices.Num();
	const int32 SizeX_Full = 2 * IndexMap.SizeQ_Full;

	check(DeltaX_FR.size() == SizeX_FR);
	check(DeltaX_FR.allFinite());

	Eigen::VectorXd DeltaX_Full = Eigen::VectorXd::Zero(SizeX_Full);

	for (int32 Index_FR = 0; Index_FR < SizeX_FR; ++Index_FR)
	{
		const int32 Index_Full = IndexMap.X_FRToFullIndices[Index_FR];
		DeltaX_Full(Index_Full) = DeltaX_FR(Index_FR);
	}

	return DeltaX_Full;
}

// -----------------------------------------------------------------------------
// Controlability
// -----------------------------------------------------------------------------

bool FControlSys::AnalyzeControllability(
	const Eigen::MatrixXd& A,
	const Eigen::MatrixXd& B,
	FControllabilityResult& OutResult
) const
{
	OutResult = FControllabilityResult{};

	const int32 StateDimension = static_cast<int32>(A.rows());
	const int32 ControlDimension = static_cast<int32>(B.cols());

	if (
		StateDimension <= 0
		|| ControlDimension <= 0
		|| A.cols() != StateDimension
		|| B.rows() != StateDimension
		|| !A.allFinite()
		|| !B.allFinite()
		)
	{
		return false;
	}

	OutResult.StateDimension = StateDimension;
	OutResult.ControlDimension = ControlDimension;

	if (!BuildControllabilityMatrix(
		A,
		B,
		OutResult.ControllabilityMatrix
	))
	{
		return false;
	}

	if (!ComputeMatrixRankSVD(
		OutResult.ControllabilityMatrix,
		OutResult.Rank,
		OutResult.RankTolerance,
		OutResult.SingularValues
	))
	{
		return false;
	}

	const double MaxSingularValue =
		OutResult.SingularValues.maxCoeff();

	const double MinSingularValue =
		OutResult.SingularValues.minCoeff();

	OutResult.RelativeMinSingularValue =
		MaxSingularValue > 0.0
		? MinSingularValue / MaxSingularValue
		: 0.0;

	OutResult.bIsControllable =
		OutResult.Rank == StateDimension;

	return true;
}

// -----------------------------------------------------------------------------
// LQR Model
// -----------------------------------------------------------------------------

bool FControlSys::DesignLQRController(
	FFullSys& FullSys,
	double Dt,
	const FSysState& NominalX_Full,
	const Eigen::VectorXd& NominalU,
	const TArray<FU_ExtForce>& NominalU_ExtForces,
	FLQRDesign& OutLQRModel
)
{
	if (
		Dt <= 0.0
		|| NominalU.size() != GetNumControlChannels()
		|| !NominalU.allFinite()
		)
	{
		return false;
	}

	FLQRDesign NewLQRModel;

	NewLQRModel.Dt = Dt;
	NewLQRModel.NominalX_Full = NominalX_Full;
	NewLQRModel.NominalU = NominalU;

	const FSysState FullSysX_BeforeDesign = FullSys.BodiesToFullStateStruct();
	// Set FullSys to the nominal state used for the LQR design.
	// This guarantees that J, JDot, H and T are evaluated
	// at the same operating point as A and B.
	FullSys.SetXState2Bodies(NominalX_Full);
	FullSys.SyncRodStates();


	const FStateDOFIndexMap& IndexMap = FullSys.GetStateIndexMap();

	const int32 SizeQ_FR = IndexMap.SizeQ_FR;
	const int32 SizeX_FR = IndexMap.X_FRToFullIndices.Num();
	const int32 NumControlChannels = GetNumControlChannels();

	const Eigen::VectorXd Tau_Nominal_FR =
		ComputeTau_FR(NominalU);

	// Constraint tangent space
	// -------------------------------------------------------------------------

	Eigen::MatrixXd J_FR;
	Eigen::MatrixXd JDot_FR;
	Eigen::MatrixXd H_X_FR;

	if (!BuildJandJDot_FR(FullSys, J_FR, JDot_FR))
	{
		FullSys.SetXState2Bodies(FullSysX_BeforeDesign);
		FullSys.SyncRodStates();
		return false;
	}

	if (!BuildH_FR(J_FR, JDot_FR, H_X_FR))
	{
		FullSys.SetXState2Bodies(FullSysX_BeforeDesign);
		FullSys.SyncRodStates();
		return false;
	}

	if (!BuildKernelBasis_FR(H_X_FR, NewLQRModel.T_kerToFR))
	{
		FullSys.SetXState2Bodies(FullSysX_BeforeDesign);
		FullSys.SyncRodStates();
		return false;
	}

	// Discrete FR linearization
	// -------------------------------------------------------------------------

	constexpr double LinearizationEpsilonX = 1.0e-5;
	constexpr double LinearizationEpsilonU = 1.0e-5;

	Eigen::MatrixXd A_FR_Discrete;
	Eigen::MatrixXd B_FR_Discrete;

	if (!ComputeA_FR_Discrete(
		FullSys,
		Dt,
		NominalX_Full,
		Tau_Nominal_FR,
		NominalU_ExtForces,
		LinearizationEpsilonX,
		A_FR_Discrete
	))
	{
		FullSys.SetXState2Bodies(FullSysX_BeforeDesign);
		FullSys.SyncRodStates();
		return false;
	}

	if (!ComputeB_FR_Discrete(
		FullSys,
		Dt,
		NominalX_Full,
		NominalU,
		NominalU_ExtForces,
		LinearizationEpsilonU,
		B_FR_Discrete
	))
	{
		FullSys.SetXState2Bodies(FullSysX_BeforeDesign);
		FullSys.SyncRodStates();
		return false;
	}

	// Restore Bodies to original state before perturbations from A, B computation 
	// -------------------------------------------------------------------------

	FullSys.SetXState2Bodies(FullSysX_BeforeDesign);
	FullSys.SyncRodStates();

	// FR -> ker reduced model
	// -------------------------------------------------------------------------

	NewLQRModel.A_ker_Discrete =
		NewLQRModel.T_kerToFR.transpose() * A_FR_Discrete * NewLQRModel.T_kerToFR;

	NewLQRModel.B_ker_Discrete =
		NewLQRModel.T_kerToFR.transpose() * B_FR_Discrete;

	// Controllability
	// -------------------------------------------------------------------------

	if (!AnalyzeControllability(
		NewLQRModel.A_ker_Discrete,
		NewLQRModel.B_ker_Discrete,
		NewLQRModel.Controllability
	))
	{
		return false;
	}

	// LQR cost definition
	// -------------------------------------------------------------------------

	constexpr double PositionStateWeight = 10.0;
	constexpr double VelocityStateWeight = 1.0;
	constexpr double ControlWeight = 1.0;

	Eigen::MatrixXd Q_FR = Eigen::MatrixXd::Zero(SizeX_FR, SizeX_FR);

	for (int32 QIndex_FR = 0; QIndex_FR < SizeQ_FR; ++QIndex_FR)
	{
		Q_FR(QIndex_FR, QIndex_FR) = PositionStateWeight;

		const int32 QDotIndex_FR =
			SizeQ_FR + QIndex_FR;

		Q_FR(QDotIndex_FR, QDotIndex_FR) = VelocityStateWeight;
	}

	const Eigen::MatrixXd Q_ker =
		NewLQRModel.T_kerToFR.transpose() * Q_FR * NewLQRModel.T_kerToFR;

	const Eigen::MatrixXd R =
		ControlWeight * Eigen::MatrixXd::Identity(NumControlChannels, NumControlChannels);

	// DARE
	// -------------------------------------------------------------------------

	if (!SolveDARE(
		NewLQRModel.A_ker_Discrete,
		NewLQRModel.B_ker_Discrete,
		Q_ker,
		R,
		NewLQRModel.DARE
	))
	{
		return false;
	}

	// LQR gain
	// -------------------------------------------------------------------------

	if (!ComputeLQRGain(
		NewLQRModel.A_ker_Discrete,
		NewLQRModel.B_ker_Discrete,
		R,
		NewLQRModel.DARE.P,
		NewLQRModel.K
	))
	{
		return false;
	}

	OutLQRModel = MoveTemp(NewLQRModel);
	OutLQRModel.AdaptDataFromEigen2Debug();

	return true;
}

// -----------------------------------------------------------------------------
// Discrete Algebraic Riccati Equation (DARE) Solver
// -----------------------------------------------------------------------------

bool FControlSys::SolveDARE(
	const Eigen::MatrixXd& A,
	const Eigen::MatrixXd& B,
	const Eigen::MatrixXd& Q,
	const Eigen::MatrixXd& R,
	FDAREResult& OutResult
) const
{
	/*
	* Discrete Algebraic Riccati Equation (DARE):
	 *
	 * P = Q + A^T P A
	 *     - A^T P B (R + B^T P B)^-1 B^T P A
	 *
	 * The inverse is not formed explicitly.
	 * Instead, the linear system
	 *
	 * (R + B^T P B) K = B^T P A
	 *
	 * is solved at each iteration.
	 */


	OutResult = FDAREResult{};

	const int32 StateDimension = static_cast<int32>(A.rows());
	const int32 ControlDimension = static_cast<int32>(B.cols());

	if (
		StateDimension <= 0
		|| ControlDimension <= 0
		|| A.cols() != StateDimension
		|| B.rows() != StateDimension
		|| Q.rows() != StateDimension
		|| Q.cols() != StateDimension
		|| R.rows() != ControlDimension
		|| R.cols() != ControlDimension
		|| !A.allFinite()
		|| !B.allFinite()
		|| !Q.allFinite()
		|| !R.allFinite()
		)
	{
		return false;
	}

	constexpr int32 MaxIterations = 50000;
	constexpr double ConvergenceTolerance = 1.0e-12;

	Eigen::MatrixXd P = Q;

	for (int32 Iteration = 0; Iteration < MaxIterations; ++Iteration)
	{
		const Eigen::MatrixXd S = R + B.transpose() * P * B;
		const Eigen::MatrixXd BTPA = B.transpose() * P * A;

		Eigen::LDLT<Eigen::MatrixXd> LDLT(S);

		if (LDLT.info() != Eigen::Success)
		{
			return false;
		}

		const Eigen::MatrixXd GainTerm = LDLT.solve(BTPA);

		if (LDLT.info() != Eigen::Success || !GainTerm.allFinite())
		{
			return false;
		}

		Eigen::MatrixXd PNext = Q + A.transpose() * P * A - A.transpose() * P * B * GainTerm;

		PNext = (0.5 * (PNext + PNext.transpose())).eval();

		if (!PNext.allFinite())
		{
			return false;
		}

		const double IterationError = (PNext - P).norm();

		P = MoveTemp(PNext);

		OutResult.Iterations = Iteration + 1;

		if (IterationError <= ConvergenceTolerance * FMath::Max(1.0, P.norm()))
		{
			OutResult.bConverged = true;
			break;
		}
	}

	if (!OutResult.bConverged)
	{
		return false;
	}

	const Eigen::MatrixXd S = R + B.transpose() * P * B;
	const Eigen::MatrixXd BTPA = B.transpose() * P * A;

	Eigen::LDLT<Eigen::MatrixXd> LDLT(S);

	if (LDLT.info() != Eigen::Success)
	{
		return false;
	}

	const Eigen::MatrixXd GainTerm = LDLT.solve(BTPA);

	if (LDLT.info() != Eigen::Success || !GainTerm.allFinite())
	{
		return false;
	}

	const Eigen::MatrixXd RiccatiP = Q + A.transpose() * P * A - A.transpose() * P * B * GainTerm;

	OutResult.P = MoveTemp(P);
	OutResult.Residual = (OutResult.P - RiccatiP).norm();

	return true;
}

bool FControlSys::ComputeLQRGain(
	const Eigen::MatrixXd& A,
	const Eigen::MatrixXd& B,
	const Eigen::MatrixXd& R,
	const Eigen::MatrixXd& P,
	Eigen::MatrixXd& OutK
) const
{
	/*
	 * Discrete LQR gain:
	 *
	 * K = (R + B^T P B)^-1 B^T P A
	 *
	 * The inverse is not formed explicitly.
	 * Instead, the linear system
	 *
	 * (R + B^T P B) K = B^T P A
	 *
	 * is solved.
	 */

	const int32 StateDimension = static_cast<int32>(A.rows());
	const int32 ControlDimension = static_cast<int32>(B.cols());

	if (
		StateDimension <= 0
		|| ControlDimension <= 0
		|| A.cols() != StateDimension
		|| B.rows() != StateDimension
		|| P.rows() != StateDimension
		|| P.cols() != StateDimension
		|| R.rows() != ControlDimension
		|| R.cols() != ControlDimension
		|| !A.allFinite()
		|| !B.allFinite()
		|| !P.allFinite()
		|| !R.allFinite()
		)
	{
		return false;
	}

	const Eigen::MatrixXd S =
		R + B.transpose() * P * B;

	const Eigen::MatrixXd BTPA =
		B.transpose() * P * A;

	Eigen::LDLT<Eigen::MatrixXd> LDLT(S);

	if (LDLT.info() != Eigen::Success)
	{
		return false;
	}

	OutK = LDLT.solve(BTPA);

	if (LDLT.info() != Eigen::Success || !OutK.allFinite())
	{
		return false;
	}

	return true;
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

bool FControlSys::ComputeA_FullColumn_Continuous(
	FFullSys& FullSys,
	const FSysState& NominalX_Full,
	const Eigen::VectorXd& NominalTau_FR,
	const TArray<FU_ExtForce>& NominalU_ExtForces,
	int32 PerturbationIndexX,
	double Epsilon,
	Eigen::VectorXd& OutA_iColumn
)
{
	const Eigen::VectorXd NominalXVector_Full = NominalX_Full.ToVector();

	const int32 NumStates = static_cast<int32>(NominalXVector_Full.size());

	check(PerturbationIndexX >= 0);
	check(PerturbationIndexX < NumStates);
	check(Epsilon > 0.0);

	Eigen::VectorXd XPlusVector_Full = NominalXVector_Full;
	Eigen::VectorXd XMinusVector_Full = NominalXVector_Full;

	XPlusVector_Full(PerturbationIndexX) += Epsilon;   // e_1 = (0, 1, 0, 0, ...) Thus, 1 * Epsilon = Epsilon
	XMinusVector_Full(PerturbationIndexX) -= Epsilon;

	const FSysState XPlus_Full = FSysState::ToStruct(XPlusVector_Full);
	const FSysState XMinus_Full = FSysState::ToStruct(XMinusVector_Full);

	FSysStateDerivative XDotPlus_Full;
	FSysStateDerivative XDotMinus_Full;

	if (!FullSys.EvaluateXDot(XPlus_Full, NominalTau_FR, NominalU_ExtForces, XDotPlus_Full))
	{
		return false;
	}

	if (!FullSys.EvaluateXDot(XMinus_Full, NominalTau_FR, NominalU_ExtForces, XDotMinus_Full))
	{
		return false;
	}

	OutA_iColumn =
		(XDotPlus_Full.ToVector() - XDotMinus_Full.ToVector())
		/ (2.0 * Epsilon);

	return true;
}

bool FControlSys::ComputeA_FullColumn_Discrete(
	FFullSys& FullSys,
	double Dt,
	const FSysState& NominalX_Full,
	const Eigen::VectorXd& NominalTau_FR,
	const TArray<FU_ExtForce>& NominalU_ExtForces,
	int32 PerturbationIndexX,
	double Epsilon,
	Eigen::VectorXd& OutA_iColumn
)
{
	const Eigen::VectorXd NominalXVector_Full = NominalX_Full.ToVector();

	const int32 NumStates = static_cast<int32>(NominalXVector_Full.size());

	check(Dt > 0.0);
	check(PerturbationIndexX >= 0);
	check(PerturbationIndexX < NumStates);
	check(Epsilon > 0.0);

	Eigen::VectorXd XPlusVector_Full = NominalXVector_Full;
	Eigen::VectorXd XMinusVector_Full = NominalXVector_Full;

	XPlusVector_Full(PerturbationIndexX) += Epsilon;
	XMinusVector_Full(PerturbationIndexX) -= Epsilon;

	const FSysState XPlus_Full = FSysState::ToStruct(XPlusVector_Full);
	const FSysState XMinus_Full = FSysState::ToStruct(XMinusVector_Full);

	FSysState XNextPlus_Full;
	FSysState XNextMinus_Full;

	if (!FullSys.EvaluateNextXState(
		Dt,
		XPlus_Full,
		NominalTau_FR,
		NominalU_ExtForces,
		XNextPlus_Full
	))
	{
		return false;
	}

	if (!FullSys.EvaluateNextXState(
		Dt,
		XMinus_Full,
		NominalTau_FR,
		NominalU_ExtForces,
		XNextMinus_Full
	))
	{
		return false;
	}

	OutA_iColumn =
		(XNextPlus_Full.ToVector() - XNextMinus_Full.ToVector())
		/ (2.0 * Epsilon);

	return true;
}

bool FControlSys::ComputeB_FullColumn_Continuous(
	FFullSys& FullSys,
	const FSysState& NominalX_Full,
	const Eigen::VectorXd& NominalU_Control,
	const TArray<FU_ExtForce>& NominalU_ExtForces,
	int32 PerturbationIndexU,
	double EpsilonU,
	Eigen::VectorXd& OutB_iColumn
)
{
	if (
		EpsilonU <= 0.0
		|| PerturbationIndexU < 0
		|| PerturbationIndexU >= NominalU_Control.size()
		)
	{
		return false;
	}

	Eigen::VectorXd UPlus_Control =
		NominalU_Control;

	Eigen::VectorXd UMinus_Control =
		NominalU_Control;

	UPlus_Control(PerturbationIndexU) += EpsilonU;
	UMinus_Control(PerturbationIndexU) -= EpsilonU;

	const Eigen::VectorXd TauPlus_FR =
		ComputeTau_FR(UPlus_Control);

	const Eigen::VectorXd TauMinus_FR =
		ComputeTau_FR(UMinus_Control);

	FSysStateDerivative XDotPlus_Full;
	FSysStateDerivative XDotMinus_Full;

	if (!FullSys.EvaluateXDot(
		NominalX_Full,
		TauPlus_FR,
		NominalU_ExtForces,
		XDotPlus_Full
	))
	{
		return false;
	}

	if (!FullSys.EvaluateXDot(
		NominalX_Full,
		TauMinus_FR,
		NominalU_ExtForces,
		XDotMinus_Full
	))
	{
		return false;
	}

	const Eigen::VectorXd XDotPlusVector_Full =
		XDotPlus_Full.ToVector();

	const Eigen::VectorXd XDotMinusVector_Full =
		XDotMinus_Full.ToVector();

	if (XDotPlusVector_Full.size() != XDotMinusVector_Full.size())
	{
		return false;
	}

	OutB_iColumn =
		(XDotPlusVector_Full - XDotMinusVector_Full)
		/ (2.0 * EpsilonU);

	return OutB_iColumn.allFinite();
}

bool FControlSys::ComputeB_FullColumn_Discrete(
	FFullSys& FullSys,
	double Dt,
	const FSysState& NominalX_Full,
	const Eigen::VectorXd& NominalU_Control,
	const TArray<FU_ExtForce>& NominalU_ExtForces,
	int32 PerturbationIndexU,
	double EpsilonU,
	Eigen::VectorXd& OutB_iColumn
)
{
	const int32 NumControlChannels = GetNumControlChannels();

	check(Dt > 0.0);
	check(EpsilonU > 0.0);
	check(NominalU_Control.size() == NumControlChannels);
	check(PerturbationIndexU >= 0);
	check(PerturbationIndexU < NumControlChannels);

	Eigen::VectorXd UPlus = NominalU_Control;
	Eigen::VectorXd UMinus = NominalU_Control;

	UPlus(PerturbationIndexU) += EpsilonU;
	UMinus(PerturbationIndexU) -= EpsilonU;

	const Eigen::VectorXd TauPlus_FR = ComputeTau_FR(UPlus);
	const Eigen::VectorXd TauMinus_FR = ComputeTau_FR(UMinus);

	FSysState XNextPlus_Full;
	FSysState XNextMinus_Full;

	if (!FullSys.EvaluateNextXState(
		Dt,
		NominalX_Full,
		TauPlus_FR,
		NominalU_ExtForces,
		XNextPlus_Full
	))
	{
		return false;
	}

	if (!FullSys.EvaluateNextXState(
		Dt,
		NominalX_Full,
		TauMinus_FR,
		NominalU_ExtForces,
		XNextMinus_Full
	))
	{
		return false;
	}

	OutB_iColumn =
		(XNextPlus_Full.ToVector() - XNextMinus_Full.ToVector())
		/ (2.0 * EpsilonU);

	return true;
}

bool FControlSys::BuildControllabilityMatrix(
	const Eigen::MatrixXd& A,
	const Eigen::MatrixXd& B,
	Eigen::MatrixXd& OutControllabilityMatrix
) const
{
	const int32 StateDimension = static_cast<int32>(A.rows());
	const int32 ControlDimension = static_cast<int32>(B.cols());

	if (
		StateDimension <= 0
		|| ControlDimension <= 0
		|| A.cols() != StateDimension
		|| B.rows() != StateDimension
		|| !A.allFinite()
		|| !B.allFinite()
		)
	{
		return false;
	}

	OutControllabilityMatrix.resize(
		StateDimension,
		StateDimension * ControlDimension
	);

	Eigen::MatrixXd APower =
		Eigen::MatrixXd::Identity(StateDimension, StateDimension);

	for (int32 PowerIndex = 0; PowerIndex < StateDimension; ++PowerIndex)
	{
		OutControllabilityMatrix.block(
			0,
			PowerIndex * ControlDimension,
			StateDimension,
			ControlDimension
		) = APower * B;

		APower = APower * A;
	}

	return true;
}

bool FControlSys::ComputeMatrixRankSVD(
	const Eigen::MatrixXd& Matrix,
	int32& OutRank,
	double& OutRankTolerance,
	Eigen::VectorXd& OutSingularValues
) const
{
	if (
		Matrix.rows() <= 0
		|| Matrix.cols() <= 0
		|| !Matrix.allFinite()
		)
	{
		return false;
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> SVD(
		Matrix,
		Eigen::ComputeThinU | Eigen::ComputeThinV
	);

	OutSingularValues = SVD.singularValues();

	if (OutSingularValues.size() <= 0)
	{
		return false;
	}

	const double MaxSingularValue =
		OutSingularValues.maxCoeff();

	const int32 MaxMatrixDimension =
		FMath::Max(
			static_cast<int32>(Matrix.rows()),
			static_cast<int32>(Matrix.cols())
		);

	OutRankTolerance =
		static_cast<double>(MaxMatrixDimension)
		* std::numeric_limits<double>::epsilon()
		* MaxSingularValue;

	OutRank = 0;

	for (Eigen::Index SingularValueIndex = 0; SingularValueIndex < OutSingularValues.size(); ++SingularValueIndex)
	{
		if (OutSingularValues(SingularValueIndex) > OutRankTolerance)
		{
			++OutRank;
		}
	}

	return true;
}