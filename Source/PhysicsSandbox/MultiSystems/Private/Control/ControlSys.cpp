#include "Control/ControlSys.h"

#include "Simulation/FullSys.h"

bool FControlSys::ComputeA_FullColumn(
	FFullSys& FullSys,
	const FSysState& NominalX_Full,
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

	XPlusVector_Full(PerturbationIndexX) += Epsilon;
	XMinusVector_Full(PerturbationIndexX) -= Epsilon;

	const FSysState XPlus_Full = FSysState::ToStruct(XPlusVector_Full);
	const FSysState XMinus_Full = FSysState::ToStruct(XMinusVector_Full);

	FSysStateDerivative XDotPlus_Full;
	FSysStateDerivative XDotMinus_Full;

	if (!FullSys.EvaluateDynamics(XPlus_Full, XDotPlus_Full))
	{
		return false;
	}

	if (!FullSys.EvaluateDynamics(XMinus_Full, XDotMinus_Full))
	{
		return false;
	}

	OutA_iColumn =
		(XDotPlus_Full.ToVector() - XDotMinus_Full.ToVector())
		/ (2.0 * Epsilon);

	return true;
}

bool FControlSys::ComputeA_FreeRedundant(	// FR = Free-Redundant
	FFullSys& FullSys,
	const FSysState& NominalX_Full,
	double Epsilon,
	Eigen::MatrixXd& OutA_FR
)
{
	const FStateDOFIndexMap& IndexMap = FullSys.GetStateIndexMap();

	const int32 SizeX_Full = static_cast<int32>(NominalX_Full.ToVector().size());

	const int32 SizeX_FR = IndexMap.XStateFreeIndices.Num();

	check(Epsilon > 0.0);

	OutA_FR.resize(	SizeX_FR, SizeX_FR);

	for (
		int32 ColumnA_FR = 0;
		ColumnA_FR < SizeX_FR;
		++ColumnA_FR
		)
	{
		const int32 PerturbationIndexX_Full = 
			IndexMap.XStateFreeIndices[ColumnA_FR];

		Eigen::VectorXd A_iColumn_Full;

		if (!ComputeA_FullColumn(
			FullSys,
			NominalX_Full,
			PerturbationIndexX_Full,
			Epsilon,
			A_iColumn_Full
		))
		{	
			FSysStateDerivative XDotNominal_Full;
			FullSys.EvaluateDynamics(
				NominalX_Full,
				XDotNominal_Full
			);

			return false;
		}

		check(A_iColumn_Full.size() == SizeX_Full);

		for (
			int32 RowA_FR = 0;
			RowA_FR < SizeX_FR;
			++RowA_FR
			)
		{
			const int32 RowFull =
				IndexMap.XStateFreeIndices[RowA_FR];

			OutA_FR(RowA_FR, ColumnA_FR) =
				A_iColumn_Full(RowFull);
		}
	}

	FSysStateDerivative XDotNominal_Full;

	if (!FullSys.EvaluateDynamics(	// Return to Nominal state  after perturbations
		NominalX_Full,
		XDotNominal_Full
	))
	{
		return false;
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

	const int32 SizeX_FR = IndexMap.XStateFreeIndices.Num();

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
			OutA_FullDisplay(IndexMap.XStateFreeIndices[RowA_FR], IndexMap.XStateFreeIndices[ColumnA_FR]) =
				A_FR(RowA_FR, ColumnA_FR);
		}
	}
}