#include "Control/ControlSys.h"

#include "Simulation/FullSys.h"


bool FControlSys::ComputeStateJacobianColumn(
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