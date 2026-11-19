#include "Scenario/TriplePendulum.h"

#include "Simulation/FullSys.h"
#include "Control/ControlSys.h"
#include "Elements/PendulumRod.h"


FSysState FTriplePendulum::BuildState(EStatePreset Preset)
{
	Eigen::VectorXd Q_Equilibrium = Eigen::VectorXd::Zero(8);

	// Base
	Q_Equilibrium(0) = 2.50;
	Q_Equilibrium(1) = -1.20;

	// Mass1
	Q_Equilibrium(2) = 2.50;
	Q_Equilibrium(3) = -0.40;

	// Mass2
	Q_Equilibrium(4) = 2.50;
	Q_Equilibrium(5) = 0.35;

	// Mass3
	Q_Equilibrium(6) = 2.50;
	Q_Equilibrium(7) = 1.26;

	Eigen::VectorXd Q = Q_Equilibrium;

	switch (Preset)
	{
	case VerticalEquilibrium:
		break;

	case SlightLeft:

		// Mass3
		Q(6) = Q_Equilibrium(6) - 0.01;
		Q(7) = Q_Equilibrium(7) - 0.0000549467;

		break;
	
	case ModerateLeft:

		// Mass3
		Q(6) = Q_Equilibrium(6) - 0.02;
		Q(7) = Q_Equilibrium(7) - 0.0002198068;

		break;

	default:
		checkNoEntry();
		return {};
	}

	const Eigen::VectorXd QDot = Eigen::VectorXd::Zero(8);

	return FSysState(Q, QDot);
}

void FTriplePendulum::BuildSystem(
	FFullSys& FullSys,
	FControlSys& ControlSys,
	TArray<FU_ExtForce>& U_ExtForces
)
{
	const FSysState X_Equilibrium =
		BuildState(VerticalEquilibrium);

	// -------------------------------------------------------------------------
	// Subsystem
	// -------------------------------------------------------------------------

	FSubSys& TripleInvertedPendulum =
		FullSys.CreateSubSys(TEXT("TripleInvertedPendulum"));

	// -------------------------------------------------------------------------
	// Bodies
	// -------------------------------------------------------------------------

	const int32 BaseIndex =
		FullSys.AddBody(
			FBody::SlidingMassX(
				FVector2D(
					X_Equilibrium.Q(0),
					X_Equilibrium.Q(1)
				),
				2.0
			)
		);

	const int32 Mass1Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Equilibrium.Q(2),
					X_Equilibrium.Q(3)
				),
				1.4
			)
		);

	const int32 Mass2Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Equilibrium.Q(4),
					X_Equilibrium.Q(5)
				),
				1.0
			)
		);

	const int32 Mass3Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Equilibrium.Q(6),
					X_Equilibrium.Q(7)
				),
				0.7
			)
		);

	// -------------------------------------------------------------------------
	// Rods
	// -------------------------------------------------------------------------

	const int32 Rod1Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BaseIndex,
				Mass1Index
			)
		);

	const int32 Rod2Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Mass1Index,
				Mass2Index
			)
		);

	const int32 Rod3Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Mass2Index,
				Mass3Index
			)
		);

	// -------------------------------------------------------------------------
	// Control Channels
	// -------------------------------------------------------------------------

	FLinearControlChannel Channel0(TEXT("BaseHorizontal"));
	Channel0.AddForceContribution(BaseIndex, FVector2D(1.0, 0.0));
	ControlSys.AddLinearControlChannel(MoveTemp(Channel0));

	/*
	FLinearControlChannel Channel1(TEXT("Mass2Horizontal"));
	Channel1.AddForceContribution(Mass2Index, FVector2D(1.0, 0.0));
	ControlSys.AddLinearControlChannel(MoveTemp(Channel1));
	*/

	// -------------------------------------------------------------------------
	// External force inputs
	// -------------------------------------------------------------------------

	U_ExtForces.Reset();

	U_ExtForces.Add({
		Mass1Index,
		FVector2D::ZeroVector
		});

	// -------------------------------------------------------------------------
	// Subsystem associations - Bodies
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddBody(TEXT("Base"), BaseIndex);
	TripleInvertedPendulum.AddBody(TEXT("Mass1"), Mass1Index);
	TripleInvertedPendulum.AddBody(TEXT("Mass2"), Mass2Index);
	TripleInvertedPendulum.AddBody(TEXT("Mass3"), Mass3Index);

	// -------------------------------------------------------------------------
	// Subsystem associations - Rods
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddElement(TEXT("Rod1"), Rod1Index);
	TripleInvertedPendulum.AddElement(TEXT("Rod2"), Rod2Index);
	TripleInvertedPendulum.AddElement(TEXT("Rod3"), Rod3Index);
}