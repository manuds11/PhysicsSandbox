#include "Scenario/TriplePendulum.h"

#include "Simulation/FullSys.h"
#include "Control/ControlSys.h"
#include "Elements/PendulumRod.h"


FSysState FTriplePendulum::BuildStatePreset(EStatePreset Preset)
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

	case StrongLeft:

		// Mass3
		Q(6) = Q_Equilibrium(6) - 0.05;          // Delta X = -0.05 m
		Q(7) = Q_Equilibrium(7) - 0.0013746647;  // Preserve Rod3 length

		break;

	case StrongRight:

		// Mass3
		Q(6) = Q_Equilibrium(6) + 0.075;         // Delta X = +0.075 m
		Q(7) = Q_Equilibrium(7) - 0.0030959257;  // Preserve Rod3 length

		break;

	case ExtremeRight:

		// Mass3
		Q(6) = Q_Equilibrium(6) + 0.10;          // Delta X = +0.10 m
		Q(7) = Q_Equilibrium(7) - 0.0055111941;  // Preserve Rod3 length

		break;
	
	default:
		checkNoEntry();
		return {};
	}

	const Eigen::VectorXd QDot = Eigen::VectorXd::Zero(8);

	return FSysState(Q, QDot);
}

// =============================================================================
// Geometry
// =============================================================================

void FTriplePendulum::BuildGeometry(
	EGeometryPreset Preset,
	FFullSys& FullSys
)
{
	switch (Preset)
	{
	case StandardGeometry:
	{
		const FSysState X_Equilibrium = BuildStatePreset(
			VerticalEquilibrium
		);

		FSubSys& TripleInvertedPendulum = FullSys.CreateSubSys(
			TEXT("TripleInvertedPendulum")
		);

		// Bodies
		BodiesIndex.Base = FullSys.AddBody(
			FBody::SlidingMassX(
				FVector2D(
					X_Equilibrium.Q(0),
					X_Equilibrium.Q(1)
				),
				2.0
			)
		);

		BodiesIndex.Mass1 = FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Equilibrium.Q(2),
					X_Equilibrium.Q(3)
				),
				1.4
			)
		);

		BodiesIndex.Mass2 = FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Equilibrium.Q(4),
					X_Equilibrium.Q(5)
				),
				1.0
			)
		);

		BodiesIndex.Mass3 = FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Equilibrium.Q(6),
					X_Equilibrium.Q(7)
				),
				0.7
			)
		);

		// Rods
		const int32 Rod1Index = FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BodiesIndex.Base,
				BodiesIndex.Mass1
			)
		);

		const int32 Rod2Index = FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BodiesIndex.Mass1,
				BodiesIndex.Mass2
			)
		);

		const int32 Rod3Index = FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BodiesIndex.Mass2,
				BodiesIndex.Mass3
			)
		);

		// Subsystem associations - Bodies
		TripleInvertedPendulum.AddBody(TEXT("Base"), BodiesIndex.Base);
		TripleInvertedPendulum.AddBody(TEXT("Mass1"), BodiesIndex.Mass1);
		TripleInvertedPendulum.AddBody(TEXT("Mass2"), BodiesIndex.Mass2);
		TripleInvertedPendulum.AddBody(TEXT("Mass3"), BodiesIndex.Mass3);

		// Subsystem associations - Elements
		TripleInvertedPendulum.AddElement(TEXT("Rod1"), Rod1Index);
		TripleInvertedPendulum.AddElement(TEXT("Rod2"), Rod2Index);
		TripleInvertedPendulum.AddElement(TEXT("Rod3"), Rod3Index);

		break;
	}

	default:
		checkNoEntry();
		break;
	}
}

// =============================================================================
// Actuación controlada
// =============================================================================

void FTriplePendulum::ConfigureActuation(
	EActuationPreset Preset,
	FControlSys& ControlSys
)
{
	switch (Preset)
	{
	case NoActuation:
		break;


	case BaseOnly:
	{
		FLinearControlChannel BaseHorizontal(
			TEXT("BaseHorizontal")
		);

		BaseHorizontal.AddForceContribution(
			BodiesIndex.Base,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(
			MoveTemp(BaseHorizontal)
		);

		break;
	}


	case BaseAndMass2:
	{
		FLinearControlChannel BaseHorizontal(
			TEXT("BaseHorizontal")
		);

		BaseHorizontal.AddForceContribution(
			BodiesIndex.Base,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(
			MoveTemp(BaseHorizontal)
		);

		FLinearControlChannel Mass2Horizontal(
			TEXT("Mass2Horizontal")
		);

		Mass2Horizontal.AddForceContribution(
			BodiesIndex.Mass2,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(
			MoveTemp(Mass2Horizontal)
		);

		break;
	}


	default:
		checkNoEntry();
		break;
	}
}

void FTriplePendulum::AddExternalForces(
	TArray<FU_ExtForce>& U_ExtForces
)
{
	U_ExtForces.Reset();

	U_ExtForces.Add({
		BodiesIndex.Mass1,
		FVector2D::ZeroVector
		});
}

// =============================================================================
// System
// =============================================================================

void FTriplePendulum::BuildSystem(
	EGeometryPreset GeometryPreset,
	EActuationPreset ActuationPreset,
	FFullSys& FullSys,
	FControlSys& ControlSys,
	TArray<FU_ExtForce>& U_ExtForces
)
{
	BuildGeometry(
		GeometryPreset,
		FullSys
	);

	ConfigureActuation(
		ActuationPreset,
		ControlSys
	);

	AddExternalForces(
		U_ExtForces
	);
}