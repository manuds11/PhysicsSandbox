#include "Scenario/TriplePendulumWithTirants.h"

#include "Simulation/FullSys.h"
#include "Control/ControlSys.h"
#include "Elements/PendulumRod.h"
#include "Elements/SpringDamper.h"


// -----------------------------------------------------------------------------
// System construction
// -----------------------------------------------------------------------------

void FTriplePendulumWithTirants::BuildSystem(
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

	AddExternalForces(U_ExtForces);
}


// -----------------------------------------------------------------------------
// State presets
// -----------------------------------------------------------------------------

FSysState FTriplePendulumWithTirants::BuildStatePreset(EStatePreset Preset)
{
	Eigen::VectorXd Q_Equilibrium = Eigen::VectorXd::Zero(12);

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

	// LeftAnchor
	Q_Equilibrium(8) = 1.14;
	Q_Equilibrium(9) = 1.26;

	// RightAnchor
	Q_Equilibrium(10) = 3.86;
	Q_Equilibrium(11) = 1.26;


	// Rod lengths
	// ------------------------

	const double L1 = Q_Equilibrium(3) - Q_Equilibrium(1);
	const double L2 = Q_Equilibrium(5) - Q_Equilibrium(3);
	const double L3 = Q_Equilibrium(7) - Q_Equilibrium(5);


	// Angular presets
	// ------------------------

	double Theta1 = 0.0;
	double Theta2 = 0.0;
	double Theta3 = 0.0;

	switch (Preset)
	{
	case EStatePreset::VerticalEquilibrium:
		break;

	case EStatePreset::Default:

		Theta1 = FMath::DegreesToRadians(6.0);
		Theta2 = FMath::DegreesToRadians(-15.0);
		Theta3 = FMath::DegreesToRadians(15.0);

		break;

	case EStatePreset::SlightLeft:

		Theta3 = FMath::DegreesToRadians(-0.1);

		break;

	case EStatePreset::ModerateLeft:

		Theta3 = FMath::DegreesToRadians(-5.0);

		break;

	case EStatePreset::StrongRight:

		Theta3 = FMath::DegreesToRadians(15.0);

		break;

	default:
		checkNoEntry();
		return {};
	}


	// Reconstruct body positions
	// ------------------------

	Eigen::VectorXd Q = Q_Equilibrium;

	// Mass1
	Q(2) = Q(0) + L1 * FMath::Sin(Theta1);
	Q(3) = Q(1) + L1 * FMath::Cos(Theta1);

	// Mass2
	Q(4) = Q(2) + L2 * FMath::Sin(Theta2);
	Q(5) = Q(3) + L2 * FMath::Cos(Theta2);

	// Mass3
	Q(6) = Q(4) + L3 * FMath::Sin(Theta3);
	Q(7) = Q(5) + L3 * FMath::Cos(Theta3);

	const Eigen::VectorXd QDot = Eigen::VectorXd::Zero(12);

	return FSysState(Q, QDot);
}


// -----------------------------------------------------------------------------
// Geometry
// -----------------------------------------------------------------------------

void FTriplePendulumWithTirants::BuildGeometry(
	EGeometryPreset Preset,
	FFullSys& FullSys
)
{
	const FSysState X_Equilibrium = BuildStatePreset(EStatePreset::VerticalEquilibrium);


	// Subsystem
	// ------------------------

	FSubSys& TripleInvertedPendulum = FullSys.CreateSubSys(
		TEXT("TriplePendulumWithTirants")
	);


	// Base
	// ------------------------

	const FVector2D BasePosition(
		X_Equilibrium.Q(0),
		X_Equilibrium.Q(1)
	);

	switch (Preset)
	{
	case EGeometryPreset::SlidingBase:

		BodiesIndex.Base = FullSys.AddBody(
			FBody::SlidingMassX(
				BasePosition,
				2.0
			)
		);

		break;

	case EGeometryPreset::FixedBase:

		BodiesIndex.Base = FullSys.AddBody(
			FBody::Fixed(BasePosition)
		);

		break;

	default:
		checkNoEntry();
		return;
	}


	// Masses
	// ------------------------

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


	// Anchors
	// ------------------------

	BodiesIndex.LeftAnchor = FullSys.AddBody(
		FBody::Fixed(
			FVector2D(
				X_Equilibrium.Q(8),
				X_Equilibrium.Q(9)
			)
		)
	);

	BodiesIndex.RightAnchor = FullSys.AddBody(
		FBody::Fixed(
			FVector2D(
				X_Equilibrium.Q(10),
				X_Equilibrium.Q(11)
			)
		)
	);


	// Rods
	// ------------------------

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


	// Springs
	// ------------------------

	const double SpringStiffness = 10.0;
	const double SpringDamping = 0.15;
	const double SpringRestLength = 1.36;

	const int32 LeftSpringIndex = FullSys.AddElement(
		MakeUnique<FSpringDamper>(
			BodiesIndex.LeftAnchor,
			BodiesIndex.Mass3,
			SpringStiffness,
			SpringDamping,
			SpringRestLength
		)
	);

	const int32 RightSpringIndex = FullSys.AddElement(
		MakeUnique<FSpringDamper>(
			BodiesIndex.Mass3,
			BodiesIndex.RightAnchor,
			SpringStiffness,
			SpringDamping,
			SpringRestLength
		)
	);


	// Subsystem associations - Bodies
	// ------------------------

	TripleInvertedPendulum.AddBody(TEXT("Base"), BodiesIndex.Base);
	TripleInvertedPendulum.AddBody(TEXT("Mass1"), BodiesIndex.Mass1);
	TripleInvertedPendulum.AddBody(TEXT("Mass2"), BodiesIndex.Mass2);
	TripleInvertedPendulum.AddBody(TEXT("Mass3"), BodiesIndex.Mass3);
	TripleInvertedPendulum.AddBody(TEXT("LeftAnchor"), BodiesIndex.LeftAnchor);
	TripleInvertedPendulum.AddBody(TEXT("RightAnchor"), BodiesIndex.RightAnchor);


	// Subsystem associations - Elements
	// ------------------------

	TripleInvertedPendulum.AddElement(TEXT("Rod1"), Rod1Index);
	TripleInvertedPendulum.AddElement(TEXT("Rod2"), Rod2Index);
	TripleInvertedPendulum.AddElement(TEXT("Rod3"), Rod3Index);
	TripleInvertedPendulum.AddElement(TEXT("LeftSpring"), LeftSpringIndex);
	TripleInvertedPendulum.AddElement(TEXT("RightSpring"), RightSpringIndex);
}


// -----------------------------------------------------------------------------
// Actuation
// -----------------------------------------------------------------------------

void FTriplePendulumWithTirants::ConfigureActuation(
	EActuationPreset Preset,
	FControlSys& ControlSys
)
{
	switch (Preset)
	{
	case EActuationPreset::NoActuation:
		break;


	case EActuationPreset::BaseOnly:
	{
		FLinearControlChannel BaseHorizontal(TEXT("BaseHorizontal"));

		BaseHorizontal.AddForceContribution(
			BodiesIndex.Base,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(MoveTemp(BaseHorizontal));

		break;
	}
	case EActuationPreset::Mass1Only:
	{
		FLinearControlChannel Mass1Horizontal(TEXT("Mass1Horizontal"));

		Mass1Horizontal.AddForceContribution(
			BodiesIndex.Mass1,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(MoveTemp(Mass1Horizontal));

		break;
	}
	case EActuationPreset::Mass3Only:
	{
		FLinearControlChannel Mass3Horizontal(TEXT("Mass3Horizontal"));

		Mass3Horizontal.AddForceContribution(
			BodiesIndex.Mass3,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(MoveTemp(Mass3Horizontal));

		break;
	}
	case EActuationPreset::Mass1AndMass2:
	{
		FLinearControlChannel Mass1Horizontal(TEXT("Mass1Horizontal"));

		Mass1Horizontal.AddForceContribution(
			BodiesIndex.Mass1,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(MoveTemp(Mass1Horizontal));


		FLinearControlChannel Mass2Horizontal(TEXT("Mass2Horizontal"));

		Mass2Horizontal.AddForceContribution(
			BodiesIndex.Mass2,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(MoveTemp(Mass2Horizontal));

		break;
	}
	case EActuationPreset::BaseAndMass1:
	{
		FLinearControlChannel BaseHorizontal(TEXT("BaseHorizontal"));

		BaseHorizontal.AddForceContribution(
			BodiesIndex.Base,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(MoveTemp(BaseHorizontal));


		FLinearControlChannel Mass1Horizontal(TEXT("Mass1Horizontal"));

		Mass1Horizontal.AddForceContribution(
			BodiesIndex.Mass1,
			FVector2D(1.0, 0.0)
		);

		ControlSys.AddLinearControlChannel(MoveTemp(Mass1Horizontal));

		break;
	}
	default:
		checkNoEntry();
		break;
	}
}


// -----------------------------------------------------------------------------
// External forces
// -----------------------------------------------------------------------------

void FTriplePendulumWithTirants::AddExternalForces(
	TArray<FU_ExtForce>& U_ExtForces
)
{
	U_ExtForces.Reset();

	U_ExtForces.Add({
		BodiesIndex.Mass1,
		FVector2D::ZeroVector
		});
}