#include "Scenario/DoubleSpringDoublePendulum.h"

#include "Simulation/FullSys.h"
#include "Control/ControlSys.h"
#include "Elements/PendulumRod.h"
#include "Elements/SpringDamper.h"


// -----------------------------------------------------------------------------
// System construction
// -----------------------------------------------------------------------------

void FDoubleSpringDoublePendulum::BuildSystem(
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

FSysState FDoubleSpringDoublePendulum::BuildStatePreset(EStatePreset Preset)
{
	Eigen::VectorXd Q_Normal = Eigen::VectorXd::Zero(10);

	// Wall
	Q_Normal(0) = 0.0;
	Q_Normal(1) = 0.0;

	// Mass1
	Q_Normal(2) = 2.0;
	Q_Normal(3) = 0.0;

	// Mass2
	Q_Normal(4) = 3.5;
	Q_Normal(5) = 0.0;

	// Bob1
	Q_Normal(6) = 4.5;
	Q_Normal(7) = -0.5;

	// Bob2
	Q_Normal(8) = 5.2;
	Q_Normal(9) = -1.4;


	Eigen::VectorXd Q = Q_Normal;

	switch (Preset)
	{
	case EStatePreset::Normal:
		break;

	case EStatePreset::HighEnergyRight:

		// Double pendulum approximately 60 deg to the right of vertical.
		// Rod lengths are preserved.

		// Bob1
		Q(6) = 4.4682458366;
		Q(7) = -0.5590169944;

		// Bob2
		Q(8) = 5.4556667195;
		Q(9) = -1.1291047069;

		break;

	default:
		checkNoEntry();
		return {};
	}

	const Eigen::VectorXd QDot = Eigen::VectorXd::Zero(10);

	return FSysState(Q, QDot);
}


// -----------------------------------------------------------------------------
// Geometry
// -----------------------------------------------------------------------------

void FDoubleSpringDoublePendulum::BuildGeometry(
	EGeometryPreset Preset,
	FFullSys& FullSys
)
{
	switch (Preset)
	{
	case EGeometryPreset::StandardGeometry:
	{
		const FSysState X_Normal = BuildStatePreset(EStatePreset::Normal);


		// Subsystem
		// ------------------------

		FSubSys& DoubleSpringDoublePendulum = FullSys.CreateSubSys(
			TEXT("DoubleSpringDoublePendulum")
		);


		// Bodies
		// ------------------------

		BodiesIndex.Wall = FullSys.AddBody(
			FBody::Fixed(
				FVector2D(
					X_Normal.Q(0),
					X_Normal.Q(1)
				)
			)
		);

		BodiesIndex.Mass1 = FullSys.AddBody(
			FBody::SlidingMassX(
				FVector2D(
					X_Normal.Q(2),
					X_Normal.Q(3)
				),
				1.0
			)
		);

		BodiesIndex.Mass2 = FullSys.AddBody(
			FBody::SlidingMassX(
				FVector2D(
					X_Normal.Q(4),
					X_Normal.Q(5)
				),
				1.0
			)
		);

		BodiesIndex.Bob1 = FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Normal.Q(6),
					X_Normal.Q(7)
				),
				1.5
			)
		);

		BodiesIndex.Bob2 = FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Normal.Q(8),
					X_Normal.Q(9)
				),
				1.0
			)
		);


		// Springs
		// ------------------------

		const double SpringStiffness = 20.0;
		const double SpringDamping = 1.0;
		const double SpringRestLength = 1.5;

		const int32 Spring1Index = FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				BodiesIndex.Wall,
				BodiesIndex.Mass1,
				SpringStiffness,
				SpringDamping,
				SpringRestLength
			)
		);

		const int32 Spring2Index = FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				BodiesIndex.Mass1,
				BodiesIndex.Mass2,
				SpringStiffness,
				SpringDamping,
				SpringRestLength
			)
		);


		// Pendulum rods
		// ------------------------

		const int32 PendulumRod1Index = FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BodiesIndex.Mass2,
				BodiesIndex.Bob1
			)
		);

		const int32 PendulumRod2Index = FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BodiesIndex.Bob1,
				BodiesIndex.Bob2
			)
		);


		// Subsystem associations - Bodies
		// ------------------------

		DoubleSpringDoublePendulum.AddBody(TEXT("Wall"), BodiesIndex.Wall);
		DoubleSpringDoublePendulum.AddBody(TEXT("Mass1"), BodiesIndex.Mass1);
		DoubleSpringDoublePendulum.AddBody(TEXT("Mass2"), BodiesIndex.Mass2);
		DoubleSpringDoublePendulum.AddBody(TEXT("Bob1"), BodiesIndex.Bob1);
		DoubleSpringDoublePendulum.AddBody(TEXT("Bob2"), BodiesIndex.Bob2);


		// Subsystem associations - Elements
		// ------------------------

		DoubleSpringDoublePendulum.AddElement(TEXT("Spring1"), Spring1Index);
		DoubleSpringDoublePendulum.AddElement(TEXT("Spring2"), Spring2Index);
		DoubleSpringDoublePendulum.AddElement(TEXT("PendulumRod1"), PendulumRod1Index);
		DoubleSpringDoublePendulum.AddElement(TEXT("PendulumRod2"), PendulumRod2Index);


		// Ports
		// ------------------------

		DoubleSpringDoublePendulum.AddPort(TEXT("End"), BodiesIndex.Mass2);

		break;
	}

	default:
		checkNoEntry();
		break;
	}
}


// -----------------------------------------------------------------------------
// Actuation
// -----------------------------------------------------------------------------

void FDoubleSpringDoublePendulum::ConfigureActuation(
	EActuationPreset Preset,
	FControlSys& ControlSys
)
{
	switch (Preset)
	{
	case EActuationPreset::NoActuation:
		break;

	default:
		checkNoEntry();
		break;
	}
}


// -----------------------------------------------------------------------------
// External forces
// -----------------------------------------------------------------------------

void FDoubleSpringDoublePendulum::AddExternalForces(
	TArray<FU_ExtForce>& U_ExtForces
)
{
	U_ExtForces.Reset();
}