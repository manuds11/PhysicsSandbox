#include "Scenario/DoubleSpringDoublePendulum.h"

#include "Simulation/FullSys.h"
#include "Elements/PendulumRod.h"
#include "Elements/SpringDamper.h"


FSysState FDoubleSpringDoublePendulum::BuildState(EStatePreset Preset)
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
	case Normal:
		break;

	case HighEnergyRight:

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

void FDoubleSpringDoublePendulum::BuildSystem(FFullSys& FullSys)
{
	const FSysState X_Normal = BuildState(Normal);

	// -------------------------------------------------------------------------
	// Subsystem
	// -------------------------------------------------------------------------

	FSubSys& DoubleSpringDoublePendulum =
		FullSys.CreateSubSys(TEXT("DoubleSpringDoublePendulum"));

	// -------------------------------------------------------------------------
	// Bodies
	// -------------------------------------------------------------------------

	const int32 WallIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(
					X_Normal.Q(0),
					X_Normal.Q(1)
				)
			)
		);

	const int32 Mass1Index =
		FullSys.AddBody(
			FBody::SlidingMassX(
				FVector2D(
					X_Normal.Q(2),
					X_Normal.Q(3)
				),
				1.0
			)
		);

	const int32 Mass2Index =
		FullSys.AddBody(
			FBody::SlidingMassX(
				FVector2D(
					X_Normal.Q(4),
					X_Normal.Q(5)
				),
				1.0
			)
		);

	const int32 Bob1Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Normal.Q(6),
					X_Normal.Q(7)
				),
				1.5
			)
		);

	const int32 Bob2Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Normal.Q(8),
					X_Normal.Q(9)
				),
				1.0
			)
		);

	// -------------------------------------------------------------------------
	// Springs
	// -------------------------------------------------------------------------

	const int32 Spring1Index =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				WallIndex,
				Mass1Index,
				20.0,
				1.0,
				1.5
			)
		);

	const int32 Spring2Index =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				Mass1Index,
				Mass2Index,
				20.0,
				1.0,
				1.5
			)
		);

	// -------------------------------------------------------------------------
	// Pendulum rods
	// -------------------------------------------------------------------------

	const int32 PendulumRod1Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Mass2Index,
				Bob1Index
			)
		);

	const int32 PendulumRod2Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Bob1Index,
				Bob2Index
			)
		);

	// -------------------------------------------------------------------------
	// Subsystem associations
	// -------------------------------------------------------------------------

	DoubleSpringDoublePendulum.AddBody(TEXT("Wall"), WallIndex);
	DoubleSpringDoublePendulum.AddBody(TEXT("Mass1"), Mass1Index);
	DoubleSpringDoublePendulum.AddBody(TEXT("Mass2"), Mass2Index);
	DoubleSpringDoublePendulum.AddBody(TEXT("Bob1"), Bob1Index);
	DoubleSpringDoublePendulum.AddBody(TEXT("Bob2"), Bob2Index);

	DoubleSpringDoublePendulum.AddElement(TEXT("Spring1"), Spring1Index);
	DoubleSpringDoublePendulum.AddElement(TEXT("Spring2"), Spring2Index);
	DoubleSpringDoublePendulum.AddElement(TEXT("PendulumRod1"), PendulumRod1Index);
	DoubleSpringDoublePendulum.AddElement(TEXT("PendulumRod2"), PendulumRod2Index);

	DoubleSpringDoublePendulum.AddPort(TEXT("End"), Mass2Index);
}