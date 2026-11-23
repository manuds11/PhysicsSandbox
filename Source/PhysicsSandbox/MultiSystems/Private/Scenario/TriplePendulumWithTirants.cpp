#include "Scenario/TriplePendulumWithTirants.h"

#include "Simulation/FullSys.h"
#include "Elements/PendulumRod.h"
#include "Elements/SpringDamper.h"


FSysState FTriplePendulumWithTirants::BuildState(EStatePreset Preset)
{
	Eigen::VectorXd Q_Default = Eigen::VectorXd::Zero(12);

	// Base
	Q_Default(0) = 2.50;
	Q_Default(1) = -1.20;

	// Mass1
	Q_Default(2) = 2.58;
	Q_Default(3) = -0.40;

	// Mass2
	Q_Default(4) = 2.38;
	Q_Default(5) = 0.32;

	// Mass3
	Q_Default(6) = 2.62;
	Q_Default(7) = 1.20;

	// LeftAnchor
	Q_Default(8) = 1.14;
	Q_Default(9) = 1.20;

	// RightAnchor
	Q_Default(10) = 3.86;
	Q_Default(11) = 1.20;


	Eigen::VectorXd Q = Q_Default;

	switch (Preset)
	{
	case Default:
		break;

	default:
		checkNoEntry();
		return {};
	}

	const Eigen::VectorXd QDot = Eigen::VectorXd::Zero(12);

	return FSysState(Q, QDot);
}

void FTriplePendulumWithTirants::BuildSystem(FFullSys& FullSys)
{
	const FSysState X_Default = BuildState(Default);

	// -------------------------------------------------------------------------
	// Subsystem
	// -------------------------------------------------------------------------

	FSubSys& TripleInvertedPendulum =
		FullSys.CreateSubSys(TEXT("TriplePendulumWithTirants"));

	// -------------------------------------------------------------------------
	// Bodies
	// -------------------------------------------------------------------------

	const int32 BaseIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(
					X_Default.Q(0),
					X_Default.Q(1)
				)
			)
		);

	const int32 Mass1Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Default.Q(2),
					X_Default.Q(3)
				),
				1.4
			)
		);

	const int32 Mass2Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Default.Q(4),
					X_Default.Q(5)
				),
				1.0
			)
		);

	const int32 Mass3Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(
					X_Default.Q(6),
					X_Default.Q(7)
				),
				0.7
			)
		);

	const int32 LeftAnchorIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(
					X_Default.Q(8),
					X_Default.Q(9)
				)
			)
		);

	const int32 RightAnchorIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(
					X_Default.Q(10),
					X_Default.Q(11)
				)
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
	// Tirants
	// -------------------------------------------------------------------------

	const int32 LeftSpringIndex =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				LeftAnchorIndex,
				Mass3Index,
				30.0,
				0.05,
				1.48
			)
		);

	const int32 RightSpringIndex =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				Mass3Index,
				RightAnchorIndex,
				30.0,
				0.05,
				1.24
			)
		);

	// -------------------------------------------------------------------------
	// Subsystem associations - Bodies
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddBody(TEXT("Base"), BaseIndex);
	TripleInvertedPendulum.AddBody(TEXT("Mass1"), Mass1Index);
	TripleInvertedPendulum.AddBody(TEXT("Mass2"), Mass2Index);
	TripleInvertedPendulum.AddBody(TEXT("Mass3"), Mass3Index);
	TripleInvertedPendulum.AddBody(TEXT("LeftAnchor"), LeftAnchorIndex);
	TripleInvertedPendulum.AddBody(TEXT("RightAnchor"), RightAnchorIndex);

	// -------------------------------------------------------------------------
	// Subsystem associations - Elements
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddElement(TEXT("Rod1"), Rod1Index);
	TripleInvertedPendulum.AddElement(TEXT("Rod2"), Rod2Index);
	TripleInvertedPendulum.AddElement(TEXT("Rod3"), Rod3Index);

	TripleInvertedPendulum.AddElement(TEXT("LeftSpring"), LeftSpringIndex);
	TripleInvertedPendulum.AddElement(TEXT("RightSpring"), RightSpringIndex);
}