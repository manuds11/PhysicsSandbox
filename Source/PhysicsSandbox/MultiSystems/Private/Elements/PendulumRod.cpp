#include "Elements/PendulumRod.h"

FPendulumRod::FPendulumRod(
	int32 InPivotBody,
	int32 InBobBody,
	double InTheoreticalLength
)
	: PivotBody(InPivotBody)
	, BobBody(InBobBody)
	, TheoreticalLength(InTheoreticalLength)
{
}

bool FPendulumRod::GetConnectedBodies(
	int32& OutBodyA,
	int32& OutBodyB
) const
{
	OutBodyA = PivotBody;
	OutBodyB = BobBody;

	return true;
}

void FPendulumRod::ApplyForces(
	TArray<FBody>& Bodies
) const
{
	if (!Bodies.IsValidIndex(PivotBody) || !Bodies.IsValidIndex(BobBody))
	{
		return;
	}

	FBody& Pivot = Bodies[PivotBody];
	FBody& Bob = Bodies[BobBody];

	UpdatePendulumKinematics(Pivot, Bob);

	if (PendulumPos.ComputedLength <= UE_SMALL_NUMBER)
	{
		return;
	}

	const double Tension =
		ComputeTension(
			Pivot,
			Bob,
			PolarBase
		);

	const FVector2D ConstraintForce =
		Tension * PolarBase.e_Radial;

	Pivot.NetForce += ConstraintForce;
	Bob.NetForce -= ConstraintForce;
}

void FPendulumRod::UpdatePendulumKinematics(
	const FBody& Pivot,
	const FBody& Bob
) const
{
	PendulumPos.Bob2Pivot =
		Bob.Position - Pivot.Position;

	PendulumPos.ComputedLength =
		PendulumPos.Bob2Pivot.Size();

	if (PendulumPos.ComputedLength <= UE_SMALL_NUMBER)
	{
		PolarBase = FPolarBase();
		PendulumVel = FPendulumVelocities();
		return;
	}

	PolarBase = ComputePolarBase();

	PendulumVel.Bob2Pivot =
		Bob.Velocity - Pivot.Velocity;

	PendulumVel.Bob2PivotTangential =
		FVector2D::DotProduct(
			PendulumVel.Bob2Pivot,
			PolarBase.e_Theta
		);

	PendulumVel.Bob2PivotRadial =
		FVector2D::DotProduct(
			PendulumVel.Bob2Pivot,
			PolarBase.e_Radial
		);
}

FPolarBase FPendulumRod::ComputePolarBase() const
{
	FPolarBase ComputedPolarBase;

	ComputedPolarBase.e_Radial =
		PendulumPos.Bob2Pivot / PendulumPos.ComputedLength;

	// e_Theta is e_Radial rotated 90 degrees counter-clockwise.
	ComputedPolarBase.e_Theta = FVector2D(
		-ComputedPolarBase.e_Radial.Y,
		ComputedPolarBase.e_Radial.X
	);

	return ComputedPolarBase;
}

FVector2D FPendulumRod::ComputeFreeAcceleration(
	const FBody& Body
) const
{
	if (Body.Mass <= UE_SMALL_NUMBER)
	{
		return FVector2D::ZeroVector;
	}

	FVector2D FreeAcceleration =
		Body.NetForce / Body.Mass;

	if (Body.bXFixed)
	{
		FreeAcceleration.X = 0.0;
	}

	if (Body.bYFixed)
	{
		FreeAcceleration.Y = 0.0;
	}

	return FreeAcceleration;
}

double FPendulumRod::ComputeEffectiveInverseMass(
	const FBody& Body,
	const FVector2D& e_Radial
) const
{
	if (Body.Mass <= UE_SMALL_NUMBER)
	{
		return 0.0;
	}

	double EffectiveInverseMass = 0.0;

	if (!Body.bXFixed)
	{
		EffectiveInverseMass +=
			e_Radial.X * e_Radial.X / Body.Mass;
	}

	if (!Body.bYFixed)
	{
		EffectiveInverseMass +=
			e_Radial.Y * e_Radial.Y / Body.Mass;
	}

	return EffectiveInverseMass;
}

double FPendulumRod::ComputeTension(
	const FBody& Pivot,
	const FBody& Bob,
	const FPolarBase& InPolarBase
) const
{
	if (
		Pivot.Mass <= UE_SMALL_NUMBER ||
		Bob.Mass <= UE_SMALL_NUMBER ||
		PendulumPos.ComputedLength <= UE_SMALL_NUMBER
		)
	{
		return 0.0;
	}

	// Constraint:
	// |x_B - x_P| = L
	//
	// Velocity-level constraint:
	// Dot(v_B - v_P, e_Radial) = 0
	//
	// Acceleration-level constraint:
	// Dot(a_B - a_P, e_Radial) = -v_t^2 / L
	//
	// Newton with fixed axes:
	// a_P = movable components of (F_P + T e_Radial) / m_P
	// a_B = movable components of (F_B - T e_Radial) / m_B
	//
	// Tension:
	// T =
	// [ Dot(a_B_external - a_P_external, e_Radial)
	//   + (v_t^2 / L) ]
	// / EffectiveInverseMassSum

	const double TangentialSpeedSquared =
		PendulumVel.Bob2PivotTangential
		* PendulumVel.Bob2PivotTangential;

	const FVector2D ExternalAccelerationDifference =
		ComputeFreeAcceleration(Bob)
		- ComputeFreeAcceleration(Pivot);

	const double ExternalRadialAcceleration =
		FVector2D::DotProduct(
			ExternalAccelerationDifference,
			InPolarBase.e_Radial
		);

	const double EffectiveInverseMassSum =
		ComputeEffectiveInverseMass(Pivot, InPolarBase.e_Radial)
		+ ComputeEffectiveInverseMass(Bob, InPolarBase.e_Radial);

	if (EffectiveInverseMassSum <= UE_SMALL_NUMBER)
	{
		return 0.0;
	}

	const double ConstraintLength =
		TheoreticalLength > UE_SMALL_NUMBER
		? TheoreticalLength
		: PendulumPos.ComputedLength;

	const double Tension =
		(
			ExternalRadialAcceleration
			+ TangentialSpeedSquared / ConstraintLength
			)
		/ EffectiveInverseMassSum;

	return Tension;
}