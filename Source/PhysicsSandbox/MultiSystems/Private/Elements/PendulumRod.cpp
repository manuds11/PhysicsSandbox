#include "Elements/PendulumRod.h"

FPendulumRod::FPendulumRod(
	int32 InPivotBody,
	int32 InBobBody,
	double InLength
)
	: PivotBody(InPivotBody)
	, BobBody(InBobBody)
	, Length(InLength)
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

	double CurrentLengthMod = 0.0;

	const FPolarBase PolarBase =
		ComputePolarBase(
			Pivot,
			Bob,
			CurrentLengthMod
		);

	if (CurrentLengthMod <= UE_SMALL_NUMBER)
	{
		return;
	}

	const FVector2D RelativeVelocity =
		ComputeRelativeVelocity(
			Pivot,
			Bob
		);

	const double TangentialSpeed =
		ComputeTangentialSpeed(
			RelativeVelocity,
			PolarBase
		);

	const double TangentialSpeedModSquared =
		TangentialSpeed * TangentialSpeed;

	const double Tension =
		ComputeTension(
			Pivot,
			Bob,
			PolarBase,
			CurrentLengthMod,
			TangentialSpeedModSquared
		);

	const FVector2D ConstraintForce =
		Tension * PolarBase.e_Radial;

	Pivot.NetForce += ConstraintForce;
	Bob.NetForce -= ConstraintForce;
}

FPolarBase FPendulumRod::ComputePolarBase(
	const FBody& Pivot,
	const FBody& Bob,
	double& CurrentLength
) const
{
	FPolarBase PolarBase;

	const FVector2D RelativePosition =
		Bob.Position - Pivot.Position;

	CurrentLength =
		RelativePosition.Size();

	if (CurrentLength <= UE_SMALL_NUMBER)
	{
		return PolarBase;
	}

	PolarBase.e_Radial =
		RelativePosition / CurrentLength;

	// e_Theta is e_Radial rotated 90 degrees counter-clockwise.
	PolarBase.e_Theta = FVector2D(
		-PolarBase.e_Radial.Y,
		PolarBase.e_Radial.X
	);

	return PolarBase;
}

FVector2D FPendulumRod::ComputeRelativeVelocity(
	const FBody& Pivot,
	const FBody& Bob
) const
{
	return Bob.Velocity - Pivot.Velocity;
}

double FPendulumRod::ComputeTangentialSpeed(
	const FVector2D& RelativeVelocity,
	const FPolarBase& PolarBase
) const
{
	return FVector2D::DotProduct(
		RelativeVelocity,
		PolarBase.e_Theta
	);
}

double FPendulumRod::ComputeTension(
	const FBody& Pivot,
	const FBody& Bob,
	const FPolarBase& PolarBase,
	double CurrentLengthMod,
	double TangentialSpeedModSquared
) const
{
	if (
		Pivot.Mass <= UE_SMALL_NUMBER ||
		Bob.Mass <= UE_SMALL_NUMBER ||
		CurrentLengthMod <= UE_SMALL_NUMBER
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
	// Newton:
	// a_P = (F_P + T e_Radial) / m_P
	// a_B = (F_B - T e_Radial) / m_B
	//
	// Tension:
	// T =
	// [ Dot((F_B / m_B) - (F_P / m_P), e_Radial)
	//   + (v_t^2 / L) ]
	// / [ (1 / m_P) + (1 / m_B) ]

	const FVector2D ExternalAccelerationDifference =
		(Bob.NetForce / Bob.Mass)
		- (Pivot.NetForce / Pivot.Mass);

	const double ExternalRadialAcceleration =
		FVector2D::DotProduct(
			ExternalAccelerationDifference,
			PolarBase.e_Radial
		);

	const double InverseMassSum =
		(1.0 / Pivot.Mass)
		+ (1.0 / Bob.Mass);

	const double Tension =
		(
			ExternalRadialAcceleration
			+ TangentialSpeedModSquared / CurrentLengthMod
			)
		/ InverseMassSum;

	return Tension;
}