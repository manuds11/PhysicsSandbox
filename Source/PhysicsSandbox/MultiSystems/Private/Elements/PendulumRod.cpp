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

	if (
		Pivot.Mass <= UE_SMALL_NUMBER ||
		Bob.Mass <= UE_SMALL_NUMBER ||
		PendulumPos.ComputedLength <= UE_SMALL_NUMBER
		)
	{
		return 0.0;
	}

	const double TangentialSpeedSquared =
		PendulumVel.Bob2PivotTangential
		* PendulumVel.Bob2PivotTangential;

	const double ConstraintLength =
		TheoreticalLength > UE_SMALL_NUMBER
		? TheoreticalLength
		: PendulumPos.ComputedLength;

	FVector2D PivotExternalAcceleration =
		Pivot.NetForce / Pivot.Mass;

	if (Pivot.bXFixed)
	{
		PivotExternalAcceleration.X = 0.0;
	}

	if (Pivot.bYFixed)
	{
		PivotExternalAcceleration.Y = 0.0;
	}

	FVector2D BobExternalAcceleration =
		Bob.NetForce / Bob.Mass;

	if (Bob.bXFixed)
	{
		BobExternalAcceleration.X = 0.0;
	}

	if (Bob.bYFixed)
	{
		BobExternalAcceleration.Y = 0.0;
	}

	const FVector2D ExternalAccelerationDifference =
		BobExternalAcceleration - PivotExternalAcceleration;

	const double ExternalRadialAcceleration =
		FVector2D::DotProduct(
			ExternalAccelerationDifference,
			InPolarBase.e_Radial
		);

	double PivotEffectiveInverseMass = 0.0;

	if (!Pivot.bXFixed)
	{
		const double ProjectionOnX =
			FVector2D::DotProduct(
				InPolarBase.e_Radial,
				CartesianBase::e_X
			);

		PivotEffectiveInverseMass +=
			ProjectionOnX * ProjectionOnX / Pivot.Mass;
	}

	if (!Pivot.bYFixed)
	{
		const double ProjectionOnY =
			FVector2D::DotProduct(
				InPolarBase.e_Radial,
				CartesianBase::e_Y
			);

		PivotEffectiveInverseMass +=
			ProjectionOnY * ProjectionOnY / Pivot.Mass;
	}

	double BobEffectiveInverseMass = 0.0;

	if (!Bob.bXFixed)
	{
		const double ProjectionOnX =
			FVector2D::DotProduct(
				InPolarBase.e_Radial,
				CartesianBase::e_X
			);

		BobEffectiveInverseMass +=
			ProjectionOnX * ProjectionOnX / Bob.Mass;
	}

	if (!Bob.bYFixed)
	{
		const double ProjectionOnY =
			FVector2D::DotProduct(
				InPolarBase.e_Radial,
				CartesianBase::e_Y
			);

		BobEffectiveInverseMass +=
			ProjectionOnY * ProjectionOnY / Bob.Mass;
	}

	const double ConstraintCoefficientA =
		PivotEffectiveInverseMass
		+ BobEffectiveInverseMass;

	if (ConstraintCoefficientA <= UE_SMALL_NUMBER)
	{
		return 0.0;
	}

	const double ConstraintRhsB =
		ExternalRadialAcceleration
		+ TangentialSpeedSquared / ConstraintLength;

	const double Tension =
		ConstraintRhsB / ConstraintCoefficientA;

	return Tension;
}