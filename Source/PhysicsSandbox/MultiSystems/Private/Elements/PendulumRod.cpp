#include "Elements/PendulumRod.h"

FPendulumRod::FPendulumRod(
	int32 InPivotBody,
	int32 InBobBody
)
	: PivotBody(InPivotBody)
	, BobBody(InBobBody)
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

void FPendulumRod::InitializePendulumLength()
{
	if (PendulumPos.ComputedLength <= UE_SMALL_NUMBER)
	{
		return;
	}

	PendulumPos.InitialLength = PendulumPos.ComputedLength;
	PendulumPos.bLengthInitialized = true;
}

void FPendulumRod::ApplyForces(
	TArray<FBody>& Bodies
)
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

	LastComputedTension = Tension;

	Pivot.NetForce += ConstraintForce;
	Bob.NetForce -= ConstraintForce;

}

void FPendulumRod::UpdatePendulumKinematics(
	const FBody& Pivot,
	const FBody& Bob
)
{
	PendulumPos.Bob2Pivot =
		Bob.Position - Pivot.Position;

	PendulumPos.ComputedLength =
		PendulumPos.Bob2Pivot.Size();

	if (!PendulumPos.bLengthInitialized)
	{
		InitializePendulumLength();
	}

	if (PendulumPos.ComputedLength <= UE_SMALL_NUMBER)
	{
		PolarBase = FPolarBase();
		PendulumVel = FPendulumVelocities();
		return;
	}

	PolarBase = ComputePolarBase();

	UpdateLengthErrors();

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

void FPendulumRod::UpdateLengthErrors()
{
	if (PendulumPos.InitialLength <= UE_SMALL_NUMBER)
	{
		PendulumPos.LengthAbsError = 0.0;
		PendulumPos.LengthRelError = 0.0;
		return;
	}
	
	PendulumPos.LengthAbsError =
		PendulumPos.ComputedLength - PendulumPos.InitialLength;

	PendulumPos.LengthRelError =
		PendulumPos.LengthAbsError / PendulumPos.InitialLength;
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
	const FVector2D& In_e_Radial
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
			In_e_Radial.X * In_e_Radial.X / Body.Mass;
	}

	if (!Body.bYFixed)
	{
		EffectiveInverseMass +=
			In_e_Radial.Y * In_e_Radial.Y / Body.Mass;
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

	const double ConstraintLength = PendulumPos.InitialLength;

	FVector2D PivotExternalAcceleration = ComputeFreeAcceleration(Pivot); // Checks restricted axes and set to zero acceleration components if so.

	FVector2D BobExternalAcceleration = ComputeFreeAcceleration(Bob);

	const FVector2D Accel_Bob2Pivot =
		BobExternalAcceleration - PivotExternalAcceleration;

	const double ExtAccel_Radial_Bob2Pivot = // Calcula la aceleracion debida a las fuerzas externas sobre el Body sin contar la tension, las que recibe de otros elementos.
		FVector2D::DotProduct(
			Accel_Bob2Pivot,
			InPolarBase.e_Radial
		);

	double PivotEffectiveInverseMass = ComputeEffectiveInverseMass(
		Pivot,
		InPolarBase.e_Radial
	);

	double BobEffectiveInverseMass = ComputeEffectiveInverseMass(
		Bob,
		InPolarBase.e_Radial
	);

	const double ConstraintCoefficientA =
		PivotEffectiveInverseMass
		+ BobEffectiveInverseMass;

	if (ConstraintCoefficientA <= UE_SMALL_NUMBER)
	{
		return 0.0;
	}

	const double ConstraintRhsB =
		ExtAccel_Radial_Bob2Pivot
		+ TangentialSpeedSquared / ConstraintLength;

	const double Tension =
		ConstraintRhsB / ConstraintCoefficientA;

	return Tension;
}