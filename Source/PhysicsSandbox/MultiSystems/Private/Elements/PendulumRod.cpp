#include "Elements/PendulumRod.h"
#include "Math/Units.h"

// -----------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------

FPendulumRod::FPendulumRod(
	int32 InPivotBody,
	int32 InBobBody
)
	: PivotBody(InPivotBody)
	, BobBody(InBobBody)
{
}

// -----------------------------------------------------------------------------
// ISysElement interface
// -----------------------------------------------------------------------------

ESysElementType FPendulumRod::GetElementType() const
{
	return ESysElementType::PendulumRod;
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

void FPendulumRod::UpdateRodJacobian(
	const TArray<FBody>& Bodies
)
{
	if (
		!Bodies.IsValidIndex(PivotBody)
		|| !Bodies.IsValidIndex(BobBody)
		)
	{
		RodJacobian = FRodJacobian();
		return;
	}

	const FBody& Pivot = Bodies[PivotBody];
	const FBody& Bob = Bodies[BobBody];

	UpdatePendulumKinematics(Pivot, Bob);

	if (
		PendulumPos.ComputedLength
		<= UE_SMALL_NUMBER
		)
	{
		RodJacobian =
			FRodJacobian();

		return;
	}

	RodJacobian.JPivot =
		-PolarBase.e_Radial;	// e_Radial = Constriction Normal vector pointing from Pivot to Bob.

	RodJacobian.JBob =
		PolarBase.e_Radial;

	RodJacobian.JDotVel =
		FMath::Square( PendulumVel.Bob2PivotTangential )
		/ PendulumPos.ComputedLength;		// Computed Length must ne used, NOT INITIAL LENGTH, in order to compute accuretely the angular frequency of the pendulum  Omega = Vel_tan/|r|
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

// -----------------------------------------------------------------------------
// State update pipeline
// -----------------------------------------------------------------------------

void FPendulumRod::InitializePendulumLength()
{
	if (PendulumPos.ComputedLength <= UE_SMALL_NUMBER)
	{
		return;
	}

	PendulumPos.InitialLength = PendulumPos.ComputedLength;
	PendulumPos.bLengthInitialized = true;
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

// -----------------------------------------------------------------------------
// Derived computations
// -----------------------------------------------------------------------------

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

double FPendulumRod::ComputeTension(
	const FBody& Pivot,
	const FBody& Bob,
	const FPolarBase& InPolarBase
) const
{
	// Distance constraint:
	// |x_B - x_P| = L0
	//
	// Acceleration-level constraint:
	// Dot(a_B - a_P, e_Radial) = -v_t^2 / L0
	//
	// The rod forces are:
	// Pivot: +T e_Radial
	// Bob:   -T e_Radial
	//
	// After accounting for fixed axes, the scalar constraint equation is:
	//
	//     A * T = b
	//
	// where:
	//     A = sum of effective inverse masses in the radial direction
	//     b = external relative radial acceleration + v_t^2 / L0
	//
	// Therefore:
	//     T = b / A
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

FVector2D FPendulumRod::ComputeFreeAcceleration(
	const FBody& Body
) const
{
	// Computes acceleration from the currently accumulated external forces.
	// Acceleration components along fixed axes are set to zero because those
	// forces are balanced by support reactions.

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
	// Measures the radial acceleration produced by one unit of tension.
	//
	// For each free Cartesian axis:
	//     contribution = e_component^2 / mass
	//
	// Fixed axes do not contribute because their constraint reactions prevent
	// acceleration in those directions.

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

void FPendulumRod::ProjectVelocities(
	TArray<FBody>& Bodies
)
{
	// Velocity-level constraint:
	//
	//     Dot(v_B - v_P, e_Radial) = 0
	//
	// After integration, the provisional state may contain a non-zero radial
	// relative speed:
	//
	//     v_r = Dot(v_B - v_P, e_Radial)
	//
	// We apply equal and opposite radial impulses:
	//
	//     Pivot: +J e_Radial
	//     Bob:   -J e_Radial
	//
	// The scalar impulse is obtained from:
	//
	//     A J = v_r
	//
	// where A is the sum of the effective inverse masses in the radial
	// direction. Therefore:
	//
	//     J = v_r / A
	//
	// Only the radial component is removed. The tangential relative velocity,
	// which represents the physical pendulum motion, is preserved.

	if (!Bodies.IsValidIndex(PivotBody) || !Bodies.IsValidIndex(BobBody))
	{
		return;
	}

	FBody& Pivot = Bodies[PivotBody];
	FBody& Bob = Bodies[BobBody];

	// Recompute the pendulum geometry and velocity from the provisional
	// post-integration state at t_(n+1).
	UpdatePendulumKinematics(Pivot, Bob);

	// ComputeFreeVelocity(const FBody & Body) is not neccesarry because before integration the fixed axes where already imposed and ar conserved.

	if (PendulumPos.ComputedLength <= UE_SMALL_NUMBER)
	{
		return;
	}

	const double RadialSpeedError =
		PendulumVel.Bob2PivotRadial;

	const double PivotEffectiveInverseMass =
		ComputeEffectiveInverseMass(
			Pivot,
			PolarBase.e_Radial
		);

	const double BobEffectiveInverseMass =
		ComputeEffectiveInverseMass(
			Bob,
			PolarBase.e_Radial
		);

	const double ConstraintCoefficientA =
		PivotEffectiveInverseMass
		+ BobEffectiveInverseMass;

	if (ConstraintCoefficientA <= UE_SMALL_NUMBER)
	{
		return;
	}

	const double ConstraintRhsB =
		RadialSpeedError;

	const double ImpulseMagnitude =
		ConstraintRhsB / ConstraintCoefficientA;

	const FVector2D ConstraintImpulse =
		ImpulseMagnitude * PolarBase.e_Radial;

	ApplyVelocityImpulse(
		Pivot,
		ConstraintImpulse
	);

	ApplyVelocityImpulse(
		Bob,
		-ConstraintImpulse
	);

	// Refresh diagnostics so the displayed radial speed corresponds to the
	// corrected state rather than the provisional integrated state.
	UpdatePendulumKinematics(Pivot, Bob);
}

void FPendulumRod::ApplyVelocityImpulse(
	FBody& Body,
	const FVector2D& Impulse
) const
{
	// Impulse-momentum relation:
	//
	//     Impulse = m DeltaVelocity
	//
	// Therefore:
	//
	//     DeltaVelocity = Impulse / m
	//
	// Fixed-axis components are not modified because the corresponding
	// support reactions prevent velocity changes along those axes.

	if (Body.Mass <= UE_SMALL_NUMBER)
	{
		return;
	}

	if (!Body.bXFixed)
	{
		Body.Velocity.X +=
			Impulse.X / Body.Mass;
	}

	if (!Body.bYFixed)
	{
		Body.Velocity.Y +=
			Impulse.Y / Body.Mass;
	}
}

void FPendulumRod::ProjectPositions(
	TArray<FBody>& Bodies
)
{
	// Position-level constraint:
	//
	//     |x_B - x_P| = L0
	//
	// After integration, the provisional geometry may violate the rod length:
	//
	//     C = |x_B - x_P| - L0
	//
	// We apply equal and opposite radial position corrections:
	//
	//     Pivot: +DeltaLambda e_Radial
	//     Bob:   -DeltaLambda e_Radial
	//
	// The scalar correction is obtained from:
	//
	//     A DeltaLambda = C
	//
	// where A is the sum of the effective inverse masses in the radial
	// direction.
	//
	// This is a geometric correction, not a physical force or impulse.

	if (!Bodies.IsValidIndex(PivotBody) || !Bodies.IsValidIndex(BobBody))
	{
		return;
	}

	FBody& Pivot = Bodies[PivotBody];
	FBody& Bob = Bodies[BobBody];

	// Recompute geometry from the provisional post-integration state.
	UpdatePendulumKinematics(Pivot, Bob);

	if (
		PendulumPos.ComputedLength <= UE_SMALL_NUMBER ||
		PendulumPos.InitialLength <= UE_SMALL_NUMBER
		)
	{
		return;
	}

	const double LengthError =
		PendulumPos.ComputedLength
		- PendulumPos.InitialLength;

	const double PivotEffectiveInverseMass =
		ComputeEffectiveInverseMass(
			Pivot,
			PolarBase.e_Radial
		);

	const double BobEffectiveInverseMass =
		ComputeEffectiveInverseMass(
			Bob,
			PolarBase.e_Radial
		);

	const double ConstraintCoefficientA =
		PivotEffectiveInverseMass
		+ BobEffectiveInverseMass;

	if (ConstraintCoefficientA <= UE_SMALL_NUMBER)
	{
		return;
	}

	const double PositionCorrectionMagnitude =
		LengthError / ConstraintCoefficientA;

	const FVector2D PositionCorrection =
		PositionCorrectionMagnitude
		* PolarBase.e_Radial;

	ApplyPositionCorrection(
		Pivot,
		PositionCorrection
	);

	ApplyPositionCorrection(
		Bob,
		-PositionCorrection
	);

	// Refresh diagnostics after correction.
	UpdatePendulumKinematics(Pivot, Bob);
}

void FPendulumRod::ApplyPositionCorrection(
	FBody& Body,
	const FVector2D& Correction
) const
{
	if (Body.Mass <= UE_SMALL_NUMBER)
	{
		return;
	}

	if (!Body.bXFixed)
	{
		Body.Position.X +=
			Correction.X / Body.Mass;
	}

	if (!Body.bYFixed)
	{
		Body.Position.Y +=
			Correction.Y / Body.Mass;
	}
}