#include "Elements/PendulumRod.h"

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

// -----------------------------------------------------------------------------
// ISysElement interface
// -----------------------------------------------------------------------------

void FPendulumRod::ApplyForces(
	TArray<FBody>& Bodies
)
{
	if (
		!Bodies.IsValidIndex(PivotBody)
		|| !Bodies.IsValidIndex(BobBody)
		)
	{
		return;
	}

	FBody& Pivot =
		Bodies[PivotBody];

	FBody& Bob =
		Bodies[BobBody];

	// -------------------------------------------------------------------------
	// Pivot constraint force
	// -------------------------------------------------------------------------

	if (!Pivot.bXFixed)
	{
		Pivot.NetForce.X +=
			RodConstraintForces.PivotForce.X;
	}

	if (!Pivot.bYFixed)
	{
		Pivot.NetForce.Y +=
			RodConstraintForces.PivotForce.Y;
	}

	// -------------------------------------------------------------------------
	// Bob constraint force
	// -------------------------------------------------------------------------

	if (!Bob.bXFixed)
	{
		Bob.NetForce.X +=
			RodConstraintForces.BobForce.X;
	}

	if (!Bob.bYFixed)
	{
		Bob.NetForce.Y +=
			RodConstraintForces.BobForce.Y;
	}
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

void FPendulumRod::InitializePendulumLength()
{
	if (PendulumPos.ComputedLength <= UE_SMALL_NUMBER)
	{
		return;
	}

	PendulumPos.InitialLength =
		PendulumPos.ComputedLength;

	PendulumPos.bLengthInitialized =
		true;
}

// -----------------------------------------------------------------------------
// Constraint state update
// -----------------------------------------------------------------------------

void FPendulumRod::UpdateRodState(
	const TArray<FBody>& Bodies
)
{
	if (
		!Bodies.IsValidIndex(PivotBody)
		|| !Bodies.IsValidIndex(BobBody)
		)
	{
		RodJacobian =
			FRodJacobian();

		return;
	}

	const FBody& Pivot =
		Bodies[PivotBody];

	const FBody& Bob =
		Bodies[BobBody];

	// -------------------------------------------------------------------------
	// Position state
	// -------------------------------------------------------------------------

	PendulumPos.Bob2Pivot =
		Bob.Position
		- Pivot.Position;

	PendulumPos.ComputedLength =
		PendulumPos.Bob2Pivot.Size();

	if (!PendulumPos.bLengthInitialized)
	{
		InitializePendulumLength();
	}

	if (
		PendulumPos.ComputedLength
		<= UE_SMALL_NUMBER
		)
	{
		PolarBase =
			FRodPolarBase();

		RodJacobian =
			FRodJacobian();

		PendulumVel =
			FPendulumVelocities();

		return;
	}

	// -------------------------------------------------------------------------
	// Polar base
	// -------------------------------------------------------------------------

	PolarBase =
		ComputePolarBase();

	// -------------------------------------------------------------------------
	// Velocity state
	// -------------------------------------------------------------------------

	PendulumVel.Bob2Pivot =
		Bob.Velocity
		- Pivot.Velocity;

	PendulumVel.Bob2PivotRadial =
		FVector2D::DotProduct(
			PendulumVel.Bob2Pivot,
			PolarBase.e_Radial
		);

	PendulumVel.Bob2PivotTangential =
		FVector2D::DotProduct(
			PendulumVel.Bob2Pivot,
			PolarBase.e_Theta
		);

	// -------------------------------------------------------------------------
	// Constraint Jacobian
	// -------------------------------------------------------------------------

	// e_Radial is the constraint normal pointing from pivot to bob.
	RodJacobian.JPivot =
		-PolarBase.e_Radial;

	RodJacobian.JBob =
		PolarBase.e_Radial;

	/*
	 * JDotVel = v_t² / |r|
	 *
	 * The current computed length must be used rather than the target length,
	 * since the instantaneous angular velocity is:
	 *
	 * Omega = v_t / |r|
	 */
	RodJacobian.JDotVel =
		FMath::Square(
			PendulumVel.Bob2PivotTangential
		)
		/ PendulumPos.ComputedLength;
}

FRodPolarBase FPendulumRod::ComputePolarBase() const
{
	FRodPolarBase ComputedPolarBase;

	if (PendulumPos.ComputedLength <= UE_SMALL_NUMBER)
	{
		return ComputedPolarBase;
	}

	ComputedPolarBase.e_Radial =
		PendulumPos.Bob2Pivot
		/ PendulumPos.ComputedLength;

	// e_Theta is e_Radial rotated 90 degrees counter-clockwise.
	ComputedPolarBase.e_Theta =
		FVector2D(
			-ComputedPolarBase.e_Radial.Y,
			ComputedPolarBase.e_Radial.X
		);

	return ComputedPolarBase;
}


// -----------------------------------------------------------------------------
// Constraint forces
// -----------------------------------------------------------------------------

void FPendulumRod::UpdateConstraintForces(
	double Lambda
)
{
	/*
	 * Constraint-force contribution of this rod:
	 *
	 * Q_c,i = J_i^T Lambda_i
	 *
	 * Each Jacobian block generates the force applied
	 * to its corresponding connected body.
	 */

	RodConstraintForces.PivotForce =
		RodJacobian.JPivot
		* Lambda;

	RodConstraintForces.BobForce =
		RodJacobian.JBob
		* Lambda;
}

// -----------------------------------------------------------------------------
// Diagnostics
// -----------------------------------------------------------------------------

void FPendulumRod::ComputeInstantErrorData()
{
	if (PendulumPos.InitialLength <= UE_SMALL_NUMBER)
	{
		InstantErrorData =
			FInstantErrorData();

		return;
	}

	InstantErrorData.LengthAbsError =
		PendulumPos.ComputedLength
		- PendulumPos.InitialLength;

	InstantErrorData.LengthRelError =
		InstantErrorData.LengthAbsError
		/ PendulumPos.InitialLength;

	InstantErrorData.RadialVelocityError =
		PendulumVel.Bob2PivotRadial;
}

void FPendulumRod::UpdateStatisticalErrorData()
{
	if (
		!PendulumPos.bLengthInitialized
		|| PendulumPos.InitialLength <= UE_SMALL_NUMBER
		)
	{
		return;
	}

	if (!StatisticalErrorData.bHasPreviousLengthSample)
	{
		StatisticalErrorData.PreviousLength =
			PendulumPos.ComputedLength;

		StatisticalErrorData.StepLengthIncrement =
			0.0;

		StatisticalErrorData.bHasPreviousLengthSample =
			true;

		return;
	}

	StatisticalErrorData.StepLengthIncrement =
		PendulumPos.ComputedLength
		- StatisticalErrorData.PreviousLength;

	StatisticalErrorData.PreviousLength =
		PendulumPos.ComputedLength;

	++StatisticalErrorData.SampleCount;

	const double SampleCount =
		static_cast<double>(
			StatisticalErrorData.SampleCount
			);

	StatisticalErrorData.MeanStepLengthIncrement +=
		(
			StatisticalErrorData.StepLengthIncrement
			- StatisticalErrorData.MeanStepLengthIncrement
			)
		/ SampleCount;

	StatisticalErrorData.MeanRadialVelocity +=
		(
			PendulumVel.Bob2PivotRadial
			- StatisticalErrorData.MeanRadialVelocity
			)
		/ SampleCount;
}