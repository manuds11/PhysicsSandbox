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
// Constraint force and impulse
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

void FPendulumRod::UpdateConstraintImpulses(
	double Eta
)
{
	/*
	 * Constraint impulse contribution of this rod:
	 *
	 * I_c,i = J_i^T P_i
	 */

	RodConstraintImpulses.PivotImpulse =
		RodJacobian.JPivot
		* Eta;

	RodConstraintImpulses.BobImpulse =
		RodJacobian.JBob
		* Eta;
}

void FPendulumRod::UpdatePositionCorrections(
	double Mu
)
{
	/*
	 * Position-constraint contribution of this rod:
	 *
	 * DeltaQ_i = J_i^T Mu_i
	 *
	 * The inverse mass is applied later when the
	 * correction is transferred to each body:
	 *
	 * Deltaq = M^-1 J^T Mu
	 */

	RodPositionCorrections.PivotCorrection =
		RodJacobian.JPivot
		* Mu;

	RodPositionCorrections.BobCorrection =
		RodJacobian.JBob
		* Mu;
}

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

void FPendulumRod::ApplyImpulses2Bodies(
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
	// Pivot impulse
	// -------------------------------------------------------------------------

	if (Pivot.Mass > UE_SMALL_NUMBER)
	{
		const double PivotInverseMass =
			1.0 / Pivot.Mass;

		if (!Pivot.bXFixed)
		{
			Pivot.Velocity.X +=
				RodConstraintImpulses.PivotImpulse.X
				* PivotInverseMass;
		}

		if (!Pivot.bYFixed)
		{
			Pivot.Velocity.Y +=
				RodConstraintImpulses.PivotImpulse.Y
				* PivotInverseMass;
		}
	}

	// -------------------------------------------------------------------------
	// Bob impulse
	// -------------------------------------------------------------------------

	if (Bob.Mass > UE_SMALL_NUMBER)
	{
		const double BobInverseMass =
			1.0 / Bob.Mass;

		if (!Bob.bXFixed)
		{
			Bob.Velocity.X +=
				RodConstraintImpulses.BobImpulse.X
				* BobInverseMass;
		}

		if (!Bob.bYFixed)
		{
			Bob.Velocity.Y +=
				RodConstraintImpulses.BobImpulse.Y
				* BobInverseMass;
		}
	}
}

void FPendulumRod::ApplyPosCorrections2Bodies(
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
	// Pivot
	// -------------------------------------------------------------------------

	if (Pivot.Mass > UE_SMALL_NUMBER)
	{
		const double InverseMass =
			1.0 / Pivot.Mass;

		if (!Pivot.bXFixed)
		{
			Pivot.Position.X +=
				RodPositionCorrections.PivotCorrection.X
				* InverseMass;
		}

		if (!Pivot.bYFixed)
		{
			Pivot.Position.Y +=
				RodPositionCorrections.PivotCorrection.Y
				* InverseMass;
		}
	}

	// -------------------------------------------------------------------------
	// Bob
	// -------------------------------------------------------------------------

	if (Bob.Mass > UE_SMALL_NUMBER)
	{
		const double InverseMass =
			1.0 / Bob.Mass;

		if (!Bob.bXFixed)
		{
			Bob.Position.X +=
				RodPositionCorrections.BobCorrection.X
				* InverseMass;
		}

		if (!Bob.bYFixed)
		{
			Bob.Position.Y +=
				RodPositionCorrections.BobCorrection.Y
				* InverseMass;
		}
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