#pragma once

#include "CoreMinimal.h"
#include "Simulation/FullSysTypes.h"

struct FRodPolarBase
{
	FVector2D e_Radial =
		FVector2D::ZeroVector;

	FVector2D e_Theta =
		FVector2D::ZeroVector;
};

struct FRodJacobian
{
	FVector2D JPivot =
		FVector2D::ZeroVector;

	FVector2D JBob =
		FVector2D::ZeroVector;

	double JDotVel = 0.0;
};

struct FPendulumPositions
{
	FVector2D Bob2Pivot =
		FVector2D::ZeroVector;

	double InitialLength = 0.0;
	double ComputedLength = 0.0;

	// Signed length error relative to the target length.
	double LengthAbsError = 0.0;
	double LengthRelError = 0.0;

	// Signed length increment between consecutive physics steps.
	double PreviousLength = 0.0;
	double StepLengthIncrement = 0.0;
	double StepMeanLengthIncrement = 0.0;

	bool bLengthInitialized = false;
	bool bHasPreviousLengthSample = false;
};

struct FPendulumVelocities
{
	FVector2D Bob2Pivot =
		FVector2D::ZeroVector;

	double Bob2PivotRadial = 0.0;
	double Bob2PivotTangential = 0.0;

	double MeanRadialVelocity = 0.0;
};

struct FRodConstraintForces
{
	FVector2D PivotForce =
		FVector2D::ZeroVector;

	FVector2D BobForce =
		FVector2D::ZeroVector;
};

// -----------------------------------------------------------------------------
// Pendulum rod
// -----------------------------------------------------------------------------

class FPendulumRod : public ISysElement
{
public:

	// -------------------------------------------------------------------------
	// Construction
	// -------------------------------------------------------------------------

	FPendulumRod(
		int32 InPivotBody,
		int32 InBobBody
	);

	// -------------------------------------------------------------------------
	// ISysElement interface
	// -------------------------------------------------------------------------

	virtual ESysElementType GetElementType() const override;

	virtual bool GetConnectedBodies(
		int32& OutBodyA,
		int32& OutBodyB
	) const override;

	virtual void ApplyForces(
		TArray<FBody>& Bodies
	) override;

	// -------------------------------------------------------------------------
	// Rod state update
	// -------------------------------------------------------------------------

	void UpdateRodJacobian(
		const TArray<FBody>& Bodies
	);

	void UpdateConstraintForces(
		double Lambda
	);

	const FPendulumPositions& GetPendulumPositions() const
	{
		return PendulumPos;
	}

	const FPendulumVelocities& GetPendulumVelocities() const
	{
		return PendulumVel;
	}

	const FRodConstraintForces& GetConstraintForces() const
	{
		return RodConstraintForces;
	}

	const FRodJacobian& GetRodJacobian() const
	{
		return RodJacobian;
	}

	double GetJDotV() const
	{
		return RodJacobian.JDotVel;
	}

	double GetConstraintFunctionValue() const
	{
		return PendulumPos.LengthAbsError;
	}

private:

	// -------------------------------------------------------------------------
	// Rod state update helpers
	// -------------------------------------------------------------------------

	void InitializePendulumLength();

	void UpdatePendulumKinematics(
		const FBody& Pivot,
		const FBody& Bob
	);

	FRodPolarBase ComputePolarBase() const;

	void UpdateErrorData();

	// -------------------------------------------------------------------------
	// Connectivity
	// -------------------------------------------------------------------------

	int32 PivotBody = INDEX_NONE;
	int32 BobBody = INDEX_NONE;

	// -------------------------------------------------------------------------
	// Runtime state
	// -------------------------------------------------------------------------

	FRodPolarBase PolarBase;
	FRodJacobian RodJacobian;

	FPendulumPositions PendulumPos;
	FPendulumVelocities PendulumVel;

	FRodConstraintForces RodConstraintForces;

	// -------------------------------------------------------------------------
	// Diagnostics
	// -------------------------------------------------------------------------

	uint64 ErrorDataSampleCount = 0;
};