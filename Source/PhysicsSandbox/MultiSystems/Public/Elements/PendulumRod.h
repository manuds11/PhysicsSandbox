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

	bool bLengthInitialized = false;
};

struct FPendulumVelocities
{
	FVector2D Bob2Pivot =
		FVector2D::ZeroVector;

	double Bob2PivotRadial = 0.0;
	double Bob2PivotTangential = 0.0;
};

struct FRodConstraintForces
{
	FVector2D PivotForce =
		FVector2D::ZeroVector;

	FVector2D BobForce =
		FVector2D::ZeroVector;
};

struct FRodConstraintImpulses
{
	FVector2D PivotImpulse =
		FVector2D::ZeroVector;

	FVector2D BobImpulse =
		FVector2D::ZeroVector;
};

struct FInstantErrorData
{
	double LengthAbsError = 0.0;
	double LengthRelError = 0.0;

	double RadialVelocityError = 0.0;
};

struct FStatisticalErrorData
{
	double PreviousLength = 0.0;

	double StepLengthIncrement = 0.0;
	double MeanStepLengthIncrement = 0.0;

	double MeanRadialVelocity = 0.0;

	uint64 SampleCount = 0;

	bool bHasPreviousLengthSample = false;
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

	void ApplyImpulses(
		TArray<FBody>& Bodies
	);

	// -------------------------------------------------------------------------
	// Rod state update
	// -------------------------------------------------------------------------

	void UpdateRodState(
		const TArray<FBody>& Bodies
	);

	void UpdateConstraintForces(
		double Lambda
	);

	void UpdateConstraintImpulses(
		double Impulse
	);

	void ComputeInstantErrorData();

	void UpdateStatisticalErrorData();

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
		return InstantErrorData.LengthAbsError;
	}

	double GetVelocityConstraintError() const
	{
		return InstantErrorData.RadialVelocityError;
	}

	const FRodConstraintImpulses& GetConstraintImpulses() const
	{
		return RodConstraintImpulses;
	}

	const FInstantErrorData& GetInstantErrorData() const
	{
		return InstantErrorData;
	}

	const FStatisticalErrorData& GetStatisticalErrorData() const
	{
		return StatisticalErrorData;
	}

private:

	// -------------------------------------------------------------------------
	// Rod state update helpers
	// -------------------------------------------------------------------------

	void InitializePendulumLength();

	FRodPolarBase ComputePolarBase() const;

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
	FRodConstraintImpulses RodConstraintImpulses;

	FInstantErrorData InstantErrorData;
	FStatisticalErrorData StatisticalErrorData;

	// -------------------------------------------------------------------------
	// Diagnostics
	// -------------------------------------------------------------------------

	uint64 ErrorDataSampleCount = 0;
};