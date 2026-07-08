#pragma once

#include "CoreMinimal.h"
#include "Simulation/FullSysTypes.h"
#include "Math/Units.h"

struct FPolarBase
{
	FVector2D e_Radial = FVector2D::ZeroVector;
	FVector2D e_Theta = FVector2D::ZeroVector;

	FVector2D e_RadialX = FVector2D::ZeroVector;
	FVector2D e_RadialY = FVector2D::ZeroVector;

	FVector2D e_ThetaX = FVector2D::ZeroVector;
	FVector2D e_ThetaY = FVector2D::ZeroVector;	// Projection into cartesian base
};

struct FPendulumVelocities
{
	FVector2D Bob2Pivot = FVector2D::ZeroVector;
	double Bob2PivotTangential = 0.0;
	double Bob2PivotRadial = 0.0;
};

struct FPendulumPositions
{
	FVector2D Bob2Pivot = FVector2D::ZeroVector;
	double ComputedLength = 0.0;
};

class FPendulumRod : public ISysElement
{
public:
	FPendulumRod(
		int32 InPivotBody,
		int32 InBobBody,
		double InTheoreticalLength
	);

	virtual bool GetConnectedBodies(
		int32& OutBodyA,
		int32& OutBodyB
	) const override;

	virtual void ApplyForces(
		TArray<FBody>& Bodies
	) const override;

private:
	void UpdatePendulumKinematics(
		const FBody& Pivot,
		const FBody& Bob
	) const;

	FPolarBase ComputePolarBase() const;

	FVector2D ComputeFreeAcceleration(
		const FBody& Body
	) const;

	double ComputeEffectiveInverseMass(
		const FBody& Body,
		const FVector2D& e_Radial
	) const;

	double ComputeTension(
		const FBody& Pivot,
		const FBody& Bob,
		const FPolarBase& InPolarBase
	) const;

private:
	int32 PivotBody = INDEX_NONE;
	int32 BobBody = INDEX_NONE;

	double TheoreticalLength = 1.0;

	mutable FPendulumPositions PendulumPos;
	mutable FPendulumVelocities PendulumVel;
	mutable FPolarBase PolarBase;
};