#pragma once

#include "CoreMinimal.h"
#include "Simulation/FullSysTypes.h"

struct FPolarBase
{
	FVector2D e_Radial = FVector2D::ZeroVector;
	FVector2D e_Theta = FVector2D::ZeroVector;
};

class FPendulumRod : public ISysElement
{
public:
	FPendulumRod(
		int32 InPivotBody,
		int32 InBobBody,
		double InLength
	);

	virtual bool GetConnectedBodies(
		int32& OutBodyA,
		int32& OutBodyB
	) const override;

	virtual void ApplyForces(
		TArray<FBody>& Bodies
	) const override;

private:
	FPolarBase ComputePolarBase(
		const FBody& Pivot,
		const FBody& Bob,
		double& CurrentLength
	) const;

	FVector2D ComputeRelativeVelocity(
		const FBody& Pivot,
		const FBody& Bob
	) const;

	double ComputeTangentialSpeed(
		const FVector2D& RelativeVelocity,
		const FPolarBase& PolarBase
	) const;

	double ComputeTension(
		const FBody& Pivot,
		const FBody& Bob,
		const FPolarBase& PolarBase,
		double CurrentLengthMod,
		double TangentialSpeedModSquared
	) const;

private:
	int32 PivotBody = INDEX_NONE;
	int32 BobBody = INDEX_NONE;

	double Length = 1.0;
};