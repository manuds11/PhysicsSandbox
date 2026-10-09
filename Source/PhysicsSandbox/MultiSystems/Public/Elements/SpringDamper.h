#pragma once

#include "CoreMinimal.h"
#include "Simulation/FullSysTypes.h"

class FSpringDamper : public ISysElement
{
public:
	FSpringDamper(
		int32 InBodyAIndex,
		int32 InBodyBIndex,
		double InStiffness,
		double InDamping,
		double InRestLength
	);

	virtual ESysElementType GetElementType() const override;

	virtual bool GetConnectedBodies(
		int32& OutBodyA,
		int32& OutBodyB
	) const override;

	virtual void ApplyForces(TArray<FBody>& Bodies) override;

	virtual double ComputePotentialEnergy(
		const TArray<FBody>& Bodies
	) const override;

	virtual double ComputeDissipatedPower(const TArray<FBody>& Bodies) const override;

	virtual FColor GetDebugColor() const override
	{
		return FColor::Purple;
	}

	virtual bool GetEquilibriumPoint(
		const TArray<FBody>& Bodies,
		FVector2D& OutPoint
	) const override;

private:
	int32 BodyAIndex = INDEX_NONE;
	int32 BodyBIndex = INDEX_NONE;

	double Stiffness = 20.0;	// N/m
	double Damping = 1.0;		// N*s/m
	double RestLength = 1.0;	// m
};