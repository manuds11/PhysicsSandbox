#pragma once

#include "CoreMinimal.h"
#include "Simulation/FullSysTypes.h"

class FSpringDamper : public ISysElement
{
public:
	FSpringDamper(
		int32 InBodyA,
		int32 InBodyB,
		double InStiffness,
		double InDamping,
		double InRestLength
	);

	virtual void ApplyForces(TArray<FBody>& Bodies) const override;

private:
	int32 BodyA = INDEX_NONE;
	int32 BodyB = INDEX_NONE;

	double Stiffness = 100.0;
	double Damping = 1.0;
	double RestLength = 100.0;
};