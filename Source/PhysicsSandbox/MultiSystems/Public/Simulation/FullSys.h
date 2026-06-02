#pragma once

#include "CoreMinimal.h"
#include "Simulation/FullSysTypes.h"

class FFullSys
{
public:
	int32 AddBody(const FBody& Body);

	void AddElement(TUniquePtr<ISysElement> Element);

	void Step(double Dt);

	const TArray<FBody>& GetBodies() const;

private:
	TArray<FBody> Bodies;
	TArray<TUniquePtr<ISysElement>> Elements;

	void ClearForces();
	void ApplyElements();
	void ComputeAccelerations();
	void Integrate(double Dt);
	void ApplyFixedAxes();
};