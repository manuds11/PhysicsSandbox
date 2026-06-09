#pragma once

#include "Simulation/FullSysTypes.h"
#include "Simulation/SysIntegrator.h"

#include "CoreMinimal.h"

class FFullSys
{
public:
	FFullSys();

	int32 AddBody(const FBody& Body);

	void AddElement(TUniquePtr<ISysElement> Element);

	void SetIntegrator(TUniquePtr<ISysIntegrator> InIntegrator);

	void Step(double Dt);

	const TArray<FBody>& GetBodies() const;

private:
	TArray<FBody> Bodies;
	TArray<TUniquePtr<ISysElement>> Elements;

	TUniquePtr<ISysIntegrator> Integrator;

	void ClearForces();
	void ApplyElementInteractions();
	void ComputeAccelerations();
	void Integrate(double Dt);
	void ApplyFixedAxes();
};