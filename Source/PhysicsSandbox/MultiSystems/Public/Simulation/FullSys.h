#pragma once

#include "Simulation/FullSysTypes.h"
#include "Simulation/SysIntegrator.h"
#include "Simulation/SubSys.h"
#include "Simulation/RodSys.h"

#include "CoreMinimal.h"

class FPendulumRod;

class FFullSys
{
public:
	FFullSys();
	void Initialize();

	FSubSys& CreateSubSys(FName Name);
	const TArray<FSubSys>& GetSubSystems() const;

	int32 AddBody(const FBody& Body);
	int32 AddElement(TUniquePtr<ISysElement> Element);

	void SetIntegrator(TUniquePtr<ISysIntegrator> InIntegrator);
	bool Step(double Dt);

	const TArray<FBody>& GetBodies() const;
	const TArray<TUniquePtr<ISysElement>>& GetElements() const;

private:
	TArray<FSubSys> SubSystems;
	TArray<FBody> Bodies;
	TArray<TUniquePtr<ISysElement>> Elements;

	TUniquePtr<ISysIntegrator> Integrator;

	bool bInitialized = false;

	void ClearForces();
	void ApplyNonConstraintInteractions();
	void ApplyGravity();
	void ComputeAccelerations();
	void Integrate(double Dt);
	void ApplyFixedAxes();

	bool IsStateFinite() const;

	// -----------------------------------------------------------------------------
	// MultiRod System
	// -----------------------------------------------------------------------------
public:
	const TArray<FPendulumRod*>&
		GetRodSystemArray() const
	{
		return RodSys.GetRodSystemArray();
	}

	const TArray<double>&
		GetRodSystemLambdaVector() const
	{
		return RodSys.GetSystemLambdaVector();
	}

private:
	FRodSys RodSys;
};


