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
	void Step(double Dt);

	const TArray<FBody>& GetBodies() const;
	const TArray<TUniquePtr<ISysElement>>& GetElements() const;

private:
	TArray<FSubSys> SubSystems;
	TArray<FBody> Bodies;
	TArray<TUniquePtr<ISysElement>> Elements;

	TUniquePtr<ISysIntegrator> Integrator;

	void ClearForces();
	void ApplyNonConstraintInteractions();
	void ApplyGravity();
	void ComputeAccelerations();
	void Integrate(double Dt);
	void ApplyFixedAxes();

	// Obsolete: Constraint projection is now handled by the rod system.
	void ProjectConstraintVelocities();
	void ProjectConstraintPositions();

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


