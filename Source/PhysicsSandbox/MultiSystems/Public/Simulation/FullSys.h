#pragma once

#include "Simulation/FullSysTypes.h"
#include "Simulation/SysState.h"

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
	int32 AddForceInput(const FForceInput& ForceInput);

	void SetIntegrator(TUniquePtr<ISysIntegrator> InIntegrator);
	bool Step(double Dt);

	const TArray<FBody>& GetBodies() const;
	const TArray<TUniquePtr<ISysElement>>& GetElements() const;

	// Control
	FSysState BuildFullState() const;

private:
	TArray<FSubSys> SubSystems;
	TArray<FBody> Bodies;
	TArray<TUniquePtr<ISysElement>> Elements;
	TArray<FForceInput> ForceInputs;

	TUniquePtr<ISysIntegrator> Integrator;

	bool bInitialized = false;

	Eigen::VectorXd BuildXDot_Full() const;
	bool EvaluateDynamics(Eigen::VectorXd& OutXDot_Full);

	void ClearForces();
	void ApplyNonConstraintInteractions();
	void ApplyGravity();
	void ApplyForceInputs();
	void ComputeAccelerations();
	void Integrate(double Dt);
	void ApplyFixedAxes();

	bool IsStateFinite() const;

	// -----------------------------------------------------------------------------
	// MultiRod System
	// -----------------------------------------------------------------------------
public:
	const TArray<FPendulumRod*>& GetRodSystemArray() const
	{
		return RodSys.GetRodSystemArray();
	}

	const TArray<double>& GetRodSystemLambdaVector() const
	{
		return RodSys.GetSystemLambdaVector();
	}

private:
	FRodSys RodSys;
};


