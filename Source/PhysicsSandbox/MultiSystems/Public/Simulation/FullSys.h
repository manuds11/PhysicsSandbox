#pragma once

#include "Simulation/FullSysTypes.h"
#include "Simulation/SysState.h"
#include "Simulation/SysDiagnostics.h"

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
	void SetForceInput(
		int32 ForceInputIndex,
		const FVector2D& Force
	);

	void SetIntegrator(TUniquePtr<ISysIntegrator> InIntegrator);
	void SetVelocityCorrectionsEnabled(bool bEnabled) { RodSys.bEnableVelocityCorrections = bEnabled; }
	void SetPositionCorrectionsEnabled(bool bEnabled) { RodSys.bEnablePositionCorrections = bEnabled; }

	bool Step(double Dt);

	const TArray<FBody>& GetBodies() const;
	const TArray<TUniquePtr<ISysElement>>& GetElements() const;
	const TArray<FPendulumRod*>& GetRodSystemArray() const {	return RodSys.GetRodSystemArray(); }
	const TArray<double>& GetRodSystemLambdaVector() const {	return RodSys.GetSystemLambdaVector(); }

	// Control
	FSysState BuildFullState() const;

	// ----------------------------------------------------------------------------- *****
	// DEMO CONTROL
	// -----------------------------------------------------------------------------
	bool bEnableDemoFeedbackControl = false;
	
	int32 Mass1ForceInputArrayIndex = INDEX_NONE;
	int32 Mass3ForceInputArrayIndex = INDEX_NONE;

	double Mass1ReferenceY = 0.0;
	double Mass3ReferenceX = 0.0;

	double Mass1KpY = 20.0;
	double Mass3KpX = 20.0;

	void UpdateDemoFeedbackControl();
	// ----------------------------------------------------------------------------- *****
	FSystemEnergyState ComputeSystemEnergy() const;

private:
	FRodSys RodSys;
	
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

	// Debug Helpers
	bool IsStateFinite() const;

	// -----------------------------------------------------------------------------
	// MultiRod System
	// -----------------------------------------------------------------------------
public:
	
	
};


