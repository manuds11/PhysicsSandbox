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
	void SetVelocityCorrection1Enabled(bool bEnabled) { RodSys.bEnableVelocityCorrection1 = bEnabled; }
	void SetPositionCorrectionsEnabled(bool bEnabled) { RodSys.bEnablePositionCorrection = bEnabled; }
	void SetVelocityCorrection2Enabled(bool bEnabled) { RodSys.bEnableVelocityCorrection2 = bEnabled; }

	bool Step(double Dt);

	const TArray<FBody>& GetBodies() const;
	const TArray<TUniquePtr<ISysElement>>& GetElements() const;
	const TArray<FPendulumRod*>& GetRodSystemArray() const {	return RodSys.GetRodSystemArray(); }
	const TArray<double>& GetRodSystemLambdaVector() const {	return RodSys.GetSystemLambdaVector(); }

	// Control
	FSysState BuildFullState() const;
	FSysStateDerivative BuildFullStateDerivative(
		const Eigen::VectorXd& InQDot_Full
	) const;

	// -----------------------------------------------------------------------------
	// Energy Balance
	// -----------------------------------------------------------------------------
	const FSystemEnergyBalance& GetEnergyBalance() const { return EnergyBalance; }
	FSystemEnergyState ComputeSystemEnergy() const;

	const FDeltaEnergy& GetIntegrationEnergy() const { return IntegrationEnergy; }
	const FConstraintCorrectionEnergyDiagnostics& GetCorrectionEnergyDiagnostics() const
	{
		return RodSys.GetCorrectionEnergyDiagnostics();
	}

private:
	FRodSys RodSys;
	
	TArray<FSubSys> SubSystems;

	TArray<FBody> Bodies;
	TArray<TUniquePtr<ISysElement>> Elements;
	TArray<FForceInput> ForceInputs;

	TUniquePtr<ISysIntegrator> Integrator;

	bool bInitialized = false;

	bool EvaluateDynamics();

	void ClearForces();
	void ApplyNonConstraintInteractions();
	void ApplyGravity();
	void ApplyForceInputs();
	void ComputeAccelerations();
	void Integrate(double Dt);
	void ApplyFixedAxes();

	// Debug Helpers
	bool IsStateFinite() const;

	// Energy Diagnostics
	FSystemEnergyBalance EnergyBalance;
	FDeltaEnergy IntegrationEnergy;
	double ComputeDissipatedPower() const;
	void UpdateEnergyBalance(double Dt, double DissipatedPowerBefore);
};


