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

	void SetIntegrator(TUniquePtr<ISysIntegrator> InIntegrator);
	void SetVelocityCorrection1Enabled(bool bEnabled) { RodSys.bEnableVelocityCorrection1 = bEnabled; }
	void SetPositionCorrectionsEnabled(bool bEnabled) { RodSys.bEnablePositionCorrection = bEnabled; }
	void SetVelocityCorrection2Enabled(bool bEnabled) { RodSys.bEnableVelocityCorrection2 = bEnabled; }

	bool EvaluateXDot(
		const FSysState& InputX_Full,
		const Eigen::VectorXd& Tau_FR,
		const TArray<FU_ExtForce>& U_ExtForces,
		FSysStateDerivative& OutputXDot_Full
	);
	bool EvaluateNextXState(
		double Dt,
		const FSysState& InputX_Full,
		const Eigen::VectorXd& Tau_FR,
		const TArray<FU_ExtForce>& U_ExtForces,
		FSysState& OutputXNext_Full,
		bool bRecordEnergyDiagnostics = false
	);
	bool Step(
		double Dt,
		const FSysState& InputX_Full,
		const Eigen::VectorXd& Tau_FR,
		const TArray<FU_ExtForce>& U_ExtForces,
		FSysState& OutputXNext_Full
	);

	const TArray<FBody>& GetBodies() const;
	const FStateDOFIndexMap& GetStateIndexMap() const {	return StateIndexMap;	}

	const TArray<TUniquePtr<ISysElement>>& GetElements() const;
	const TArray<FPendulumRod*>& GetRodSystemArray() const {	return RodSys.GetRodSystemArray(); }
	const TArray<double>& GetRodSystemLambdaVector() const {	return RodSys.GetSystemLambdaVector(); }

	// -----------------------------------------------------------------------------
	// Control
	// -----------------------------------------------------------------------------
	FSysState BodiesToFullStateStruct() const;
	FSysStateDerivative BuildFullStateDerivative(
		const Eigen::VectorXd& InQDot_Full
	) const;

	void SetXState2Bodies(
		const FSysState& InputX_Full
	);

	void SyncRodStates();

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
	FStateDOFIndexMap StateIndexMap;

	
	TArray<TUniquePtr<ISysElement>> Elements;

	TUniquePtr<ISysIntegrator> Integrator;

	bool bInitialized = false;
	
	void BuildStateIndexMap();

	void ClearForces();
	void ApplyNonConstraintInteractions();
	void ApplyGravity();
	void ApplyU_ExtForces(const TArray<FU_ExtForce>& U_ExtForces);
	void ApplyTau_FR(const Eigen::VectorXd& Tau_FR);
	void ComputeAccelerations();
	void Integrate(double Dt);
	void ApplyFixedAxes();

	// Debug Helpers
	bool IsStateFinite() const;

	// Energy Diagnostics
	FSystemEnergyBalance EnergyBalance;
	FDeltaEnergy IntegrationEnergy;
	double ComputeDissipatedPower() const;

	double ComputeControlWorkIncrement(
		const FSysState& InputX_Full,
		const FSysState& OutputXNext_Full,
		const Eigen::VectorXd& Tau_FR
	) const;


	void UpdateEnergyBalance(
		double Dt,
		double DissipatedPowerBefore,
		double ControlWorkIncrement
	);
};


