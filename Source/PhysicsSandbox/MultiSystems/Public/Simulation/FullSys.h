#pragma once

#include "Simulation/FullSysTypes.h"
#include "Simulation/SysIntegrator.h"
#include "Simulation/SubSys.h"

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
	void ApplyElementInteractions();
	void ApplyGravity();
	void ComputeAccelerations();
	void Integrate(double Dt);
	void ApplyFixedAxes();
	void ProjectConstraintVelocities();
	void ProjectConstraintPositions();

	// -----------------------------------------------------------------------------
	// MultiRod System
	// -----------------------------------------------------------------------------

	TArray<FPendulumRod*> RodSystemArray;	// Non-owning pointers to rod elements stored in Elements.

	TArray<double> RodSysMatrixA;
	TArray<double> RodSysVectorB;
	TArray<double> RodSysVectorLambda;

	void AssembleRodSystem();       // Una vez
	void UpdateRodSystemValues();   // Cada step
	
	// Helpers
	void CollectRods();
	void UpdateRodSystemJacobians();

	double& MatrixIndex2ArrayIndex(int32 Row, int32 Column);
	const double& MatrixIndex2ArrayIndex(int32 Row, int32 Column) const;
};


