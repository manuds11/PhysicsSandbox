#pragma once

#include "CoreMinimal.h"
#include "Simulation/FullSysTypes.h"

class FPendulumRod;

class FRodSys
{
public:

	// -------------------------------------------------------------------------
	// Public API
	// -------------------------------------------------------------------------

	void Assemble(
		const TArray<TUniquePtr<ISysElement>>& Elements
	);

	void Update(
		const TArray<FBody>& Bodies
	);

	bool Solve();

	void ApplyRodConstraintForces(
		TArray<FBody>& Bodies
	);

	void UpdateStatisticalErrorData();

	// Getters
	const TArray<FPendulumRod*>& GetRodSystemArray() const
	{
		return RodSystemArray;
	}

	const TArray<double>& GetSystemLambdaVector() const
	{
		return LambdaVector;
	}

private:

	// -------------------------------------------------------------------------
	// MultiRod System
	// -------------------------------------------------------------------------

	// Non-owning pointers to rod elements stored in FFullSys::Elements.
	TArray<FPendulumRod*> RodSystemArray;

	TArray<double> AMatrix;
	TArray<double> BVector;
	TArray<double> LambdaVector;

	// -------------------------------------------------------------------------
	// Helpers
	// -------------------------------------------------------------------------

	struct FRodExtreme
	{
		int32 BodyIndex = INDEX_NONE;
		FVector2D Jacobian = FVector2D::ZeroVector;
	};

	void CollectRods(
		const TArray<TUniquePtr<ISysElement>>& Elements
	);

	void UpdateJacobians(
		const TArray<FBody>& Bodies
	);

	void UpdateAMatrix(
		const TArray<FBody>& Bodies
	);

	double ComputeA_ij(
		const FPendulumRod& Rod_i,
		const FPendulumRod& Rod_j,
		const TArray<FBody>& Bodies
	) const;

	void UpdateBVector(
		const TArray<FBody>& Bodies
	);

	double ComputeB_i(
		const FPendulumRod& Rod_i,
		const TArray<FBody>& Bodies
	) const;

	double& MatrixIndex2ArrayIndex(
		int32 Row,
		int32 Column
	);

	const double& MatrixIndex2ArrayIndex(
		int32 Row,
		int32 Column
	) const;
};