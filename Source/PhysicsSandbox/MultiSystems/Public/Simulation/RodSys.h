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

	void BuildEmptyStructure(
		const TArray<TUniquePtr<ISysElement>>& Elements
	);

	bool RunConstraintForces(
		TArray<FBody>& Bodies
	);

	bool RunConstraintCorrections(
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

	// Forces - Acceleration
	TArray<double> BVector;
	TArray<double> LambdaVector;
	
	// Impulses - Velocities
	TArray<double> VelocityBVector;
	TArray<double> ImpulseEtaVector;

	// Pos corrections
	TArray<double> PositionBVector;
	TArray<double> PositionMuVector;


	static constexpr int32 MaxPositionCorrectionIterations = 5;
	static constexpr double PositionCorrectionTolerance = 1.0e-6; // m

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
	void UpdateRodStates(
		const TArray<FBody>& Bodies
	);

	void ForcesSysUpdate(const TArray<FBody>& Bodies);
	void ImpulseCorrectionSysUpdate(const TArray<FBody>& Bodies);
	void PosCorrectionSysUpdate(const TArray<FBody>& Bodies);

	bool SolveLambdaForce();
	bool SolveEtaVelCorrection();
	bool SolveMuPosCorrection();

	void ApplyAllForces(TArray<FBody>& Bodies);
	void ApplyAllVelocityProjections(TArray<FBody>& Bodies);
	void ApplyAllPositionProjections(TArray<FBody>& Bodies);

	void UpdateAMatrix(const TArray<FBody>& Bodies);

	double ComputeA_ij(
		const FPendulumRod& Rod_i,
		const FPendulumRod& Rod_j,
		const TArray<FBody>& Bodies
	) const;

	void UpdateForceBVector(const TArray<FBody>& Bodies);
	void UpdateVelocityBVector();
	void UpdatePositionBVector();

	double ComputeMaxPositionConstraintError() const;

	double ComputeForceB_i(
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