#include "Simulation/RodSys.h"

#include "Elements/PendulumRod.h"
#include "Math/DenseLinearSolver.h"

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

void FRodSys::Assemble(
	const TArray<TUniquePtr<ISysElement>>& Elements
)
{
	CollectRods(
		Elements
	);

	const int32 NumRods =
		RodSystemArray.Num();

	AMatrix.SetNumZeroed(
		NumRods * NumRods
	);

	BVector.SetNumZeroed(
		NumRods
	);

	LambdaVector.SetNumZeroed(
		NumRods
	);
}

void FRodSys::Update(
	const TArray<FBody>& Bodies
)
{
	if (RodSystemArray.IsEmpty())
	{
		return;
	}

	const int32 NumRods = RodSystemArray.Num();

	check(
		AMatrix.Num() == NumRods * NumRods
	);

	check(
		BVector.Num() == NumRods
	);

	check(
		LambdaVector.Num() == NumRods
	);

	for (double& Value : AMatrix)
	{
		Value = 0.0;
	}

	for (double& Value : BVector)
	{
		Value = 0.0;
	}

	for (double& Value : LambdaVector)
	{
		Value = 0.0;
	}

	UpdateJacobians(Bodies);

	UpdateAMatrix(Bodies);

	UpdateBVector(Bodies);
}

bool FRodSys::Solve()
{
	if (RodSystemArray.IsEmpty())
	{
		return true;
	}

	return DenseLinearSolver::Solve(
		AMatrix,
		BVector,
		LambdaVector
	);
}

void FRodSys::ApplyRodConstraintForces(
	TArray<FBody>& Bodies
)  
{
	if (RodSystemArray.IsEmpty())
	{
		return;
	}

	const int32 NumRods = RodSystemArray.Num();

	check(
		LambdaVector.Num() == NumRods
	);

	for (
		int32 Rod_iIndex = 0;
		Rod_iIndex < NumRods;
		++Rod_iIndex
		)
	{
		FPendulumRod* Rod_i =
			RodSystemArray[Rod_iIndex];

		check(Rod_i);

		const double Lambda_i =
			LambdaVector[Rod_iIndex];

		Rod_i->UpdateConstraintForces(
			Lambda_i
		);

		Rod_i->ApplyForces(
			Bodies
		);
	}
}

// -----------------------------------------------------------------------------
// Rod collection and Jacobians
// -----------------------------------------------------------------------------

void FRodSys::CollectRods(
	const TArray<TUniquePtr<ISysElement>>& Elements
)
{
	RodSystemArray.Reset();

	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		if (!Element)
		{
			continue;
		}

		if (
			Element->GetElementType()
			!= ESysElementType::PendulumRod
			)
		{
			continue;
		}

		FPendulumRod* PendulumRod =
			static_cast<FPendulumRod*>(
				Element.Get()
				);

		RodSystemArray.Add(
			PendulumRod
		);
	}
}

void FRodSys::UpdateJacobians(
	const TArray<FBody>& Bodies
)
{
	for (FPendulumRod* PendulumRod : RodSystemArray)
	{
		if (!PendulumRod)
		{
			continue;
		}

		PendulumRod->UpdateRodJacobian(Bodies);

		PendulumRod->ComputeInstantErrorData();
	}
}

// -----------------------------------------------------------------------------
// Matrix A
// -----------------------------------------------------------------------------

void FRodSys::UpdateAMatrix(
	const TArray<FBody>& Bodies
)
{
	const int32 NumRods =
		RodSystemArray.Num();

	for (
		int32 Row = 0;
		Row < NumRods;
		++Row
		)
	{
		for (
			int32 Column = 0;
			Column < NumRods;
			++Column
			)
		{
			check(RodSystemArray[Row]);
			check(RodSystemArray[Column]);

			MatrixIndex2ArrayIndex(
				Row,
				Column
			) =
				ComputeA_ij(
					*RodSystemArray[Row],
					*RodSystemArray[Column],
					Bodies
				);
		}
	}
}

// A_ij = J_i M^-1 J_j^T

double FRodSys::ComputeA_ij(
	const FPendulumRod& Rod_i,
	const FPendulumRod& Rod_j,
	const TArray<FBody>& Bodies
) const
{
	int32 BodyPivot_i = INDEX_NONE;
	int32 BodyBob_i = INDEX_NONE;

	int32 BodyPivot_j = INDEX_NONE;
	int32 BodyBob_j = INDEX_NONE;

	Rod_i.GetConnectedBodies(
		BodyPivot_i,
		BodyBob_i
	);

	Rod_j.GetConnectedBodies(
		BodyPivot_j,
		BodyBob_j
	);

	const FRodJacobian& Jacobian_i =
		Rod_i.GetRodJacobian();

	const FRodJacobian& Jacobian_j =
		Rod_j.GetRodJacobian();

	const FRodExtreme Pivot_i
	{
		BodyPivot_i,
		Jacobian_i.JPivot
	};

	const FRodExtreme Bob_i
	{
		BodyBob_i,
		Jacobian_i.JBob
	};

	const FRodExtreme Pivot_j
	{
		BodyPivot_j,
		Jacobian_j.JPivot
	};

	const FRodExtreme Bob_j
	{
		BodyBob_j,
		Jacobian_j.JBob
	};

	const auto ComputeA_ijComponent =
		[&Bodies](
			const FRodExtreme& Rod_iExtreme,
			const FRodExtreme& Rod_jExtreme
			) -> double
		{
			if (
				Rod_iExtreme.BodyIndex
				!= Rod_jExtreme.BodyIndex
				)
			{
				return 0.0;
			}

			const int32 SharedBodyIndex =
				Rod_iExtreme.BodyIndex;

			if (!Bodies.IsValidIndex(SharedBodyIndex))
			{
				return 0.0;
			}

			const FBody& SharedBody =
				Bodies[SharedBodyIndex];

			if (SharedBody.Mass <= UE_SMALL_NUMBER)
			{
				return 0.0;
			}

			const double InverseMass =
				1.0 / SharedBody.Mass;

			double A_ijComponent = 0.0;

			if (!SharedBody.bXFixed)
			{
				A_ijComponent +=
					Rod_iExtreme.Jacobian.X
					* Rod_jExtreme.Jacobian.X
					* InverseMass;
			}

			if (!SharedBody.bYFixed)
			{
				A_ijComponent +=
					Rod_iExtreme.Jacobian.Y
					* Rod_jExtreme.Jacobian.Y
					* InverseMass;
			}

			return A_ijComponent;
		};

	/*
	 * A_ij = J_i M^-1 J_j^T
	 *
	 * Each rod contributes one Jacobian block for its pivot
	 * and another for its bob. Only blocks associated with
	 * the same body produce a non-zero component.
	 */
	return
		ComputeA_ijComponent(
			Pivot_i,
			Pivot_j
		)
		+
		ComputeA_ijComponent(
			Pivot_i,
			Bob_j
		)
		+
		ComputeA_ijComponent(
			Bob_i,
			Pivot_j
		)
		+
		ComputeA_ijComponent(
			Bob_i,
			Bob_j
		);
}

// -----------------------------------------------------------------------------
// Vector B
// -----------------------------------------------------------------------------

void FRodSys::UpdateBVector(
	const TArray<FBody>& Bodies
)
{
	const int32 NumRods =
		RodSystemArray.Num();

	for (
		int32 Row = 0;
		Row < NumRods;
		++Row
		)
	{
		check(RodSystemArray[Row]);

		BVector[Row] =
			ComputeB_i(
				*RodSystemArray[Row],
				Bodies
			);
	}
}

double FRodSys::ComputeB_i(
	const FPendulumRod& Rod_i,
	const TArray<FBody>& Bodies
) const
{
	int32 BodyPivot_i = INDEX_NONE;
	int32 BodyBob_i = INDEX_NONE;

	Rod_i.GetConnectedBodies(
		BodyPivot_i,
		BodyBob_i
	);

	const FRodJacobian& Jacobian_i =
		Rod_i.GetRodJacobian();

	const FRodExtreme Pivot_i
	{
		BodyPivot_i,
		Jacobian_i.JPivot
	};

	const FRodExtreme Bob_i
	{
		BodyBob_i,
		Jacobian_i.JBob
	};

	const auto ComputeB_iComponent =
		[&Bodies](
			const FRodExtreme& Rod_iExtreme
			) -> double
		{
			if (
				!Bodies.IsValidIndex(
					Rod_iExtreme.BodyIndex
				)
				)
			{
				return 0.0;
			}

			const FBody& Body =
				Bodies[Rod_iExtreme.BodyIndex];

			if (Body.Mass <= UE_SMALL_NUMBER)
			{
				return 0.0;
			}

			const double InverseMass =
				1.0 / Body.Mass;

			double B_iComponent = 0.0;

			if (!Body.bXFixed)
			{
				B_iComponent +=
					Rod_iExtreme.Jacobian.X
					* Body.NetForce.X
					* InverseMass;
			}

			if (!Body.bYFixed)
			{
				B_iComponent +=
					Rod_iExtreme.Jacobian.Y
					* Body.NetForce.Y
					* InverseMass;
			}

			return B_iComponent;
		};

	const double JMinvF_i =
		ComputeB_iComponent(
			Pivot_i
		)
		+
		ComputeB_iComponent(
			Bob_i
		);

	const double JDotV_i =
		Rod_i.GetJDotV();

	/*
	 * Acceleration-level constraint equation:
	 *
	 * J_i a + JDot_i v = 0
	 *
	 * Substituting:
	 *
	 * a = M^-1 (F_ext + J^T Lambda)
	 *
	 * gives:
	 *
	 * A Lambda = B
	 *
	 * where:
	 *
	 * B_i = -(J_i M^-1 F_ext + JDot_i v)
	 */
	return -(
		JMinvF_i
		+ JDotV_i
		);
}

void FRodSys::UpdateStatisticalErrorData()
{
	for (FPendulumRod* PendulumRod : RodSystemArray)
	{
		if (!PendulumRod)
		{
			continue;
		}

		PendulumRod->UpdateStatisticalErrorData();
	}
}

// -----------------------------------------------------------------------------
// Matrix Management
// -----------------------------------------------------------------------------

double& FRodSys::MatrixIndex2ArrayIndex(
	int32 Row,
	int32 Column
)
{
	const int32 NumRods =
		RodSystemArray.Num();

	check(Row >= 0 && Row < NumRods);
	check(Column >= 0 && Column < NumRods);

	return AMatrix[
		Row * NumRods + Column
	];
}

const double& FRodSys::MatrixIndex2ArrayIndex(
	int32 Row,
	int32 Column
) const
{
	const int32 NumRods =
		RodSystemArray.Num();

	check(Row >= 0 && Row < NumRods);
	check(Column >= 0 && Column < NumRods);

	return AMatrix[
		Row * NumRods + Column
	];
}

