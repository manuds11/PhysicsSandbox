#pragma once

#include "CoreMinimal.h"
#include "CoreTypes.h"

namespace DenseLinearSolver
{
	/**
	 * Solves:
	 *
	 *     MatrixA * X = VectorB
	 *
	 * using Gaussian elimination with partial pivoting.
	 *
	 * MatrixA must represent a square matrix stored
	 * in dense row-major order.
	 *
	 * Returns false if:
	 * - MatrixA and VectorB dimensions are inconsistent.
	 * - The system is singular or numerically degenerate.
	 *
	 * The input matrix and vector are not modified.
	 */
	bool Solve(
		const TArray<double>& MatrixA,
		const TArray<double>& VectorB,
		TArray<double>& OutSolution
	);
}