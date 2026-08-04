#include "Math/DenseLinearSolver.h"

namespace DenseLinearSolver
{
	bool Solve(
		const TArray<double>& MatrixA,
		const TArray<double>& VectorB,
		TArray<double>& OutSolution
	)
	{
		const int32 SystemSize =
			VectorB.Num();

		if (SystemSize == 0)
		{
			OutSolution.Reset();
			return true;
		}

		if (
			MatrixA.Num()
			!= SystemSize * SystemSize
			)
		{
			OutSolution.Reset();
			return false;
		}

		/*
		 * Gaussian elimination modifies both the matrix
		 * and the right-hand-side vector, so local copies
		 * are used.
		 */
		TArray<double> WorkingMatrix =
			MatrixA;

		TArray<double> WorkingVector =
			VectorB;

		OutSolution.Init(
			0.0,
			SystemSize
		);

		// -----------------------------------------------------
		// Forward elimination with partial pivoting
		// -----------------------------------------------------

		for (
			int32 PivotColumn = 0;
			PivotColumn < SystemSize;
			++PivotColumn
			)
		{
			int32 PivotRow =
				PivotColumn;

			double MaxPivotMagnitude =
				FMath::Abs(
					WorkingMatrix[
						PivotColumn
							* SystemSize
							+ PivotColumn
					]
				);

			/*
			 * Find the row with the largest absolute value
			 * in the current pivot column.
			 */
			for (
				int32 CandidateRow =
				PivotColumn + 1;
				CandidateRow < SystemSize;
				++CandidateRow
				)
			{
				const double CandidateMagnitude =
					FMath::Abs(
						WorkingMatrix[
							CandidateRow
								* SystemSize
								+ PivotColumn
						]
					);

				if (
					CandidateMagnitude
				> MaxPivotMagnitude
					)
				{
					MaxPivotMagnitude =
						CandidateMagnitude;

					PivotRow =
						CandidateRow;
				}
			}

			/*
			 * A zero or nearly-zero pivot means that the
			 * system is singular or numerically degenerate.
			 */
			if (
				MaxPivotMagnitude
				<= UE_SMALL_NUMBER
				)
			{
				OutSolution.Init(
					0.0,
					SystemSize
				);

				return false;
			}

			/*
			 * Move the selected pivot row into the current
			 * pivot position.
			 */
			if (PivotRow != PivotColumn)
			{
				for (
					int32 Column = 0;
					Column < SystemSize;
					++Column
					)
				{
					Swap(
						WorkingMatrix[
							PivotColumn
								* SystemSize
								+ Column
						],
						WorkingMatrix[
							PivotRow
								* SystemSize
								+ Column
						]
					);
				}

				Swap(
					WorkingVector[PivotColumn],
					WorkingVector[PivotRow]
				);
			}

			const double PivotValue =
				WorkingMatrix[
					PivotColumn
						* SystemSize
						+ PivotColumn
				];

			/*
			 * Eliminate every coefficient below the pivot.
			 */
			for (
				int32 Row = PivotColumn + 1;
				Row < SystemSize;
				++Row
				)
			{
				const double EliminationFactor =
					WorkingMatrix[
						Row
							* SystemSize
							+ PivotColumn
					]
					/ PivotValue;

				WorkingMatrix[
					Row
						* SystemSize
						+ PivotColumn
				] = 0.0;

				for (
					int32 Column =
					PivotColumn + 1;
					Column < SystemSize;
					++Column
					)
				{
					WorkingMatrix[
						Row
							* SystemSize
							+ Column
					] -=
						EliminationFactor
							* WorkingMatrix[
								PivotColumn
									* SystemSize
									+ Column
							];
				}

				WorkingVector[Row] -=
					EliminationFactor
					* WorkingVector[
						PivotColumn
					];
			}
		}

		// -----------------------------------------------------
		// Back substitution
		// -----------------------------------------------------

		for (
			int32 Row = SystemSize - 1;
			Row >= 0;
			--Row
			)
		{
			double KnownTerms = 0.0;

			for (
				int32 Column = Row + 1;
				Column < SystemSize;
				++Column
				)
			{
				KnownTerms +=
					WorkingMatrix[
						Row
							* SystemSize
							+ Column
					]
					* OutSolution[Column];
			}

			const double DiagonalValue =
				WorkingMatrix[
					Row
						* SystemSize
						+ Row
				];

			if (
				FMath::Abs(DiagonalValue)
				<= UE_SMALL_NUMBER
				)
			{
				OutSolution.Init(
					0.0,
					SystemSize
				);

				return false;
			}

			OutSolution[Row] =
				(
					WorkingVector[Row]
					- KnownTerms
					)
				/ DiagonalValue;
		}

		return true;
	}
}