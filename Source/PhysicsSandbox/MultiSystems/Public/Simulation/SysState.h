#pragma once

#include "CoreMinimal.h"

#include <Eigen/Dense>

struct FStateDOFIndexMap
{
	int32 SizeQ_Full = 0;
	int32 SizeQ_FR = 0;

	TArray<int32> Q_FRToFullIndices;
	TArray<int32> QFull_FixedIndices;

	TArray<int32> X_FRToFullIndices;
	TArray<int32> X_FullFixedIndices;

	int32 FindQ_FRIndex(int32 Q_FullIndex) const
	{
		for (int32 Q_FRIndex = 0; Q_FRIndex < Q_FRToFullIndices.Num(); ++Q_FRIndex)
		{
			if (Q_FRToFullIndices[Q_FRIndex] == Q_FullIndex)
			{
				return Q_FRIndex;
			}
		}

		return INDEX_NONE;
	}

	Eigen::VectorXd ExtractX_FR(
		const Eigen::VectorXd& X_Full
	) const
	{
		check(X_Full.size() == 2 * SizeQ_Full);

		Eigen::VectorXd X_FR(X_FRToFullIndices.Num());

		for (int32 XIndex_FR = 0; XIndex_FR < X_FRToFullIndices.Num(); ++XIndex_FR)
		{
			X_FR(XIndex_FR) = X_Full(X_FRToFullIndices[XIndex_FR]);
		}

		return X_FR;
	}

	Eigen::VectorXd ExpandX_FRToFull(
		const Eigen::VectorXd& X_FR
	) const
	{
		check(X_FR.size() == X_FRToFullIndices.Num());

		Eigen::VectorXd X_Full = Eigen::VectorXd::Zero(2 * SizeQ_Full);

		for (int32 XIndex_FR = 0; XIndex_FR < X_FRToFullIndices.Num(); ++XIndex_FR)
		{
			X_Full(X_FRToFullIndices[XIndex_FR]) = X_FR(XIndex_FR);
		}

		return X_Full;
	}
};

struct FSysState
{
	Eigen::VectorXd Q;
	Eigen::VectorXd QDot;

	FSysState() = default;

	FSysState(
		Eigen::VectorXd InQ,
		Eigen::VectorXd InQDot
	)
		: Q(MoveTemp(InQ))
		, QDot(MoveTemp(InQDot))
	{
		check(Q.size() == QDot.size());
	}

	Eigen::VectorXd ToVector() const
	{
		check(Q.size() == QDot.size());

		Eigen::VectorXd XState(Q.size() + QDot.size());

		XState << Q, QDot;

		return XState;
	}

	static FSysState ToStruct(
		const Eigen::VectorXd& XState
	)
	{
		check(XState.size() % 2 == 0);

		const Eigen::Index NumCoordinates = XState.size() / 2;

		return FSysState(
			XState.head(NumCoordinates),
			XState.tail(NumCoordinates)
		);
	}
};

struct FSysStateDerivative
{
	Eigen::VectorXd QDot;
	Eigen::VectorXd QDotDot;

	FSysStateDerivative() = default;

	FSysStateDerivative(
		const Eigen::VectorXd& InQDot,
		Eigen::VectorXd InQDotDot
	)
		: QDot(InQDot)
		, QDotDot(MoveTemp(InQDotDot))
	{
		check(QDot.size() == QDotDot.size());
	}

	Eigen::VectorXd ToVector() const
	{
		check(QDot.size() == QDotDot.size());

		Eigen::VectorXd XDotState(QDot.size() + QDotDot.size());

		XDotState << QDot, QDotDot;

		return XDotState;
	}

	static FSysStateDerivative ToStruct(
		const Eigen::VectorXd& XDotState
	)
	{
		check(XDotState.size() % 2 == 0);

		const Eigen::Index NumCoordinates = XDotState.size() / 2;

		return FSysStateDerivative(
			XDotState.head(NumCoordinates),
			XDotState.tail(NumCoordinates)
		);
	}
};
