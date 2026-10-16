#pragma once

#include "CoreMinimal.h"

#include <Eigen/Dense>


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
