#pragma once

#include "CoreMinimal.h"

#include <Eigen/Dense>

struct FSysState
{
	Eigen::VectorXd Q_Full;
	Eigen::VectorXd QDot_Full;

	FSysState() = default;

	FSysState(
		Eigen::VectorXd InQ_Full,
		Eigen::VectorXd InQDot_Full
	)
		: Q_Full(MoveTemp(InQ_Full))
		, QDot_Full(MoveTemp(InQDot_Full))
	{
		check(Q_Full.size() == QDot_Full.size());
	}

	Eigen::VectorXd ToVector() const
	{
		check(Q_Full.size() == QDot_Full.size());

		Eigen::VectorXd XState(
			Q_Full.size() + QDot_Full.size()
		);

		XState << Q_Full, QDot_Full;

		return XState;
	}
};


struct FSysStateDerivative
{
	Eigen::VectorXd QDot_Full;
	Eigen::VectorXd QDotDot_Full;

	FSysStateDerivative() = default;

	FSysStateDerivative(
		const Eigen::VectorXd& InQDot_Full,
		Eigen::VectorXd InQDotDot_Full
	)
		: QDot_Full(InQDot_Full)
		, QDotDot_Full(MoveTemp(InQDotDot_Full))
	{
		check(QDot_Full.size() == QDotDot_Full.size());
	}

	Eigen::VectorXd ToVector() const
	{
		check(QDot_Full.size() == QDotDot_Full.size());

		Eigen::VectorXd XDot(
			QDot_Full.size() + QDotDot_Full.size()
		);

		XDot << QDot_Full, QDotDot_Full;

		return XDot;
	}
};
 