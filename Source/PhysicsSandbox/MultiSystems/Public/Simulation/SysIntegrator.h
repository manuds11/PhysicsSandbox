#pragma once

class ISysIntegrator
{
public:
	virtual ~ISysIntegrator() = default;

	virtual void IntegrateScalar(
		double& Position,
		double& Velocity,
		double Acceleration,
		double Dt
	) const = 0;
};

class FExplicitEulerSysIntegrator : public ISysIntegrator
{
public:
	virtual void IntegrateScalar(
		double& Position,
		double& Velocity,
		double Acceleration,
		double Dt
	) const override;
};

class FSemiImplicitEulerSysIntegrator : public ISysIntegrator
{
public:
	virtual void IntegrateScalar(
		double& Position,
		double& Velocity,
		double Acceleration,
		double Dt
	) const override;
};