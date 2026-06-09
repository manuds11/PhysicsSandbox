#include "Simulation/SysIntegrator.h"

void FExplicitEulerSysIntegrator::IntegrateScalar(
	double& Position,
	double& Velocity,
	double Acceleration,
	double Dt
) const
{
	Position += Velocity * Dt;
	Velocity += Acceleration * Dt;
}

void FSemiImplicitEulerSysIntegrator::IntegrateScalar(
	double& Position,
	double& Velocity,
	double Acceleration,
	double Dt
) const
{
	Velocity += Acceleration * Dt;
	Position += Velocity * Dt;
}