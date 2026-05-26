// OscillatorIntegrator.h

#pragma once

#include "Simulation/OscillatorTypes.h"

class FOscillatorIntegrator
{
public:
    virtual ~FOscillatorIntegrator() = default;

    virtual FOscillatorCoreState Integrate(
        const FOscillatorCoreState& PreviousState,
		double PreviousAcceleration,
        double Dt
    ) const = 0;
};

class FExplicitEulerIntegrator : public FOscillatorIntegrator
{
public:
    virtual FOscillatorCoreState Integrate(
        const FOscillatorCoreState& PreviousState,
        double PreviousAcceleration,
        double Dt
    ) const override;
};

class FSemiImplicitEulerIntegrator : public FOscillatorIntegrator
{
public:
    virtual FOscillatorCoreState Integrate(
        const FOscillatorCoreState& PreviousState,
        double PreviousAcceleration,
        double Dt
    ) const override;
};