// OscillatorIntegrator.h

#pragma once

#include "Simulation/OscillatorTypes.h"

class FOscillatorIntegrator
{
public:
    virtual ~FOscillatorIntegrator() = default;

    virtual FOscillatorState Integrate(
        const FOscillatorState& PreviousState,
        double Dt
    ) const = 0;
};

class FExplicitEulerIntegrator : public FOscillatorIntegrator
{
public:
    virtual FOscillatorState Integrate(
        const FOscillatorState& PreviousState,
        double Dt
    ) const override;
};

class FSemiImplicitEulerIntegrator : public FOscillatorIntegrator
{
public:
    virtual FOscillatorState Integrate(
        const FOscillatorState& PreviousState,
        double Dt
    ) const override;
};