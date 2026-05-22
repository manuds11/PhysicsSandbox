// OscillatorIntegrator.h

#pragma once

#include "Simulation/OscillatorTypes.h"

class FOscillatorIntegrator
{
public:
    virtual ~FOscillatorIntegrator() = default;

    virtual void Integrate(
        FOscillatorState& State,
        const FOscillatorParams& Params,
        double Dt
    ) const = 0;
};