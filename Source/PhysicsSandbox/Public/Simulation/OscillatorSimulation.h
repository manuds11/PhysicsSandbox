// OscillatorSimulation.h

#pragma once

#include "Simulation/OscillatorTypes.h"

class FOscillatorSimulation
{
public:
    void SetParams(const FOscillatorParams& InParams);
    void SetState(const FOscillatorState& InState);

    const FOscillatorParams& GetParams() const;
    const FOscillatorState& GetState() const;
    const FOscillatorForces& GetForces() const;
    void UpdateDerivedStateAndComputeForces();
    void Step(double Dt);

private:
    double ComputeStateAcceleration(const double NetForce) const;
    FOscillatorForces ComputeStateForces(const double Position, const double Velocity) const;

private:
    FOscillatorParams Params;
    FOscillatorState State;
    FOscillatorForces Forces;
};