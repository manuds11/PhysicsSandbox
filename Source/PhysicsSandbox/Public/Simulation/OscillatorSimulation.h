#pragma once

#include "Simulation/OscillatorTypes.h"

class FOscillatorSimulation
{
public:
    void SetParams(const FOscillatorParams& InParams);
    void SetState(const FOscillatorState& InState);

    const FOscillatorParams& GetParams() const;
    const FOscillatorState& GetState() const;

    void Step(double Dt);

private:
    double ComputeAcceleration(double Position, double Velocity) const;

private:
    FOscillatorParams Params;
    FOscillatorState State;
};