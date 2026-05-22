// OscillatorSimulation.h

#pragma once

#include "Simulation/OscillatorTypes.h"
#include "Simulation/OscillatorIntegrator.h"

class FOscillatorSimulation
{
public:
    FOscillatorSimulation();

    void SetParams(const FOscillatorParams& InParams);
    void SetInitialConditions(const FOscillatorState& InState);
    void SetIntegrator(TUniquePtr<FOscillatorIntegrator> InIntegrator);

    const FOscillatorParams& GetParams() const;
    const FOscillatorState& GetState() const;
    const FOscillatorForces& GetForces() const;
    const FOscillatorEnergy& GetEnergy() const;
    const FOscillatorMetrics& GetMetrics() const;
    
    void Step(double Dt);

private:
    FOscillatorParams Params;
    FOscillatorState State;
    FOscillatorForces Forces;
    FOscillatorEnergy Energy;
    FOscillatorMetrics Metrics;

    TUniquePtr<FOscillatorIntegrator> Integrator;

private:
    void UpdateStateDerivedMagnitudes();
    double ComputeStateAcceleration(const double NetForce) const;
    FOscillatorForces ComputeStateForces(const double Displacement, const double Velocity) const;
    void UpdateStateMechanicalEnergy();
    double ComputeStepDissipatedEnergy(double Velocity, double Dt) const;
    void UpdateEnergyBalance();
    FOscillatorMetrics ComputeMetrics() const;
};