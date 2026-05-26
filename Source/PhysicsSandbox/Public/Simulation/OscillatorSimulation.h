// OscillatorSimulation.h

#pragma once

#include "Simulation/OscillatorTypes.h"
#include "Simulation/OscillatorIntegrator.h"

class FOscillatorSimulation
{
public:
    FOscillatorSimulation();

    void SetParams(const FOscillatorParams& InParams);
    void SetInitialConditions(const FOscillatorCoreState& InCoreState);
    void SetIntegrator(TUniquePtr<FOscillatorIntegrator> InIntegrator);

    const FOscillatorParams& GetParams() const;
    const FOscillatorCoreState& GetCoreState() const;
    const FOscillatorDerivedState& GetDerivedState() const;
    const FOscillatorForces& GetForces() const;
    const FOscillatorEnergy& GetEnergy() const;
    const FOscillatorDynamicProperties& GetDynamicProperties() const;
    
    void Step(double Dt);

private:
    FOscillatorParams Params;

    FOscillatorCoreState CoreState;
    FOscillatorDerivedState DerivedState;

    FOscillatorForces Forces;
    FOscillatorEnergy Energy;
    FOscillatorDynamicProperties DynamicProperties;

    TUniquePtr<FOscillatorIntegrator> Integrator;

private:
    void UpdateStateDerivedMagnitudes();
    double ComputeStateAcceleration(const double NetForce) const;
    FOscillatorForces ComputeStateForces(const double Displacement, const double Velocity) const;
    void UpdateStateMechanicalEnergy();
    double ComputeStepDissipatedEnergy(double Velocity, double Dt) const;
    void UpdateEnergyDiagnostics();
    FOscillatorDynamicProperties ComputeDynamicProperties() const;
};