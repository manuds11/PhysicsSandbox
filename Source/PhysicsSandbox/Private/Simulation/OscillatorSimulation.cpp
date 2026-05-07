// OscillatorSimulation.cpp

#include "Simulation/OscillatorSimulation.h"

void FOscillatorSimulation::SetParams(const FOscillatorParams& InParams)
{
    Params = InParams;
}

void FOscillatorSimulation::SetState(const FOscillatorState& InState)
{
    State = InState;
}

const FOscillatorParams& FOscillatorSimulation::GetParams() const
{
    return Params;
}

const FOscillatorState& FOscillatorSimulation::GetState() const
{
    return State;
}

const FOscillatorForces& FOscillatorSimulation::GetForces() const
{
    return Forces;
}

void FOscillatorSimulation::Step(double Dt)
{
    FOscillatorState ThisTickState;
    
    const FOscillatorForces PrevTickForces = ComputeForces(State.Position, State.Velocity);
    // Compute Tick_State
    ThisTickState.Acceleration = ComputeAcceleration(PrevTickForces.NetForce);
    ThisTickState.Velocity = State.Velocity + ThisTickState.Acceleration * Dt;
    ThisTickState.Position = State.Position + ThisTickState.Velocity * Dt;

    State = ThisTickState;
    Forces = PrevTickForces;

}

double FOscillatorSimulation::ComputeAcceleration(const double NetForce) const
{
    if (Params.Mass <= 0.0)
    {
        return 0.0;
    }
    
    return NetForce / Params.Mass;
}

FOscillatorForces FOscillatorSimulation::ComputeForces(const double Position, const double Velocity) const
{
    FOscillatorForces ComputedForces;

    ComputedForces.SpringForce = -Params.Stiffness * (Position - Params.RestPosition);
    ComputedForces.DampingForce = -Params.Damping * Velocity;
    ComputedForces.NetForce = ComputedForces.SpringForce + ComputedForces.DampingForce;

    return ComputedForces;
}