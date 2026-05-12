// OscillatorSimulation.cpp

#include "Simulation/OscillatorSimulation.h"

void FOscillatorSimulation::SetParams(const FOscillatorParams& InParams)
{
    Params = InParams;
}

void FOscillatorSimulation::SetState(const FOscillatorState& InState)
{
    State.Position = InState.Position;
    State.Velocity = InState.Velocity;

    UpdateDerivedStateAndForces();
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

void FOscillatorSimulation::UpdateDerivedStateAndForces()
{ 
    Forces = ComputeStateForces(State.Position, State.Velocity);
    State.Acceleration = ComputeStateAcceleration(Forces.NetForce);
}

void FOscillatorSimulation::Step(double Dt)
{
    const FOscillatorState PreviousState = State;

    // Semi-implicit Euler integration
    // v_{ n + 1 } = v_n + a_n dt
    // x_{ n + 1 } = x_n + v_{ n + 1 } dt
    State.Velocity =
        PreviousState.Velocity + PreviousState.Acceleration * Dt;
    State.Position =
        PreviousState.Position + State.Velocity * Dt;

    UpdateDerivedStateAndForces();
}

double FOscillatorSimulation::ComputeStateAcceleration(const double NetForce) const
{
    if (Params.Mass <= 0.0)
    {
        return 0.0;
    }
    
    return NetForce / Params.Mass;
}

FOscillatorForces FOscillatorSimulation::ComputeStateForces(const double Position, const double Velocity) const
{
    FOscillatorForces ComputedForces;

    ComputedForces.SpringForce = -Params.Stiffness * (Position - Params.RestPosition);
    ComputedForces.DampingForce = -Params.Damping * Velocity;
    ComputedForces.NetForce = ComputedForces.SpringForce + ComputedForces.DampingForce;

    return ComputedForces;
}