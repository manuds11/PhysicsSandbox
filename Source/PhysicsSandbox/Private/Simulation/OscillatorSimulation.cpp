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

void FOscillatorSimulation::Step(double Dt)
{
    const double A = ComputeAcceleration(State.Position, State.Velocity);

    State.Acceleration = A;
    State.Velocity += A * Dt;
    State.Position += State.Velocity * Dt;
    
}

double FOscillatorSimulation::ComputeAcceleration(double Position, double Velocity) const
{
    if (Params.Mass <= 0.0)
    {
        return 0.0;
    }

    const FOscillatorForces Forces = ComputeForces(Position, Velocity);
    return Forces.NetForce / Params.Mass;
}

FOscillatorForces FOscillatorSimulation::ComputeForces(double Position, double Velocity) const
{
    FOscillatorForces Forces;

    Forces.SpringForce = -Params.Stiffness * (Position - Params.RestPosition);
    Forces.DampingForce = -Params.Damping * Velocity;
    Forces.NetForce = Forces.SpringForce + Forces.DampingForce;

    return Forces;
}