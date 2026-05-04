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

    State.Velocity += A * Dt;
    State.Position += State.Velocity * Dt;
    State.Acceleration = A;
}

double FOscillatorSimulation::ComputeAcceleration(double Position, double Velocity) const
{
    const double SpringForce = -Params.Stiffness * (Position - Params.RestPosition);    
    const double DampingForce = -Params.Damping * Velocity;
    const double TotalForce = SpringForce + DampingForce;
    const double A = TotalForce / Params.Mass;

    return A;
}