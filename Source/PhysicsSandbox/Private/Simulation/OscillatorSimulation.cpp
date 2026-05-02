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
    const double Acceleration = ComputeAcceleration(State.Position, State.Velocity);

    State.Velocity += Acceleration * Dt;
    State.Position += State.Velocity * Dt;
    State.Acceleration = Acceleration;
}

double FOscillatorSimulation::ComputeAcceleration(double Position, double Velocity) const
{
    const double SpringForce = -Params.Stiffness * (Position - Params.RestPosition);
    const double DampingForce = -Params.Damping * Velocity;
    const double TotalForce = SpringForce + DampingForce;

    return TotalForce / Params.Mass;
}