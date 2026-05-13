// OscillatorSimulation.cpp

#include "Simulation/OscillatorSimulation.h"
#include "Math/UnrealMathUtility.h"

void FOscillatorSimulation::SetParams(const FOscillatorParams& InParams)
{
    Params = InParams;
}

void FOscillatorSimulation::SetState(const FOscillatorState& InState)
{
    State.Position = InState.Position;
    State.Velocity = InState.Velocity;

    Metrics = ComputeMetrics();
    UpdateStateDerivedMagnitudes();
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

const FOscillatorEnergy& FOscillatorSimulation::GetEnergy() const
{
    return Energy;
}

void FOscillatorSimulation::UpdateStateDerivedMagnitudes()
{ 
    Forces = ComputeStateForces(State.Position, State.Velocity);
    Energy = ComputeStateEnergy(State.Position, State.Velocity);
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

    UpdateStateDerivedMagnitudes();
}

FOscillatorForces FOscillatorSimulation::ComputeStateForces(
    const double Position, 
    const double Velocity
) const
{
    FOscillatorForces ComputedForces;

    ComputedForces.SpringForce = -Params.Stiffness * (Position - Params.RestPosition);
    ComputedForces.DampingForce = -Params.Damping * Velocity;
    ComputedForces.NetForce = ComputedForces.SpringForce + ComputedForces.DampingForce;

    return ComputedForces;
}

FOscillatorEnergy FOscillatorSimulation::ComputeStateEnergy(
    const double Position,
    const double Velocity
) const
{
    FOscillatorEnergy ComputedEnergy;

    const double Displacement =
        Position - Params.RestPosition;

    ComputedEnergy.KineticEnergy =
        0.5 * Params.Mass * Velocity * Velocity;

    ComputedEnergy.PotentialEnergy =
        0.5 * Params.Stiffness * Displacement * Displacement;

    ComputedEnergy.TotalEnergy =
        ComputedEnergy.KineticEnergy +
        ComputedEnergy.PotentialEnergy;

    return ComputedEnergy;
}

double FOscillatorSimulation::ComputeStateAcceleration(const double NetForce) const
{
    if (Params.Mass <= 0.0)
    {
        return 0.0;
    }

    return NetForce / Params.Mass;
}

FOscillatorMetrics FOscillatorSimulation::ComputeMetrics() const
{
    FOscillatorMetrics ComputedMetrics;

    if (Params.Mass <= 0.0 || Params.Stiffness <= 0.0)
    {
        return ComputedMetrics;
    }

    ComputedMetrics.NaturalFrequency =
        FMath::Sqrt(Params.Stiffness / Params.Mass);

    ComputedMetrics.NaturalFrequencyHz =
        ComputedMetrics.NaturalFrequency / (2.0 * PI);

    ComputedMetrics.NaturalPeriod =
        1.0 / ComputedMetrics.NaturalFrequencyHz;

    ComputedMetrics.CriticalDamping =
        2.0 * FMath::Sqrt(Params.Stiffness * Params.Mass);

    if (ComputedMetrics.CriticalDamping > 0.0)
    {
        ComputedMetrics.DampingRatio =
            Params.Damping / ComputedMetrics.CriticalDamping;
    }

    if (ComputedMetrics.DampingRatio < 1.0)
    {
        ComputedMetrics.DampedFrequency =
            ComputedMetrics.NaturalFrequency *
            FMath::Sqrt(1.0 - ComputedMetrics.DampingRatio * ComputedMetrics.DampingRatio);

        ComputedMetrics.DampedFrequencyHz =
            ComputedMetrics.DampedFrequency / (2.0 * PI);

        if (ComputedMetrics.DampedFrequencyHz > 0.0)
        {
            ComputedMetrics.DampedPeriod =
                1.0 / ComputedMetrics.DampedFrequencyHz;
        }
    }

    return ComputedMetrics;
}