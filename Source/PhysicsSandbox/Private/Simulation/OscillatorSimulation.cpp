// OscillatorSimulation.cpp

#include "Simulation/OscillatorSimulation.h"
#include "Math/UnrealMathUtility.h"

void FOscillatorSimulation::SetParams(const FOscillatorParams& InParams)
{
    Params = InParams;
}

void FOscillatorSimulation::SetInitialConditions(const FOscillatorState& InState)
{
    State.Position = InState.Position;
    State.Velocity = InState.Velocity;
    State.Displacement =
        State.Position - Params.RestPosition;

    Energy = FOscillatorEnergy();

    UpdateStateMechanicalEnergy();

    Energy.InitialSystemEnergy =
        Energy.MechanicalEnergy;

    UpdateStateDerivedMagnitudes();

    Metrics = ComputeMetrics();
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
    Forces = ComputeStateForces(State.Displacement, State.Velocity);
    State.Acceleration = ComputeStateAcceleration(Forces.NetForce);

    UpdateStateMechanicalEnergy();
    UpdateEnergyBalance();

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
    State.Displacement =
        State.Position - Params.RestPosition;

    Energy.DissipatedEnergy += ComputeStepDissipatedEnergy(State.Velocity, Dt);

    UpdateStateDerivedMagnitudes();
}

FOscillatorForces FOscillatorSimulation::ComputeStateForces(
    const double Displacement, 
    const double Velocity
) const
{
    FOscillatorForces ComputedForces;

    ComputedForces.SpringForce = -Params.Stiffness * (Displacement);
    ComputedForces.DampingForce = -Params.Damping * Velocity;
    ComputedForces.NetForce = ComputedForces.SpringForce + ComputedForces.DampingForce;

    return ComputedForces;
}

void FOscillatorSimulation::UpdateStateMechanicalEnergy()
{
    Energy.KineticEnergy =
        0.5 * Params.Mass * State.Velocity * State.Velocity;

    Energy.PotentialEnergy =
        0.5 * Params.Stiffness * State.Displacement * State.Displacement;

    Energy.MechanicalEnergy =
        Energy.KineticEnergy + Energy.PotentialEnergy;
}

double FOscillatorSimulation::ComputeStateAcceleration(const double NetForce) const
{
    if (Params.Mass <= 0.0)
    {
        return 0.0;
    }

    return NetForce / Params.Mass;
}

double FOscillatorSimulation::ComputeStepDissipatedEnergy(double Velocity, double Dt) const
{
    if (Params.Damping <= 0.0 || Dt <= 0.0)
    {
        return 0.0;
    }

    const double DissipatedPower =
        Params.Damping * Velocity * Velocity;

     const double StepDissipatedEnergy =
        DissipatedPower * Dt;

    return StepDissipatedEnergy;
}

void FOscillatorSimulation::UpdateEnergyBalance()
{
    Energy.TotalEnergyWithLosses =
        Energy.MechanicalEnergy + Energy.DissipatedEnergy;

    Energy.SimEnergyError =
        Energy.TotalEnergyWithLosses - Energy.InitialSystemEnergy;

    if (Energy.InitialSystemEnergy > 0.0)
    {
        Energy.RelativeSimEnergyError =
            Energy.SimEnergyError / Energy.InitialSystemEnergy;
    }
    else
    {
        Energy.RelativeSimEnergyError = 0.0;
    }
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