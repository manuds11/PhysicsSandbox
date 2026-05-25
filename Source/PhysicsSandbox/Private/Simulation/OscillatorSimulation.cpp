// OscillatorSimulation.cpp

#include "Simulation/OscillatorSimulation.h"
#include "Math/UnrealMathUtility.h"

FOscillatorSimulation::FOscillatorSimulation()
{
    Integrator = MakeUnique<FSemiImplicitEulerIntegrator>();
}

void FOscillatorSimulation::SetParams(const FOscillatorParams& InParams)
{
    Params = InParams;
}

void FOscillatorSimulation::SetInitialConditions(const FOscillatorCoreState& InCoreState)
{
    CoreState = InCoreState;
    

	Energy = FOscillatorEnergy();  // Set to 0 as default, including DissipatedEnergy

    UpdateStateDerivedMagnitudes();

    UpdateStateMechanicalEnergy();  

    Energy.InitialSystemEnergy =
        Energy.MechanicalEnergy;

    UpdateEnergyDiagnostic();

    DynamicProperties = ComputeDynamicProperties();
}

void FOscillatorSimulation::SetIntegrator(
    TUniquePtr<FOscillatorIntegrator> InIntegrator
)
{
    Integrator = MoveTemp(InIntegrator);
}

const FOscillatorParams& FOscillatorSimulation::GetParams() const
{
    return Params;
}

const FOscillatorCoreState& FOscillatorSimulation::GetCoreState() const
{
    return CoreState;
}

const FOscillatorDerivedState& FOscillatorSimulation::GetDerivedState() const
{
    return DerivedState;
}

const FOscillatorForces& FOscillatorSimulation::GetForces() const
{
    return Forces;
}

const FOscillatorEnergy& FOscillatorSimulation::GetEnergy() const
{
    return Energy;
}

void FOscillatorSimulation::Step(double Dt)
{
    check(Integrator);

    CoreState = Integrator->Integrate(CoreState, DerivedState.Acceleration, Dt);

    UpdateStateDerivedMagnitudes();

    UpdateStateMechanicalEnergy();

    Energy.DissipatedEnergy +=
        ComputeStepDissipatedEnergy(CoreState.Velocity, Dt);
    
    UpdateEnergyDiagnostic();
}

void FOscillatorSimulation::UpdateStateDerivedMagnitudes()
{
    DerivedState.Displacement =
        CoreState.Position - Params.RestPosition;
    Forces = ComputeStateForces(DerivedState.Displacement, CoreState.Velocity);
    DerivedState.Acceleration = ComputeStateAcceleration(Forces.NetForce);
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

double FOscillatorSimulation::ComputeStateAcceleration(const double NetForce) const
{
    if (Params.Mass <= 0.0)
    {
        return 0.0;
    }

    return NetForce / Params.Mass;
}

void FOscillatorSimulation::UpdateStateMechanicalEnergy()
{
    Energy.KineticEnergy =
        0.5 * Params.Mass * CoreState.Velocity * CoreState.Velocity;

    Energy.PotentialEnergy =
        0.5 * Params.Stiffness * DerivedState.Displacement * DerivedState.Displacement;

    Energy.MechanicalEnergy =
        Energy.KineticEnergy + Energy.PotentialEnergy;
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

void FOscillatorSimulation::UpdateEnergyDiagnostic()
{
    Energy.TotalEnergyIncludingLosses =
        Energy.MechanicalEnergy + Energy.DissipatedEnergy;

    Energy.SimEnergyError =
        Energy.TotalEnergyIncludingLosses - Energy.InitialSystemEnergy;

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

FOscillatorDynamicProperties FOscillatorSimulation::ComputeDynamicProperties() const
{
    FOscillatorDynamicProperties ComputedMetrics;

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