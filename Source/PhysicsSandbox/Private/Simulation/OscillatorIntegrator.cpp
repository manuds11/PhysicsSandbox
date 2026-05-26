// OscillatorIntegrator.cpp

#include "Simulation/OscillatorIntegrator.h"

FOscillatorCoreState FExplicitEulerIntegrator::Integrate(
    const FOscillatorCoreState& PreviousState,
    double PreviousAcceleration,
    double Dt
) const
{
    FOscillatorCoreState NewState = PreviousState;
    
    // Explicit Euler integration
    // x_{ n + 1 } = x_n + v_n dt
    // v_{ n + 1 } = v_n + a_n dt
    NewState.Position =
        PreviousState.Position + PreviousState.Velocity * Dt;

    NewState.Velocity =
        PreviousState.Velocity + PreviousAcceleration * Dt;

    return NewState;
}

FOscillatorCoreState FSemiImplicitEulerIntegrator::Integrate(
    const FOscillatorCoreState& PreviousState,
    double PreviousAcceleration,
    double Dt
) const
{
    FOscillatorCoreState NewState = PreviousState;

    // Semi-implicit Euler integration
    // v_{ n + 1 } = v_n + a_n dt
    // x_{ n + 1 } = x_n + v_{ n + 1 } dt
    NewState.Velocity =
        PreviousState.Velocity + PreviousAcceleration * Dt;

    NewState.Position =
        PreviousState.Position + NewState.Velocity * Dt;

    return NewState;
}