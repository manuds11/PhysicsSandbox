// OscillatorIntegrator.cpp

#include "Simulation/OscillatorIntegrator.h"

FOscillatorState FExplicitEulerIntegrator::Integrate(
    const FOscillatorState& PreviousState,
    double Dt
) const
{
    FOscillatorState NewState = PreviousState;
    
    // Explicit Euler integration
    // x_{ n + 1 } = x_n + v_n dt
    // v_{ n + 1 } = v_n + a_n dt
    NewState.Position =
        PreviousState.Position + PreviousState.Velocity * Dt;

    NewState.Velocity =
        PreviousState.Velocity + PreviousState.Acceleration * Dt;

    return NewState;
}

FOscillatorState FSemiImplicitEulerIntegrator::Integrate(
    const FOscillatorState& PreviousState,
    double Dt
) const
{
    FOscillatorState NewState = PreviousState;

    // Semi-implicit Euler integration
    // v_{ n + 1 } = v_n + a_n dt
    // x_{ n + 1 } = x_n + v_{ n + 1 } dt
    NewState.Velocity =
        PreviousState.Velocity + PreviousState.Acceleration * Dt;

    NewState.Position =
        PreviousState.Position + NewState.Velocity * Dt;

    return NewState;
}