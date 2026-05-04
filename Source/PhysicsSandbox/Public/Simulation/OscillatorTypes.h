#pragma once

struct FOscillatorParams
{
    double Mass;
    double Stiffness;
    double Damping;
    double RestPosition;

    FOscillatorParams(
        double InMass = 1.0,
        double InStiffness = 10.0,
        double InDamping = 0.0,
        double InRestPosition = 0.0)
        : Mass(InMass)
        , Stiffness(InStiffness)
        , Damping(InDamping)
        , RestPosition(InRestPosition)
    {
    }
};

struct FOscillatorState
{
    double Position = 100.0;
    double Velocity = 0.0;
    double Acceleration = 0.0;
};