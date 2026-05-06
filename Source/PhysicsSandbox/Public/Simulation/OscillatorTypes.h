#pragma once

struct FOscillatorParams
{
    double Mass = 1.0;
    double Stiffness = 10.0;
    double Damping = 0.0;
    double RestPosition = 0.0;
};

struct FOscillatorState
{
    double Position = 100.0;
    double Velocity = 0.0;
    double Acceleration = 0.0;
};

struct FOscillatorForces
{
    double SpringForce = 0.0;
    double DampingForce = 0.0;
    double NetForce = 0.0;
};