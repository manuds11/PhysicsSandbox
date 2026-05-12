// OscillatorTypes.h

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
    double Position = 0.0;
    double Velocity = 0.0;
    double Acceleration = 0.0;

    FOscillatorState operator*(double Scalar) const
    {
        FOscillatorState Result = *this;

        Result.Position *= Scalar;
        Result.Velocity *= Scalar;
        Result.Acceleration *= Scalar;

        return Result;     
    }

    friend FOscillatorState operator*(double Scalar, const FOscillatorState& State)
    {
        return State * Scalar;
    }
};

struct FOscillatorForces
{
    double SpringForce = 0.0;
    double DampingForce = 0.0;
    double NetForce = 0.0;
};