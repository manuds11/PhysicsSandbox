// OscillatorTypes.h

#pragma once

template<typename DerivedType>
struct TMathOperators
{
    DerivedType operator*(double Scalar) const
    {
        DerivedType Result = static_cast<const DerivedType&>(*this);
        Result.Scale(Scalar);
        return Result;
    }

    friend DerivedType operator*(double Scalar, const TMathOperators& Value)
    {
        return static_cast<const DerivedType&>(Value) * Scalar;
    }
};

struct FOscillatorParams
{
    double Mass = 1.0;
    double Stiffness = 10.0;
    double Damping = 0.0;
    double RestPosition = 0.0;
};

struct FOscillatorState : public TMathOperators<FOscillatorState>
{
    double Position = 0.0;
    double Velocity = 0.0;
    double Acceleration = 0.0;

    void Scale(double Scalar)
    {
        Position *= Scalar;
        Velocity *= Scalar;
        Acceleration *= Scalar;
    }
};

struct FOscillatorForces : public TMathOperators<FOscillatorForces>
{
    double SpringForce = 0.0;
    double DampingForce = 0.0;
    double NetForce = 0.0;

    void Scale(double Scalar)
    {
        SpringForce *= Scalar;
        DampingForce *= Scalar;
        NetForce *= Scalar;
    }
};