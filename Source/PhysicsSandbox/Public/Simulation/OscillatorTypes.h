// OscillatorTypes.h

#pragma once

template<typename DerivedType>
struct TSimulationOperators
{
    DerivedType operator*(double Scalar) const
    {
        DerivedType Result = static_cast<const DerivedType&>(*this);
        Result.Scale(Scalar);
        return Result;
    }

    friend DerivedType operator*(double Scalar, const TSimulationOperators& Value)
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

struct FOscillatorState : public TSimulationOperators<FOscillatorState>
{
    double Position = 0.0;
    double Displacement = 0.0;
    double Velocity = 0.0;
    double Acceleration = 0.0;

    void Scale(double Scalar)
    {
        Position *= Scalar;
        Displacement *= Scalar;
        Velocity *= Scalar;
        Acceleration *= Scalar;
    }
};

struct FOscillatorForces : public TSimulationOperators<FOscillatorForces>
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

struct FOscillatorEnergy
{
    double KineticEnergy = 0.0;
    double PotentialEnergy = 0.0;
    double DissipatedEnergy = 0.0;
    double MechanicalEnergy = 0.0;
    double TotalEnergyWithLosses = 0.0;
    
    double InitialSystemEnergy = 0.0;
    double SimEnergyError = 0.0;
    double RelativeSimEnergyError = 0.0;
};

struct FOscillatorMetrics
{
    double NaturalFrequency = 0.0;        // omega_n [rad/s]
    double NaturalFrequencyHz = 0.0;      // f_n [Hz]
    double NaturalPeriod = 0.0;           // T_n [s]

    double CriticalDamping = 0.0;         // c_crit [N*s/m]
    double DampingRatio = 0.0;            // zeta [-]

    double DampedFrequency = 0.0;         // omega_d [rad/s]
    double DampedFrequencyHz = 0.0;       // f_d [Hz]
    double DampedPeriod = 0.0;            // T_d [s]
};