
#pragma once

#include "CoreMinimal.h"

// -----------------------------------------------------------------------------
// System energy
// -----------------------------------------------------------------------------

struct FSystemEnergyState
{
    double Kinetic = 0.0;
    double Gravitational = 0.0;
    double Elastic = 0.0;

    double Potential() const
    {
        return Gravitational + Elastic;
    }

    double Mechanical() const
    {
        return Kinetic + Gravitational + Elastic;
    }
};

// -----------------------------------------------------------------------------
// Correction diagnostics
// -----------------------------------------------------------------------------

struct FCorrectionDiagnostics
{
    FSystemEnergyState Before;
    FSystemEnergyState After;

    double PositionCorrectionRMS = 0.0;
    double PositionCorrectionMax = 0.0;

    double VelocityCorrectionRMS = 0.0;
    double VelocityCorrectionMax = 0.0;

    double DeltaKinetic() const
    {
        return After.Kinetic - Before.Kinetic;
    }

    double DeltaGravitational() const
    {
        return After.Gravitational - Before.Gravitational;
    }

    double DeltaElastic() const
    {
        return After.Elastic - Before.Elastic;
    }

    double DeltaMechanical() const
    {
        return After.Mechanical() - Before.Mechanical();
    }
};

// -----------------------------------------------------------------------------
// Constraint correction diagnostics
// -----------------------------------------------------------------------------

struct FConstraintCorrectionDiagnostics
{
    FCorrectionDiagnostics InitialVelocityProjection;
    FCorrectionDiagnostics PositionProjection;
    FCorrectionDiagnostics FinalVelocityProjection;
};
