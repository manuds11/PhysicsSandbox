
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
    double TotalDissipated = 0.0;

    double Potential() const
    {
        return Gravitational + Elastic;
    }

    double Mechanical() const
    {
        return Kinetic + Gravitational + Elastic;
    }

    double Total() const
    {
        return Mechanical() + TotalDissipated;
    }
};

// -----------------------------------------------------------------------------
// Correction diagnostics
// -----------------------------------------------------------------------------

struct FCorrectionDiagnostics
{
    FSystemEnergyState BeforeStep;
    FSystemEnergyState AfterStep;

    double PositionCorrectionRMS = 0.0;
    double PositionCorrectionMax = 0.0;

    double VelocityCorrectionRMS = 0.0;
    double VelocityCorrectionMax = 0.0;

    double DeltaKinetic() const
    {
        return AfterStep.Kinetic - BeforeStep.Kinetic;
    }

    double DeltaGravitational() const
    {
        return AfterStep.Gravitational - BeforeStep.Gravitational;
    }

    double DeltaElastic() const
    {
        return AfterStep.Elastic - BeforeStep.Elastic;
    }

    double DeltaMechanical() const
    {
        return AfterStep.Mechanical() - BeforeStep.Mechanical();
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

// -----------------------------------------------------------------------------
// Global energy diagnostics
// -----------------------------------------------------------------------------

struct FSystemEnergyBalance
{
    FSystemEnergyState Initial;
    FSystemEnergyState Current;

    double MechanicalVariationSinceStart() const
    {
        return Current.Mechanical() - Initial.Mechanical();
    }

    double EnergyResidualSinceStart() const
    {
        return Current.Total() - Initial.Total();
    }
};