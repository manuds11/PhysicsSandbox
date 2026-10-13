
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

struct FDeltaEnergy
{
    FSystemEnergyState Before;
    FSystemEnergyState After;

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

struct FConstraintCorrectionEnergyDiagnostics
{
    // Current step projections
    // -------------------------------------------------------------------------

    FDeltaEnergy InitialVelocityProjection;
    FDeltaEnergy PositionProjection;
    FDeltaEnergy FinalVelocityProjection;

    // Accumulated energy increments
    // -------------------------------------------------------------------------

    FSystemEnergyState AccumulatedInitialVelocity;
    FSystemEnergyState AccumulatedPosition;
    FSystemEnergyState AccumulatedFinalVelocity;

    // Current step energy increments
    // -------------------------------------------------------------------------

    double StepKineticEnergyIncrement() const
    {
        return InitialVelocityProjection.DeltaKinetic()
            + PositionProjection.DeltaKinetic()
            + FinalVelocityProjection.DeltaKinetic();
    }

    double StepGravitationalEnergyIncrement() const
    {
        return InitialVelocityProjection.DeltaGravitational()
            + PositionProjection.DeltaGravitational()
            + FinalVelocityProjection.DeltaGravitational();
    }

    double StepElasticEnergyIncrement() const
    {
        return InitialVelocityProjection.DeltaElastic()
            + PositionProjection.DeltaElastic()
            + FinalVelocityProjection.DeltaElastic();
    }

    double StepProjectionEnergyIncrement() const
    {
        return InitialVelocityProjection.DeltaMechanical()
            + PositionProjection.DeltaMechanical()
            + FinalVelocityProjection.DeltaMechanical();
    }

    // Accumulated energy increments
    // -------------------------------------------------------------------------

    double AccumulatedProjectionEnergy() const
    {
        return AccumulatedInitialVelocity.Mechanical()
            + AccumulatedPosition.Mechanical()
            + AccumulatedFinalVelocity.Mechanical();
    }

    // Accumulation
    // -------------------------------------------------------------------------

    void AccumulateStepEnergy()
    {
        AccumulatedInitialVelocity.Kinetic += InitialVelocityProjection.DeltaKinetic();
        AccumulatedInitialVelocity.Gravitational += InitialVelocityProjection.DeltaGravitational();
        AccumulatedInitialVelocity.Elastic += InitialVelocityProjection.DeltaElastic();

        AccumulatedPosition.Kinetic += PositionProjection.DeltaKinetic();
        AccumulatedPosition.Gravitational += PositionProjection.DeltaGravitational();
        AccumulatedPosition.Elastic += PositionProjection.DeltaElastic();

        AccumulatedFinalVelocity.Kinetic += FinalVelocityProjection.DeltaKinetic();
        AccumulatedFinalVelocity.Gravitational += FinalVelocityProjection.DeltaGravitational();
        AccumulatedFinalVelocity.Elastic += FinalVelocityProjection.DeltaElastic();
    }
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