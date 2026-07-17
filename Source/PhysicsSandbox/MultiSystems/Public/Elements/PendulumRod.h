#pragma once

#include "CoreMinimal.h"
#include "Simulation/FullSysTypes.h"

struct FPolarBase
{
    FVector2D e_Radial = FVector2D::ZeroVector;
    FVector2D e_Theta = FVector2D::ZeroVector;
};

struct FPendulumPositions
{
    FVector2D Bob2Pivot = FVector2D::ZeroVector;

    bool bLengthInitialized = false;

    double InitialLength = 1.0;
    double ComputedLength = 0.0;

    double LengthAbsError = 0.0;
    double LengthRelError = 0.0;
};

struct FPendulumVelocities
{
    FVector2D Bob2Pivot = FVector2D::ZeroVector;

    double Bob2PivotTangential = 0.0;
    double Bob2PivotRadial = 0.0;
};

class FPendulumRod : public ISysElement
{
public:
    FPendulumRod(
        int32 InPivotBody,
        int32 InBobBody
    );

    virtual bool GetConnectedBodies(
        int32& OutBodyA,
        int32& OutBodyB
    ) const override;

    const FPendulumPositions& GetPendulumPositions() const
    {
        return PendulumPos;
    }

    const FPendulumVelocities& GetPendulumVelocities() const
    {
        return PendulumVel;
    }

    double GetLastComputedTension() const
    {
        return LastComputedTension;
    }

    virtual void ApplyForces(
        TArray<FBody>& Bodies
    ) override;

    virtual void ProjectVelocities(
        TArray<FBody>& Bodies
    ) override;

    virtual void ProjectPositions(
        TArray<FBody>& Bodies
    ) override;

private:
    // Initialization
    void InitializePendulumLength();

    // State update pipeline
    void UpdatePendulumKinematics(
        const FBody& Pivot,
        const FBody& Bob
    );

    void UpdateLengthErrors();

    // Derived quantities
    FPolarBase ComputePolarBase() const;

    FVector2D ComputeFreeAcceleration(
        const FBody& Body
    ) const;

    double ComputeEffectiveInverseMass(
        const FBody& Body,
        const FVector2D& e_Radial
    ) const;

    double ComputeTension(
        const FBody& Pivot,
        const FBody& Bob,
        const FPolarBase& InPolarBase
    ) const;

    void ApplyVelocityImpulse(
        FBody& Body,
        const FVector2D& Impulse
    ) const;

    void ApplyPositionCorrection(
        FBody& Body,
        const FVector2D& Correction
    ) const;

    // Connectivity
    int32 PivotBody = INDEX_NONE;
    int32 BobBody = INDEX_NONE;

    // Runtime state
    FPendulumPositions PendulumPos;
    FPendulumVelocities PendulumVel;
    FPolarBase PolarBase;

    double LastComputedTension = 0.0; // N
};