#pragma once

#include "CoreMinimal.h"
#include "Simulation/FullSysTypes.h"

struct FPolarBase
{
    FVector2D e_Radial = FVector2D::ZeroVector;
    FVector2D e_Theta = FVector2D::ZeroVector;
};

struct FRodJacobian
{
    FVector2D JPivot = FVector2D::ZeroVector;
    FVector2D JBob = FVector2D::ZeroVector;

    double JDotVel = 0.0;
};

struct FPendulumPositions
{
    FVector2D Bob2Pivot = FVector2D::ZeroVector;

    double InitialLength = 0.0;
    double ComputedLength = 0.0;

    double LengthAbsError = 0.0;
    double LengthRelError = 0.0;

    double PreviousLength = 0.0;
    double StepLengthIncrement = 0.0;
    double StepMeanLengthIncrement = 0.0;

    bool bLengthInitialized = false;
    bool bHasPreviousLengthSample = false;
};

struct FPendulumVelocities
{
    FVector2D Bob2Pivot = FVector2D::ZeroVector;

    double Bob2PivotRadial = 0.0;
    double Bob2PivotTangential = 0.0;

    double MeanRadialVelocity = 0.0;
};

class FPendulumRod : public ISysElement
{
public:
    FPendulumRod(
        int32 InPivotBody,
        int32 InBobBody
    );

    virtual ESysElementType GetElementType() const override;

    virtual bool GetConnectedBodies(
        int32& OutBodyA,
        int32& OutBodyB
    ) const override;

    double GetJDotV() const;

    const FPendulumPositions& GetPendulumPositions() const
    {
        return PendulumPos;
    }

    const FPendulumVelocities& GetPendulumVelocities() const
    {
        return PendulumVel;
    }

    const FRodJacobian& GetRodJacobian() const
    {
        return RodJacobian;
    }

    double GetConstraintFunctionValue() const
    {
        return PendulumPos.LengthAbsError;
    }

    void UpdateRodJacobian(
        const TArray<FBody>& Bodies
    );

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

    void UpdateErrorData();

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
    
    // Simulation
    uint64 ErrorDataSampleCount = 0;
    
    // Connectivity
    int32 PivotBody = INDEX_NONE;
    int32 BobBody = INDEX_NONE;

    // Runtime state
    FPolarBase PolarBase;
    FRodJacobian RodJacobian;
    FPendulumPositions PendulumPos;
    FPendulumVelocities PendulumVel;

    double LastComputedTension = 0.0; // N
};