#pragma once

#include "CoreMinimal.h"

#include "ThirdParty/EigenIncludes.h"

class FFullSys;
class FControlSys;

class FMultiSysDisplay
{
public:

    void Initialize(
        UWorld* InWorld,
        const FFullSys* InFullSys,
        const FControlSys* InControlSys,
        const FVector& InVisualizationOffset,
        bool bInDrawControlForces,
        double InControlForceArrowScale,
        double InControlForceArrowMaxLength
    );

    void UpdateActorLocation(
        const FVector& InActorLocation
    );

    void DrawSystem() const;

private:
    
    // Display state
    // -----------------------------------------------------------------------------
    UWorld* World = nullptr;

    const FFullSys* FullSys = nullptr;
    const FControlSys* ControlSys = nullptr;

    FVector ActorLocation = FVector::ZeroVector;

    // Config
    // -----------------------------------------------------------------------------

    FVector VisualizationOffset = FVector::ZeroVector;
    bool bDrawControlForces = true;

    double ControlForceArrowScale = 5.0;       // cm / N
    double ControlForceArrowMaxLength = 250.0; // cm

    FVector ToWorld(
        const FVector2D& P
    ) const;

    double ComputeBodyDebugRadius(
        double Mass
    ) const;

    void DrawStatePerturbation(
        const Eigen::VectorXd& DeltaX_Full,
        double PositionScale,
        double VelocityScale
    ) const;

    void DrawStatePerturbationAtBody(
        int32 BodyIndex,
        const Eigen::VectorXd& DeltaX_Full,
        double PositionScale,
        double VelocityScale
    ) const;

    void DrawKernelZVector(
        const Eigen::VectorXd& Z_ker
    ) const;

    void DrawKernelZVectorAtBody(
        int32 BodyIndex,
        const Eigen::VectorXd& Z_ker
    ) const;

    void DrawControlForces() const;
};