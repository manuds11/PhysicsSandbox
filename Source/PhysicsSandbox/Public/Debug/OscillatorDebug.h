// OscillatorDebug.h

#pragma once

#include "CoreMinimal.h"
#include "Simulation/OscillatorTypes.h"
#include "OscillatorDebug.generated.h"

class UWorld;

USTRUCT(BlueprintType)
struct FOscillatorDebugSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, Category = "Oscillator|Debug")
    bool bDrawDebug = true;

    UPROPERTY(EditAnywhere, Category = "Oscillator|Debug")
    bool bPrintInfo = true;

    UPROPERTY(EditAnywhere, Category = "Oscillator|Debug")
    double VelocityArrowScale = 1.0;

    UPROPERTY(EditAnywhere, Category = "Oscillator|Debug")
    double AccelerationArrowScale = 0.1;
};

struct FOscillatorDebug
{
    static void Draw(
        UWorld* World,
        const FVector& MassWorldPosition,
        const FOscillatorCoreState& CoreStateInUEUnits,
        const FOscillatorDerivedState& DerivedStateInUEUnits,
        const double RestPositionInUEUnits,
        const FOscillatorDebugSettings& Settings
    );

    static void PrintInfo(
        double RealRunningTime,
        double AverageDeltaTime,
        double SimulationDelay,
        const double SimFixedTimeStep,
        const FOscillatorCoreState& CoreStateInSIUnits,
        const FOscillatorDerivedState& DerivedStateInSIUnits,
        const FOscillatorParams& ParamsInSIUnits,
        const FOscillatorForces& ForcesInSIUnits,
        const FOscillatorEnergy& EnergyInSIUnits,
        bool bIsSimulationRunning
    );
};