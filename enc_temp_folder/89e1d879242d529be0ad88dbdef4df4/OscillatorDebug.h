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
        const FOscillatorState& StateInUEUnits,
        const double RestPositionInUEUnits,
        const FOscillatorDebugSettings& Settings
    );

    static void PrintInfo(
        double RunningTime,
        double AverageDeltaTime,
        double SimulationTimeDebt,
        int32 LastSubStepCount,
        const FOscillatorState& StateInSIUnits,
        const FOscillatorParams& ParamsInSIUnits,
        const FOscillatorForces& ForcesInSiUnits,
        bool bIsSimulationRunning
    );
};