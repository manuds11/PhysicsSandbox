// Debug.h
#pragma once

#include "CoreMinimal.h"
#include "Simulation/OscillatorTypes.h"
#include "Debug.generated.h"

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
        const FOscillatorState& State,
        const FOscillatorParams& Params,
        double VelocityArrowScale,
        double AccelerationArrowScale
    );

    static void PrintInfo(
        double RunningTime,
        double AverageDeltaTime,
        const FOscillatorState& State,
        const FOscillatorParams& Params
    );
};