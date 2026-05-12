// OscillatorDebug.cpp

#include "Debug/OscillatorDebug.h"
#include "Math/Units.h"

#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/World.h"

void FOscillatorDebug::Draw(
    UWorld* World,
    const FVector& MassWorldPosition,
    const FOscillatorState& StateInUEUnits,
    const double RestPositionInUEUnits,
    const FOscillatorDebugSettings& Settings
)
{
    if (!World)
    {
        return;
    }

    const FVector RestWorldPosition(
        RestPositionInUEUnits,
        MassWorldPosition.Y,
        MassWorldPosition.Z
    );

    // x0 marker
    DrawDebugLine(
        World,
        RestWorldPosition + FVector(0.0, 0.0, -50.0), // All in centimeters
        RestWorldPosition + FVector(0.0, 0.0, 50.0),
        FColor::Green,
        false,
        0.0f,
        0,
        3.0f
    );

    // line: mass <-> x0
    DrawDebugLine(
        World,
        RestWorldPosition,
        MassWorldPosition,
        FColor::Cyan,
        false,
        0.0f,
        0,
        2.0f
    );

    // velocity vector
    if (!FMath::IsNearlyZero(StateInUEUnits.Velocity))
    {
        const FVector VelocityEnd =
            MassWorldPosition + FVector(StateInUEUnits.Velocity * Settings.VelocityArrowScale, 0.0, 0.0);

        DrawDebugDirectionalArrow(
            World,
            MassWorldPosition,
            VelocityEnd,
            100.0f,
            FColor::Blue,
            false,
            0.0f,
            0,
            1.0f
        );
    }

    // acceleration vector
    if (!FMath::IsNearlyZero(StateInUEUnits.Acceleration))
    {
        const FVector AccelerationEnd =
            MassWorldPosition + FVector(StateInUEUnits.Acceleration * Settings.AccelerationArrowScale, 0.0, 0.0);

        DrawDebugDirectionalArrow(
            World,
            MassWorldPosition,
            AccelerationEnd,
            100.0f,
            FColor::Red,
            false,
            0.0f,
            0,
            1.0f
        );
    }
}

void FOscillatorDebug::PrintInfo(
    double RunningTime,
    double AverageDeltaTime,
    const FOscillatorState& StateInSIUnits,
    const FOscillatorParams& ParamsInSIUnits,
    const FOscillatorForces& ForcesInSIUnits,
    bool bIsSimulationRunning
)
{
    if (!GEngine)
    {
        return;
    }

    const FString SimulationStatusText =
        bIsSimulationRunning
        ? TEXT("HARMONIC OSCILLATOR RUNNING")
        : TEXT("PRESS SPACE BAR TO START SIMULATION");

    const FColor StatusColor =
        bIsSimulationRunning
        ? FColor::Green
        : FColor::Red;

    GEngine->AddOnScreenDebugMessage(
        10,
        0.0f,
        StatusColor,
        FString::Printf(
            TEXT("%s\n"
                "\n"
                "Sim time: %.2f [s] | avg dt: %.4f [s]\n"
                "\n"
                "Spring-Mass params:\n"
                "m: %.2f [kg]\n"
                "k: %.2f [N/m]\n"
                "c: %.2f [N*s/m]\n"
                "L_0: %.2f [m]\n"
                "\n"
                "State:\n"
                "x: %.2f [m]\n"
                "v: %.2f [m/s]\n"
                "a: %.2f [m/s^2]\n"
                "\n"
                "Forces:\n"
                "F_Net: %.2f [N]\n"
                "F_Spring: %.2f [N]\n"
                "F_Damping: %.2f [N]"),
            *SimulationStatusText,
            RunningTime,
            AverageDeltaTime,
            ParamsInSIUnits.Mass,
            ParamsInSIUnits.Stiffness,
            ParamsInSIUnits.Damping,
            ParamsInSIUnits.RestPosition,
            StateInSIUnits.Position,
            StateInSIUnits.Velocity,
            StateInSIUnits.Acceleration,
            ForcesInSIUnits.NetForce,
            ForcesInSIUnits.SpringForce,
            ForcesInSIUnits.DampingForce
        )
    );
}