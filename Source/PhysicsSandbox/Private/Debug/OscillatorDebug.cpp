// OscillatorDebug.cpp

#include "Debug/OscillatorDebug.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"

void FOscillatorDebug::Draw(
    UWorld* World,
    const FVector& MassWorldPosition,
    const FOscillatorState& State,
    const FOscillatorParams& Params,
    const FOscillatorDebugSettings& Settings
)
{
    if (!World)
    {
        return;
    }

    const FVector RestWorldPosition(
        Params.RestPosition,
        MassWorldPosition.Y,
        MassWorldPosition.Z
    );

    // x0 marker
    DrawDebugLine(
        World,
        RestWorldPosition + FVector(0.0, 0.0, -50.0),
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
    if (!FMath::IsNearlyZero(State.Velocity))
    {
        const FVector VelocityEnd =
            MassWorldPosition + FVector(State.Velocity * Settings.VelocityArrowScale, 0.0, 0.0);

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
    if (!FMath::IsNearlyZero(State.Acceleration))
    {
        const FVector AccelerationEnd =
            MassWorldPosition + FVector(State.Acceleration * Settings.AccelerationArrowScale, 0.0, 0.0);

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
    const FOscillatorState& State,
    const FOscillatorParams& Params,
    const FOscillatorForces& Forces
)
{
    if (!GEngine)
    {
        return;
    }

    GEngine->AddOnScreenDebugMessage(
        10,
        0.0f,
        FColor::Cyan,
        FString::Printf(
            TEXT("Sim time: %.2f s | avg dt: %.4f s\n"
                "\n"
                "Spring-Mass params:"
                "m: %.2f | k: %.2f | c: %.2f | L_0: %.2f\n"
                "\n"
                "State:"
                "x: %.2f | v: %.2f | a: %.2f"
                "\n"
                "Forces:"
                "F_Net: %.2f\n"
                "F_Spring : %.2f | F_Damping F: %.2f"
            ),
            RunningTime,
            AverageDeltaTime,
            Params.Mass,
            Params.Stiffness,
            Params.Damping,
            Params.RestPosition,
            State.Position,
            State.Velocity,
            State.Acceleration,
            Forces.NetForce,
            Forces.SpringForce,
            Forces.DampingForce       
        )
    );
}