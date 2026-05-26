// OscillatorDebug.cpp

#include "Debug/OscillatorDebug.h"

#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/World.h"

void FOscillatorDebug::Draw(
    UWorld* World,
    const FVector& MassWorldPosition,
    const FOscillatorCoreState& CoreStateInUEUnits,
    const FOscillatorDerivedState& DerivedStateInUEUnits,
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
    if (!FMath::IsNearlyZero(CoreStateInUEUnits.Velocity))
    {
        const FVector VelocityEnd =
            MassWorldPosition + FVector(CoreStateInUEUnits.Velocity * Settings.VelocityArrowScale, 0.0, 0.0);

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
    if (!FMath::IsNearlyZero(DerivedStateInUEUnits.Acceleration))
    {
        const FVector AccelerationEnd =
            MassWorldPosition + FVector(DerivedStateInUEUnits.Acceleration * Settings.AccelerationArrowScale, 0.0, 0.0);

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
    double RealRunningTime,
    double AverageDeltaTime,
    double SimulationDelay,
    double SimFixedTimeStep,
    const FOscillatorCoreState& CoreStateInSIUnits,
    const FOscillatorDerivedState& DerivedStateInSIUnits,
    const FOscillatorParams& ParamsInSIUnits,
    const FOscillatorForces& ForcesInSIUnits,
    const FOscillatorEnergy& EnergyInSIUnits,
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
                "Timing:\n"
                "Real sim time: %.2f [s]\n"
                "Simulation delay: %.4f [s]\n"
                "avg dt: %.4f [s]\n"
                "Simulation fixedStep dt: %.4f [s]\n"
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
                "F_Damping: %.2f [N]\n"
                "\n"
                "Energy:\n"
                "Mechanical: %.4f [J]\n"
                "Dissipated: %.4f [J]\n"
                "Total w/ losses: %.4f [J]\n"
                "Sim error: %.6f [J]\n"
                "Relative sim error: %.4f [%%]"
            ),
            *SimulationStatusText,

            RealRunningTime,
            SimulationDelay,
            AverageDeltaTime,
            SimFixedTimeStep,

            ParamsInSIUnits.Mass,
            ParamsInSIUnits.Stiffness,
            ParamsInSIUnits.Damping,
            ParamsInSIUnits.RestPosition,

            CoreStateInSIUnits.Position,
            CoreStateInSIUnits.Velocity,
            DerivedStateInSIUnits.Acceleration,

            ForcesInSIUnits.NetForce,
            ForcesInSIUnits.SpringForce,
            ForcesInSIUnits.DampingForce,

            EnergyInSIUnits.MechanicalEnergy,
            EnergyInSIUnits.DissipatedEnergy,
            EnergyInSIUnits.TotalEnergyIncludingLosses,
            EnergyInSIUnits.SimEnergyError,
            EnergyInSIUnits.RelativeSimEnergyError * 100.0
        )
    );
}