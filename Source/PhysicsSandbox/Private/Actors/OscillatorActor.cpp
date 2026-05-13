// OscillatorActor.cpp

// Fill out your copyright notice in the Description page of Project Settings.


#include "Actors/OscillatorActor.h"
#include "Math/Units.h"

// Sets default values
AOscillatorActor::AOscillatorActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AOscillatorActor::BeginPlay()
{
	Super::BeginPlay();
	
    FOscillatorParams Params;
    Params.Mass = Mass;
    Params.Stiffness = Stiffness;
    Params.Damping = Damping;
    Params.RestPosition = RestPosition;

    FOscillatorState State; // Condiciones iniciales.
    State.Position = InitialPosition;
    State.Velocity = InitialVelocity;

    Simulation.SetParams(Params);
    Simulation.SetState(State);

    UpdateVisualization();

    if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
    {
        EnableInput(PC);
    }

    if (InputComponent)
    {
        InputComponent->BindKey(
            EKeys::SpaceBar,
            IE_Pressed,
            this,
            &AOscillatorActor::ToggleSimulation
        );
    }
}

void AOscillatorActor::ToggleSimulation()
{
    bIsSimulationRunning = !bIsSimulationRunning;
}

// Called every frame
void AOscillatorActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (bIsSimulationRunning)
    {
        AdvanceSimulation(DeltaTime);
    }

    UpdateVisualization();
}

void AOscillatorActor::AdvanceSimulation(double FrameDeltaTime)
{
    if (FixedTimeStep <= 0.0)
    {
        return;
    }

    RealRunningTime += FrameDeltaTime;
    FrameCount++;

    AverageDeltaTime +=
        (FrameDeltaTime - AverageDeltaTime) / static_cast<double>(FrameCount);

    const double ClampedFrameDeltaTime =
        FMath::Min(FrameDeltaTime, MaxFrameDeltaTime);

    SimulationTimeDebt += ClampedFrameDeltaTime;

    int32 SubStepCount = 0;

    while (SimulationTimeDebt >= FixedTimeStep && SubStepCount < MaxSubSteps)
    {
        Simulation.Step(FixedTimeStep);

        SimulatedRunningTime += FixedTimeStep;
        SimulationTimeDebt -= FixedTimeStep;
        SubStepCount++;
    }

    LastSubStepCount = SubStepCount;

    if (SubStepCount >= MaxSubSteps)
    {
        SimulationTimeDebt = FMath::Min(SimulationTimeDebt, FixedTimeStep);
    }

    SimulationDelay = RealRunningTime - SimulatedRunningTime;
}

void AOscillatorActor::UpdateVisualization()
{
    SetActorLocation(
        FVector(
            Simulation.GetState().Position * Units::MToCm,
            0.0,
            100.0
        )
    );

    if (DebugSettings.bDrawDebug)
    {
        FOscillatorState StateInUnrealUnits = Simulation.GetState() * Units::MToCm;

        FOscillatorDebug::Draw(
            GetWorld(),
            GetActorLocation(),     // cm
            StateInUnrealUnits,
            Simulation.GetParams().RestPosition * Units::MToCm,
            DebugSettings
        );
    }

    if (DebugSettings.bPrintInfo)
    {
        FOscillatorDebug::PrintInfo(
            RealRunningTime,
            AverageDeltaTime,
            SimulationDelay,
            Simulation.GetState(),
            Simulation.GetParams(),
            Simulation.GetForces(),
            Simulation.GetEnergy(),
            bIsSimulationRunning
        );
    }
}



