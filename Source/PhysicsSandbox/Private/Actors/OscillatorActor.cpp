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

    EnableInput(GetWorld()->GetFirstPlayerController());

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
        Simulation.Step(DeltaTime);

        UpdateTimingStats(DeltaTime);
    }

    UpdateVisualization();
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
        FOscillatorDebug::Draw(
            GetWorld(),
            GetActorLocation(),
            Simulation.GetState(),
            Simulation.GetParams(),
            DebugSettings
        );
    }

    if (DebugSettings.bPrintInfo)
    {
        FOscillatorDebug::PrintInfo(
            RunningTime,
            AverageDeltaTime,
            Simulation.GetState(),
            Simulation.GetParams(),
            Simulation.GetForces(),
            bIsSimulationRunning
        );
    }
}

void AOscillatorActor::UpdateTimingStats(double DeltaTime) // Al ser variables de Unreal van en el actor. OscillatorSimulation.h para física.
{
    RunningTime += DeltaTime;
    FrameCount++;

    AverageDeltaTime += (DeltaTime - AverageDeltaTime) / static_cast<double>(FrameCount);
}

