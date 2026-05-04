// Fill out your copyright notice in the Description page of Project Settings.


#include "Actors/OscillatorActor.h"

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

    FOscillatorState State;
    State.Position = InitialPosition;
    State.Velocity = InitialVelocity;
    State.Acceleration = 0.0;

    Simulation.SetParams(Params);
    Simulation.SetState(State);
}

// Called every frame
void AOscillatorActor::Tick(float DeltaTime)
{	
    Super::Tick(DeltaTime);

    Simulation.Step(DeltaTime);

    const double X = Simulation.GetState().Position;

    SetActorLocation(FVector(X, 0.0, 500.0));
}

