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
	
}

// Called every frame
void AOscillatorActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// prueba simple
	const float Time = GetWorld()->TimeSeconds;
	const float X = FMath::Sin(Time) * 100.0f;

	SetActorLocation(FVector(X, 0.0f, 0.0f));
}

