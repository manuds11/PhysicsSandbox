// Fill out your copyright notice in the Description page of Project Settings.


#include "DebugProbeActor.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

ADebugProbeActor::ADebugProbeActor()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ADebugProbeActor::BeginPlay()
{
	Super::BeginPlay();

	StartLocation = GetActorLocation();
}

void ADebugProbeActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	float Time = GetWorld()->GetTimeSeconds();
	float OffsetZ = FMath::Sin(Time * Speed) * Amplitude;

	FVector NewLocation = StartLocation;
	NewLocation.Z += OffsetZ;

	SetActorLocation(NewLocation);
}