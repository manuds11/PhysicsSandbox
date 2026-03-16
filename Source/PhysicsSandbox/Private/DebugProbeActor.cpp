// Fill out your copyright notice in the Description page of Project Settings.


#include "DebugProbeActor.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

ADebugProbeActor::ADebugProbeActor()
{
	PrimaryActorTick.bCanEverTick = true;

	// Crear el componente de malla
	MeshComp = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComp"));

	// Hacer que sea el root del actor
	SetRootComponent(MeshComp);

	// Opcional: desactivar simulación física
	MeshComp->SetSimulatePhysics(false);
}

void ADebugProbeActor::BeginPlay()
{
	Super::BeginPlay();

	StartLocation = GetActorLocation();

	UE_LOG(LogTemp, Warning, TEXT("DebugProbeActor BeginPlay. StartLocation = %s"), *StartLocation.ToString());
}

void ADebugProbeActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	const float Time = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	const float OffsetZ = FMath::Sin(Time * Speed) * Amplitude;

	FVector NewLocation = StartLocation;
	NewLocation.Z += OffsetZ;

	SetActorLocation(NewLocation);
}