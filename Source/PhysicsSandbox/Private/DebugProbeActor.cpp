// Fill out your copyright notice in the Description page of Project Settings.

#include "DebugProbeActor.h"

#include "Engine/World.h"
#include "GameFramework/Actor.h"

ADebugProbeActor::ADebugProbeActor()
{
    PrimaryActorTick.bCanEverTick = true;
    FrameCount = 0;
}

void ADebugProbeActor::BeginPlay()
{
    UE_LOG(LogTemp, Warning, TEXT("BeginPlay"));
    Super::BeginPlay();

   // FVector StartLocation = GetActorLocation();
   // StartLocation.Z = Z_o;
   // SetActorLocation(StartLocation);

    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            -1,
            5.f,
            FColor::Green,
            TEXT("BeginPlay ejecutado")
        );
    }
}

void ADebugProbeActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    
    RunningTime += DeltaTime;

    FVector CurrentLocation = GetActorLocation();
    v_Z += a_Z * DeltaTime;
    CurrentLocation.Z += v_Z * DeltaTime;
    SetActorLocation(CurrentLocation);

    if (CurrentLocation.Z <= GroundZ)
    {
        CurrentLocation.Z = 0.0f;
        if (v_Z < 0.0f) {
            v_Z = -v_Z * 0.8f; // rebote con pérdida
        }
    }

    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            1,
            0.0f,
            FColor::Green,
            FString::Printf(
                TEXT("Z: %.2f | vZ: %.2f | aZ: %.2f\nRunningTime: %.2f"),
                CurrentLocation.Z, v_Z, a_Z, RunningTime
            )
        );
    }
}