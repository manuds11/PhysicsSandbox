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

    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            -1,
            0.0f,
            FColor::Green,
            FString::Printf(
                TEXT("Z: %.2f | Vz: %.2f | Az: %.2f"),
                CurrentLocation.Z, v_Z, a_Z)
        );
    }
}