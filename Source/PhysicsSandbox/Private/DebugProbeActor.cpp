// Fill out your copyright notice in the Description page of Project Settings.

#include "DebugProbeActor.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

ADebugProbeActor::ADebugProbeActor()
{
    PrimaryActorTick.bCanEverTick = true;
    FrameCount = 0;
    UE_LOG(LogTemp, Warning, TEXT("Constructor de DebugProbeActor"));
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
    
    // UE_LOG(LogTemp, Warning, TEXT("Delta: %f"), DeltaTime);

    FrameCount++;
    RunningTime += DeltaTime;

    if (FrameCount % 15 == 0) // Cada 5 frames se imprime
    {
        UE_LOG(LogTemp, Warning, TEXT("Frame: %d | Time FC: %.2f"), FrameCount, RunningTime);
    }

    LogTimer += DeltaTime;

    if (LogTimer >= 1.0f)
    {
        UE_LOG(LogTemp, Warning, TEXT("Sim Time: %.2f"), RunningTime);
        LogTimer = 0.0f;
    }
}