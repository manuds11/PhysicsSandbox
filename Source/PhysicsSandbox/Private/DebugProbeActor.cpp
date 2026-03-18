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
    
    FrameCount++;
    UE_LOG(LogTemp, Warning, TEXT("Frame: %d"), FrameCount);
}