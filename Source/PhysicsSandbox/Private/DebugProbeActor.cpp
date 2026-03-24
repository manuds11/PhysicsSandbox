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
    Super::BeginPlay();
    InitialLocation = GetActorLocation();
    EnableInput(GetWorld()->GetFirstPlayerController());

}

void ADebugProbeActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    FrameCount++;
    RunningTime += DeltaTime;
    AverageDeltaTime = RunningTime / FrameCount;

    FVector CurrentLocation = GetActorLocation();

    // FÍSICA tras activación


    // MENSAJES por pantalla
}