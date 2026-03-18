// Fill out your copyright notice in the Description page of Project Settings.


#include "DebugProbeActor.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

ADebugProbeActor::ADebugProbeActor()
{
    PrimaryActorTick.bCanEverTick = true;
    UE_LOG(LogTemp, Warning, TEXT("Constructor de DebugProbeActor"));
}

void ADebugProbeActor::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogTemp, Warning, TEXT("BeginPlay de DebugProbeActor"));
}

void ADebugProbeActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}