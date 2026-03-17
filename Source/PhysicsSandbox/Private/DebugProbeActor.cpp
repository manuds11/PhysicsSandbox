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
}

void ADebugProbeActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}