// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DebugProbeActor.generated.h"

UCLASS()
class PHYSICSSANDBOX_API ADebugProbeActor : public AActor
{
    GENERATED_BODY()

public:
    ADebugProbeActor();

protected:
    virtual void BeginPlay() override;

public:
    virtual void Tick(float DeltaTime) override;
private:
    // Actor::Tick variables
    int FrameCount;
    float RunningTime = 0.0f;
    float LogTimer = 0.0f;

    // Physical variables
    float VerticalSpeed = 80.0f; 
};