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

	FVector StartLocation;

public:

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DebugProbe")
	float Speed = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DebugProbe")
	float Amplitude = 50.0f;
};