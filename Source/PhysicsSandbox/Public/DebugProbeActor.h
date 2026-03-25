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
	float SimTime = 0.0f; // Tiempo de simulación
	float AverageDeltaTime = 0.0f;
	float t = 0.0f; // Tiempo físico

	// Reference frame
	FVector Pos_0 = FVector::ZeroVector;
	FVector Pos_prevTick = FVector::ZeroVector;
	FVector V_Tick = FVector::ZeroVector;
	
	// Physical variables
	UPROPERTY(EditAnywhere, Category = "Physics")
	float g = -980.0f; // g = 980 cm/s^2 Para gravedad terrestre en unreal

	// Trajectory parameters
	float Radius = 200.0f;
	float w = 1.0f;     // Angular freq (rad/s)
	float V_z = 100.0f;  // cm/s

	FVector ComputeHelixPosition(float CurrentTime) const;
	void PrintDebugInfo() const;
};