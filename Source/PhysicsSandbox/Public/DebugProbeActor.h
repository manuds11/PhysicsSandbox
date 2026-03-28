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
	int FrameCount = 0;
	float RunningTime = 0.0f; //
	float AverageDeltaTime = 0.0f;
	float Time = 0.0f; // Tiempo físico desde que el actor comienza a moverse

	// Action variables
	bool bReleased = false;
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bEnableDebugDraw = true;
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawTrajectory = true;
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawVelocityVector = true;
	UPROPERTY(EditAnywhere, Category = "Debug")
	float VelocityArrowScale = 1.0f; // Scale factor of speed module. 1 means its real size.

	// Reference frame
	FVector Pos_0 = FVector::ZeroVector;
	FVector Pos_Tick = FVector::ZeroVector;
	FVector Pos_prevTick = FVector::ZeroVector;
	FVector Vel_Tick = FVector::ZeroVector;
	
	// Physical variables
	UPROPERTY(EditAnywhere, Category = "Physics")
	float g = -980.0f; // g = 980 cm/s^2 Para gravedad terrestre en unreal
	// Trajectory parameters
	UPROPERTY(EditAnywhere, Category = "Physics")
	float Radius = 500.0f;
	UPROPERTY(EditAnywhere, Category = "Physics")
	float Omega = PI;     // Angular freq (rad/s)
	UPROPERTY(EditAnywhere, Category = "Physics")
	float Vel_Z = 500.0f;  // cm/s

	// Methods
	void ReleaseActor();
	FVector ComputeHelixPosition(float CurrentTime) const;
	FVector ComputeVelocityVector(float dt) const;
	void DrawDebugTrajectory() const;
	void DrawDebugVelocityVector() const;
	void PrintDebugInfo() const;	
};