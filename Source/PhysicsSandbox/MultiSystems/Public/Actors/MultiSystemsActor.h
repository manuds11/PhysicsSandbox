// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Simulation/FullSys.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MultiSystemsActor.generated.h"

UCLASS()
class PHYSICSSANDBOX_API AMultiSystemsActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMultiSystemsActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	FFullSys FullSys;

	// =========================
	// Simulation control
	// =========================
	UPROPERTY(EditAnywhere, Category = "Simulation")
	bool bIsSimulationRunning = false;
	
	// =========================
	// Tick Simulation variables
	// =========================
	UPROPERTY(EditAnywhere, Category = "Simulation|Timing")
	double FixedTimeStep = 1.0 / 120.0;

	UPROPERTY(EditAnywhere, Category = "Simulation|Timing")
	int32 MaxSubSteps = 8;

	UPROPERTY(EditAnywhere, Category = "Simulation|Timing")
	double MaxFrameDeltaTime = 0.1;
	
	double SimulationTimeDebt = 0.0;
	double SimulationDelay = 0.0;

	double RealRunningTime = 0.0;
	double SimulatedRunningTime = 0.0;
	double AverageDeltaTime = 0.0;

	int64 FrameCount = 0;
	int32 LastSubStepCount = 0;

	// =========================
	// Visualization
	// =========================
	UPROPERTY(EditAnywhere, Category = "Visualization")
	FVector VisualizationOffset = FVector(-100.0, 0.0, 200.0); // in cm

	void ToggleSimulation();
	void RunSimFixedSteps(double FrameDeltaTime);
	FVector ToWorld(const FVector2D& P) const;
	void BuildDemoSystem();
	void DrawSystem() const;
	void PrintInfo() const;
};
