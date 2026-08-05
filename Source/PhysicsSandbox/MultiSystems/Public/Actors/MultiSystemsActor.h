// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Simulation/FullSys.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MultiSystemsActor.generated.h"

UENUM(BlueprintType)
enum class EIntegratorType : uint8
{
	ExplicitEuler,
	SemiImplicitEuler
};

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
	EIntegratorType IntegratorType =
		EIntegratorType::SemiImplicitEuler;

	UPROPERTY(EditAnywhere, Category = "Simulation")
	bool bIsSimulationRunning = false;
	
	// =========================
	// Tick Simulation variables
	// =========================
	UPROPERTY(EditAnywhere, Category = "Simulation|Timing")
	double FixedTimeStep = 1.0 / 960.0;

	UPROPERTY(EditAnywhere, Category = "Simulation|Timing")
	int32 MaxSubSteps = 8;

	UPROPERTY(EditAnywhere, Category = "Simulation|Timing")
	double MaxFrameDeltaTime = 0.1;
	
	double SimulationTimeDebt = 0.0;
	double SimulationDelay = 0.0;

	double SimuWallClockTime = 0.0;
	double SimuPhysicalTime = 0.0;
	double AverageDeltaTime = 0.0;

	int64 FrameCount = 0;
	int32 LastSubStepCount = 0;

	uint64 FixedStepCount = 0;

	// =========================
	// Visualization
	// =========================
	UPROPERTY(EditAnywhere, Category = "Visualization")
	FVector VisualizationOffset = FVector(-100.0, 0.0, 200.0); // in cm

	// =========================
	// AUX - Remove after debugging
	// =========================
	int32 PendulumIndexForDebug = INDEX_NONE;

	void BuildDemoSystem();

	void ToggleSimulation();
	void ConfigureIntegrator();

	void RunSimFixedSteps(double FrameDeltaTime);

	FVector ToWorld(const FVector2D& P) const;
	void DrawSystem() const;
	void PrintInfo() const;
};
