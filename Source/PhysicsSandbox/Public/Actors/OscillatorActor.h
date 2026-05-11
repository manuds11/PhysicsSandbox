// OscillatorActor.h

// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Simulation/OscillatorSimulation.h"
#include "Debug/OscillatorDebug.h"
#include "OscillatorActor.generated.h"

UCLASS()
class AOscillatorActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AOscillatorActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	// =========================
    // Tick Simulation variables
    // =========================
    FOscillatorSimulation Simulation;

    bool bIsSimulationRunning = false;
        
    double RunningTime = 0.0;
    double AverageDeltaTime = 0.0;
    int32 FrameCount = 0;

    // =========================
    // Oscillator editable parameters
    // =========================
    UPROPERTY(EditAnywhere, Category = "Oscillator|Parameters")
    double Mass = 1.0; 
    UPROPERTY(EditAnywhere, Category = "Oscillator|Parameters")
    double Stiffness = 10.0;    // [N / m]
    UPROPERTY(EditAnywhere, Category = "Oscillator|Parameters")
    double Damping = 0.5;       // [N⋅s / m]

    // =========================
    // State variables
    // =========================
    UPROPERTY(EditAnywhere, Category = "Oscillator|Parameters")
    double RestPosition = 0.0;      // m

    UPROPERTY(EditAnywhere, Category = "Oscillator|Initial State")
    double InitialPosition = 1.0;   // m

    UPROPERTY(EditAnywhere, Category = "Oscillator|Initial State")
    double InitialVelocity = 0.0;   // m/s

    // =========================
    // Debug
    // =========================
    UPROPERTY(EditAnywhere, Category = "Oscillator|Debug")
    FOscillatorDebugSettings DebugSettings;
    
    // =========================
    // Internal methods
    // =========================
    void ToggleSimulation();
    void UpdateTimingStats(double DeltaTime);
    void UpdateVisualization();
};
