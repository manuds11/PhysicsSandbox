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
    UE_LOG(LogTemp, Warning, TEXT("BeginPlay"));
    Super::BeginPlay();

   // FVector StartLocation = GetActorLocation();
   // StartLocation.Z = Z_o;
   // SetActorLocation(StartLocation);
}

void ADebugProbeActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    FrameCount++;
    RunningTime += DeltaTime;
    AverageDeltaTime = RunningTime / FrameCount;

    FVector CurrentLocation = GetActorLocation();

    if (!bReleased && RunningTime >= ReleaseTime)
    {
        bReleased = true;
    }

    if (bReleased && !bAtGround)
    { 
        v_Z += a_Z * DeltaTime;
        CurrentLocation.Z += v_Z * DeltaTime;
        SetActorLocation(CurrentLocation);

        if (CurrentLocation.Z <= GroundZ)
        {
            CurrentLocation.Z = GroundZ;
            if (v_Z < 0.0f) {
                v_Z = -v_Z * Restitution; // rebote con pérdida
                if (v_Z < StopSpeedThreshold)
                {
                    v_Z = 0.0f;
                    bAtGround = true;
                }
            
            }
        }
        SetActorLocation(CurrentLocation);
    }

    FString StateText = !bReleased ? TEXT("WAITING") : (bAtGround ? TEXT("RESTING") : TEXT("RELEASED"));
    FColor StateColor = (!bReleased || bAtGround) ? FColor::Red : FColor::Green;

    if (GEngine)
    {
        // Línea 1: estado, con color dinámico
        GEngine->AddOnScreenDebugMessage(
            1,
            0.0f,
            StateColor,
            StateText
        );

        // Línea 2: datos, siempre en verde o en el color que quieras
        GEngine->AddOnScreenDebugMessage(
            2,
            0.0f,
            FColor::Green,
            FString::Printf(
                TEXT("Z: %.2f | vZ: %.2f | aZ: %.2f\nRunningTime: %.2f\nAv. DeltaTime: %.4f"),
                CurrentLocation.Z,
                v_Z,
                a_Z,
                RunningTime,
                AverageDeltaTime
            )
        );
    }
}