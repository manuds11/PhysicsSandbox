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
    Super::BeginPlay();

    EnableInput(GetWorld()->GetFirstPlayerController());

    if (InputComponent)
    {
        InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &ADebugProbeActor::ReleaseBall);
    }
}

void ADebugProbeActor::ReleaseBall()
{
    bReleased = true;
}

void ADebugProbeActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    FrameCount++;
    RunningTime += DeltaTime;
    AverageDeltaTime = RunningTime / FrameCount;

    FVector CurrentLocation = GetActorLocation();

    // FÍSICA Tras activación
    if (bReleased && !bAtGround)
    { 
        v_Z += a_Z * DeltaTime;
        CurrentLocation.Z += v_Z * DeltaTime;

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

    if (GEngine)
    {
        FString StateText = !bReleased ? TEXT("WAITING") : (bAtGround ? TEXT("RESTING") : TEXT("RELEASED"));
        FColor StateColor = (!bReleased || bAtGround) ? FColor::Red : FColor::Green;
        GEngine->AddOnScreenDebugMessage( 1, 0.0f, StateColor, StateText );

        GEngine->AddOnScreenDebugMessage( 2,
            0.0f, 
            FColor::Green,
            FString::Printf( 
                TEXT("Z: %.2f | vZ: %.2f | aZ: %.2f\nRunningTime: %.2f\nAv. DeltaTime: %.4f"),
                CurrentLocation.Z, v_Z, a_Z, 
                RunningTime,
                AverageDeltaTime
            )
        );
    }
}