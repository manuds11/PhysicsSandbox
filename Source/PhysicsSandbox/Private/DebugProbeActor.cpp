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
    Pos_0 = GetActorLocation();
    Pos_prevTick = Pos_0;

    // EnableInput(GetWorld()->GetFirstPlayerController());
}

FVector ADebugProbeActor::ComputeHelixPosition(float time) const
{
    const float X = Radius * FMath::Cos(w * time);
    const float Y = Radius * FMath::Sin(w * time);
    const float Z = V_z * time;

    const FVector Offset(X, Y, Z);
    return Pos_0 + Offset;
}

void ADebugProbeActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    FrameCount++;
    SimTime += DeltaTime;
    AverageDeltaTime = SimTime / FrameCount;
    t += DeltaTime;

    const FVector Pos_Tick = ComputeHelixPosition(t);

    if (DeltaTime > KINDA_SMALL_NUMBER)
    {
        V_Tick = (Pos_Tick - Pos_prevTick) / DeltaTime;
    }
    else
    {
        V_Tick = FVector::ZeroVector;
    }

    SetActorLocation(Pos_Tick);
    
    Pos_prevTick = Pos_Tick;

    PrintDebugInfo();
}

void ADebugProbeActor::PrintDebugInfo() const
{
    if (!GEngine)
    {
        return;
    }

    const FVector Pos = GetActorLocation();

    GEngine->AddOnScreenDebugMessage(
        10,
        0.0f,
        FColor::Green,
        FString::Printf( TEXT("HELICAL TRANSLATION ACTIVE\nAvg. dt: %.4f"), AverageDeltaTime )
    );

    GEngine->AddOnScreenDebugMessage(
        11,
        0.0f,
        FColor::Cyan,
        FString::Printf(
            TEXT("Pos [X Y Z]: %.2f | %.2f | %.2f\nVel [X Y Z]: %.2f | %.2f | %.2f"),
            Pos.X, Pos.Y, Pos.Z,
            V_Tick.X, V_Tick.Y, V_Tick.Z
        )
    );

    GEngine->AddOnScreenDebugMessage(
        12,
        0.0f,
        FColor::Yellow,
        FString::Printf(
            TEXT("t: %.2f | Avg dt: %.4f\nRadius: %.2f | Angularfreq: %.2f | V_z: %.2f"),
            t,
            AverageDeltaTime,
            Radius,
            w,
            V_z
        )
    );
}