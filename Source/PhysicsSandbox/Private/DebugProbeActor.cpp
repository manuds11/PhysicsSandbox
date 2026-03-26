// Fill out your copyright notice in the Description page of Project Settings.

#include "DebugProbeActor.h"

#include "Engine/World.h"

namespace
{
    constexpr float CmToM = 0.01f;
}

ADebugProbeActor::ADebugProbeActor()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ADebugProbeActor::BeginPlay()
{
    Super::BeginPlay();
    Pos_0 = GetActorLocation();
    Pos_prevTick = Pos_0;

    // EnableInput(GetWorld()->GetFirstPlayerController());
}

FVector ADebugProbeActor::ComputeHelixPosition(float t) const
{
    const float X = Radius * FMath::Cos(Omega * t);
    const float Y = Radius * FMath::Sin(Omega * t);
    const float Z = Vel_Z * t;

    const FVector Offset(X, Y, Z);
    return Pos_0 + Offset;
}

void ADebugProbeActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    FrameCount++;
    RunningTime += DeltaTime;
    AverageDeltaTime = RunningTime / FrameCount;
    Time += DeltaTime;

    Pos_Tick = ComputeHelixPosition(Time);

    if (DeltaTime > KINDA_SMALL_NUMBER)
    {
        Vel_Tick = (Pos_Tick - Pos_prevTick) / DeltaTime;
    }
    else
    {
        Vel_Tick = FVector::ZeroVector;
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

    const FVector PosMeters = Pos_Tick * CmToM;
    const FVector VelMeters = Vel_Tick * CmToM;
    
    // AddOnScreenDebugMessage displays these blocks in reverse visual order,
    // so they are intentionally called from bottom block to top block.

    GEngine->AddOnScreenDebugMessage(
        11,
        0.0f,
        FColor::Cyan,
        FString::Printf(
            TEXT("Pos (m) [X Y Z]: %.2f | %.2f | %.2f\nVel (m/s) [X Y Z]: %.2f | %.2f | %.2f"
            "\nRadius (cm): %.2f | Angularfreq (rad/s): %.2f"),
            PosMeters.X, PosMeters.Y, PosMeters.Z,
            VelMeters.X, VelMeters.Y, VelMeters.Z,
            Radius, Omega
        )
    );

    GEngine->AddOnScreenDebugMessage(
        10,
        0.0f,
        FColor::Green,
        FString::Printf(TEXT("HELICAL TRANSLATION ACTIVE\nTime: %.2f | Avg. dt: %.4f"), Time, AverageDeltaTime)
    );
}