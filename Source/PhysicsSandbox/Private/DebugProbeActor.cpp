// Fill out your copyright notice in the Description page of Project Settings.

#include "DebugProbeActor.h"

#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Math/Quat.h"
#include "Math/RotationMatrix.h"

namespace
{
    constexpr float CmToM = 0.01f;
}

ADebugProbeActor::ADebugProbeActor()
{
    PrimaryActorTick.bCanEverTick = true;

    Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
    RootComponent = Mesh;

    Mesh->SetSimulatePhysics(false);
    Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void ADebugProbeActor::BeginPlay()
{
    Super::BeginPlay();
    Pos_0 = GetActorLocation();
    Pos_Tick = Pos_0;
    Pos_prevTick = Pos_0;

    EnableInput(GetWorld()->GetFirstPlayerController());
    if (InputComponent)
    {
        InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &ADebugProbeActor::ReleaseActor);
    }
}

void ADebugProbeActor::ReleaseActor()
{
    if (bReleased)
    {
        return;
    }
    bReleased = true;
}

FVector ADebugProbeActor::ComputeHelixPosition(float t) const
{
    const float X = Radius * FMath::Cos(Omega * t) - Radius;
    const float Y = Radius * FMath::Sin(Omega * t);
    const float Z = Vel_Z * t;

    const FVector Offset(X, Y, Z);
    return Pos_0 + Offset;
}

FVector ADebugProbeActor::ComputeVelocityVector(float dt) const 
{
    if (dt <= KINDA_SMALL_NUMBER)
    {
        return FVector::ZeroVector;
    }
    return (Pos_Tick - Pos_prevTick) / dt;
}

void ADebugProbeActor::UpdateActorRotation(float DeltaTime)
{
    if (Vel_Tick.IsNearlyZero())
    {
        return;
    }

    const FVector Forward = Vel_Tick.GetSafeNormal();
    const FVector WorldUp = FVector::UpVector;

    // Construye una orientación base:
    // X = Forward
    // Z = lo más cercano posible a WorldUp
    const FMatrix BaseRotationMatrix = FRotationMatrix::MakeFromXZ(Forward, WorldUp);
    const FQuat BaseQuat = BaseRotationMatrix.ToQuat();

    if (!bSpinAroundForward)
    {
        SetActorRotation(BaseQuat);
        return;
    }

    RollAngleDeg += RollRateDeg * DeltaTime;

    // Rotación extra alrededor del eje Forward
    const FQuat RollQuat( Forward, FMath::DegreesToRadians(RollAngleDeg) );
    

    // Composición final
    const FQuat FinalQuat = RollQuat * BaseQuat;

    SetActorRotation(FinalQuat);
}

void ADebugProbeActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    FrameCount++;
    RunningTime += DeltaTime;
    AverageDeltaTime = RunningTime / FrameCount;
    
    if (bReleased) 
    {
        Time += DeltaTime;

        Pos_Tick = ComputeHelixPosition(Time);
        Vel_Tick = ComputeVelocityVector(DeltaTime);
    UpdateActorRotation(DeltaTime);

        if (bEnableDebugDraw)
        {
            if (bDrawTrajectory)
            {
                DrawTrajectory();
            }
            if (bDrawVelocityVector)
            {
                DrawVelocityVector();
            }
            if (bDrawBodyFrame)
            {
                DrawBodyFrame();
            }
        }
        
        SetActorLocation(Pos_Tick);
    
        Pos_prevTick = Pos_Tick;
    }

    PrintDebugInfo();
}

void ADebugProbeActor::DrawTrajectory() const
{
    DrawDebugLine(
        GetWorld(),        // UWorld* → contexto del mundo donde dibujar
        Pos_prevTick,      // FVector → punto inicial de la línea (posición anterior)
        Pos_Tick,          // FVector → punto final de la línea (posición actual)
        FColor::Blue,      // FColor → color de la línea
        false,             // bool bPersistentLines → si la línea es permanente (false = temporal)
        10.0f,             // float LifeTime → tiempo en segundos que permanece visible
        0,                 // uint8 DepthPriority → prioridad de render (0 = normal)
        2.0f               // float Thickness → grosor de la línea
    );
}

void ADebugProbeActor::DrawVelocityVector() const {
    
    if (!Vel_Tick.IsNearlyZero())
    {
        const FVector ArrowEnd = Pos_Tick + Vel_Tick * VelocityArrowScale;

        DrawDebugDirectionalArrow(
            GetWorld(),        // UWorld* → contexto del mundo
            Pos_Tick,          // FVector → inicio de la flecha (posición actual)
            ArrowEnd,          // FVector → final de la flecha (dirección + magnitud escalada)
            10000.0f,             // float ArrowSize → tamaño de la punta de la flecha
            FColor::Black,       // FColor → color de la flecha
            false,             // bool bPersistentLines → si la flecha es persistente
            0.0f,              // float LifeTime → 0 = solo un frame
            0,                 // uint8 DepthPriority → prioridad de render
            5.0f               // float Thickness → grosor de la flecha
        );
    }
}

void ADebugProbeActor::DrawBodyFrame() const
{
    const FVector Origin = Pos_Tick;

    const FVector ForwardEnd = Origin + GetActorForwardVector() * BodyFrameAxisLength;
    const FVector RightEnd = Origin + GetActorRightVector() * BodyFrameAxisLength;
    const FVector UpEnd = Origin + GetActorUpVector() * BodyFrameAxisLength;

    DrawDebugLine(
        GetWorld(),
        Origin,
        ForwardEnd,
        FColor::Red,
        false,
        0.0f,
        0,
        3.0f
    );

    DrawDebugLine(
        GetWorld(),
        Origin,
        RightEnd,
        FColor::Green,
        false,
        0.0f,
        0,
        3.0f
    );

    DrawDebugLine(
        GetWorld(),
        Origin,
        UpEnd,
        FColor::Blue,
        false,
        0.0f,
        0,
        3.0f
    );
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
            TEXT("t (s): %.3f"
                "\nPos (m) [X Y Z]: %.2f | %.2f | %.2f\nVel (m/s) [X Y Z]: %.2f | %.2f | %.2f"
            "\nRadius (m): %.2f | Angularfreq (rad/s): %.2f"),
            Time, 
            PosMeters.X, PosMeters.Y, PosMeters.Z,
            VelMeters.X, VelMeters.Y, VelMeters.Z,
            Radius * CmToM, Omega
        )
    );

    FString PromptText = !bReleased ?
        FString::Printf(TEXT("CLICK ANYWHERE AND PRESS SPACE BAR TO RELEASE\nAvg. dt: %.4f"), AverageDeltaTime)
        : FString::Printf(TEXT("HELICAL TRANSLATION ACTIVE\nRunning Time: %.2f | Avg. dt: %.4f"), RunningTime, AverageDeltaTime);
    FColor PromptColor = !bReleased ? FColor::Red : FColor::Green;

    GEngine->AddOnScreenDebugMessage(
        10,
        0.0f,
        PromptColor,
        PromptText
    );
}