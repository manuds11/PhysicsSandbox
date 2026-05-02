// Fill out your copyright notice in the Description page of Project Settings.

#include "DebugProbeActor.h"
#include "DebugProbeControlWidget.h"

#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Math/Quat.h"
#include "Math/RotationMatrix.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Math/Units.h"


ADebugProbeActor::ADebugProbeActor()
{
    PrimaryActorTick.bCanEverTick = true;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    RootComponent = SceneRoot;
    RootComponent->SetRelativeRotation(FRotator(0.0f, 0.0f, 0.0f)); // 🔴 OFFSET INICIAL ACTOR

    Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
    Mesh->SetupAttachment(SceneRoot);
    Mesh->SetSimulatePhysics(false);
    Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    OnboardCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("OnboardCamera"));
    OnboardCamera->SetupAttachment(SceneRoot);
    OnboardCamera->SetRelativeLocation(FVector(-300.0f, 0.0f, 100.0f));
    OnboardCamera->SetRelativeRotation(FRotator(0.0f, 0.0f, 0.0f));

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(SceneRoot);

    // Configuración básica
    SpringArm->TargetArmLength = 600.0f;                            // distancia
    SpringArm->SetRelativeRotation(FRotator(-10.0f, 90.0f, 0.0f));  // ligera inclinación hacia abajo
    SpringArm->bUsePawnControlRotation = false;
    SpringArm->bInheritPitch = false;
    SpringArm->bInheritYaw = false;
    SpringArm->bInheritRoll = false;

    ChaseCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCamera"));
    ChaseCamera->SetupAttachment(SpringArm);

    TopCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("TopCamera"));
    TopCamera->SetupAttachment(SceneRoot);
    TopCamera->SetRelativeRotation(FRotator(90.0f, 0.0f, 0.0f));    // Mirando hacia arriba (+Z)
}

void ADebugProbeActor::BeginPlay()
{
    Super::BeginPlay();
    
    Mesh->SetRelativeRotation(MeshRotationOffset);   // 🔴 

    Pos_0 = GetActorLocation();
    Pos_Tick = Pos_0;
    Pos_prevTick = Pos_0;

    PlayerController = GetWorld()->GetFirstPlayerController();

    // Cameras
    if (ChaseCamera)
    {
        ChaseCamPos_Tick = ChaseCamera->GetComponentLocation();
        ChaseCamPos_prevTick = ChaseCamPos_Tick;
    }

    ApplyCameraMode();

    // Widget
    if (ControlWidgetClass && PlayerController)
    {
        ControlWidget = CreateWidget<UDebugProbeControlWidget>(PlayerController, ControlWidgetClass);

        if (ControlWidget)
        {
            ControlWidget->SetProbeReference(this);
            ControlWidget->AddToViewport();
        }
    }

    ApplyInputMode();

    EnableInput(PlayerController);
    if (InputComponent)
    {
        InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &ADebugProbeActor::ReleaseActor);
        InputComponent->BindKey(EKeys::C, IE_Pressed, this, &ADebugProbeActor::ToggleCamera);
        InputComponent->BindKey(EKeys::Tab, IE_Pressed, this, &ADebugProbeActor::ToggleInputMode);
        InputComponent->BindKey(EKeys::Escape, IE_Pressed, this, &ADebugProbeActor::QuitGame);
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

void ADebugProbeActor::ToggleCamera()
{
    switch (CameraMode)
    {
    case ECameraMode::Onboard:
        CameraMode = ECameraMode::Chase;
        break;

    case ECameraMode::Chase:
        CameraMode = ECameraMode::Top;
        break;

    case ECameraMode::Top:
        CameraMode = ECameraMode::Free;
        break;

    case ECameraMode::Free:
        CameraMode = ECameraMode::Onboard;
        break;
    }

    ApplyCameraMode();
}

void ADebugProbeActor::ApplyCameraMode()
{
    if (!PlayerController) return;

    // Desactivar todas
    OnboardCamera->SetActive(false);
    ChaseCamera->SetActive(false);
    TopCamera->SetActive(false);

    switch (CameraMode)
    {
    case ECameraMode::Onboard:
        PlayerController->SetViewTarget(this);
        OnboardCamera->SetActive(true);
        break;

    case ECameraMode::Chase:
        PlayerController->SetViewTarget(this);
        ChaseCamera->SetActive(true);
        break;

    case ECameraMode::Top:
    {
        PlayerController->SetViewTarget(this);

        TopCamera->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);

        TopCamera->SetWorldLocation(FVector(0.0f, 0.0f, 0.0f));
        TopCamera->SetWorldRotation(FRotator(90.0f, 0.0f, 0.0f));

        TopCamera->SetActive(true);
        break;
    }

    case ECameraMode::Free:
        PlayerController->SetViewTarget(nullptr); // vuelve al editor
        break;
    }

    if (CameraMode == ECameraMode::Chase)
    {
        bDrawChaseCameraFrame = false;
    }
    else
    {
        bDrawChaseCameraFrame = bDrawChaseCameraFrame_Default;
    }
}

void ADebugProbeActor::ToggleInputMode()
{
    bUIInputMode = !bUIInputMode;
    ApplyInputMode();
}

void ADebugProbeActor::ApplyInputMode()
{
    if (!PlayerController)
    {
        return;
    }

    if (bUIInputMode)
    {
        FInputModeGameAndUI InputMode;

        if (ControlWidget)
        {
            InputMode.SetWidgetToFocus(ControlWidget->TakeWidget());
        }

        InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        InputMode.SetHideCursorDuringCapture(false);

        PlayerController->SetInputMode(InputMode);
        PlayerController->bShowMouseCursor = true;
    }
    else
    {
        FInputModeGameOnly InputMode;
        PlayerController->SetInputMode(InputMode);
        PlayerController->bShowMouseCursor = false;
    }
}

void ADebugProbeActor::QuitGame()
{
    UKismetSystemLibrary::QuitGame(
        GetWorld(),
        PlayerController,
        EQuitPreference::Quit,
        false
    );
}

FVector ADebugProbeActor::ComputeHelixPosition(float DeltaTime)
{
    Theta += OmegaParam.Current * DeltaTime;
    ZOffset += VelZParam.Current * DeltaTime;

    const float X = RadiusParam.Current * FMath::Cos(Theta) - RadiusParam.Current;
    const float Y = RadiusParam.Current * FMath::Sin(Theta);
   
    const FVector Offset(X, Y, ZOffset);
    return Pos_0 + Offset;
}

FVector ADebugProbeActor::ComputeVelocityVector(float DeltaTime) const 
{
    if (DeltaTime <= KINDA_SMALL_NUMBER)
    {
        return FVector::ZeroVector;
    }
    return (Pos_Tick - Pos_prevTick) / DeltaTime;
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

    if (bEnableDebugDraw)
    {
        if (bDrawActorFrame)
        {
            DrawActorFrame();
        }
        if (bDrawMeshFrame)
        {
            DrawMeshFrame();
        }
        if (bDrawChaseCameraFrame)
        {
            DrawChaseCameraFrame();
        }
    }
    
    if (bReleased) 
    {
        MotionTime += DeltaTime;

        OmegaParam.Update(DeltaTime);       // For Transitioning between different Omegas.
        RadiusParam.Update(DeltaTime);      // Same for Radius transitioning.
        VelZParam.Update(DeltaTime); 

        Pos_Tick = ComputeHelixPosition(DeltaTime);
        Vel_Tick = ComputeVelocityVector(DeltaTime);
        
        UpdateActorRotation(DeltaTime);
        SetActorLocation(Pos_Tick);

        if (ChaseCamera)
        {
            ChaseCamPos_Tick = ChaseCamera->GetComponentLocation();
        }

        if (bEnableDebugDraw)
        {
            if (bDrawTrajectory)
            {
                DrawActorTrajectory();
            }
            if (bDrawVelocityVector)
            {
                DrawVelocityVector();
            }
            if (bDrawChaseCameraTrajectory)
            {
                DrawChaseCameraTrajectory();
            }

        }
            
        Pos_prevTick = Pos_Tick;
        ChaseCamPos_prevTick = ChaseCamPos_Tick;
    }

    PrintDebugInfo();
}

void ADebugProbeActor::DrawActorTrajectory() const
{
    DrawDebugLine(
        GetWorld(),        // UWorld* → contexto del mundo donde dibujar
        Pos_prevTick,      // FVector → punto inicial de la línea (posición anterior)
        Pos_Tick,          // FVector → punto final de la línea (posición actual)
        FColor::Blue,      // FColor → color de la línea
        false,             // bool bPersistentLines → si la línea es permanente (false = temporal)
        TrajectoryLifeTime,             // float LifeTime → tiempo en segundos que permanece visible
        0,                 // uint8 DepthPriority → prioridad de render (0 = normal)
        2.0f               // float Thickness → grosor de la línea
    );
}

void ADebugProbeActor::DrawChaseCameraTrajectory() const
{
    if (!GetWorld() || !ChaseCamera)
    {
        return;
    }

    DrawDebugLine(
        GetWorld(),
        ChaseCamPos_prevTick,
        ChaseCamPos_Tick,
        FColor::Yellow,
        false,
        TrajectoryLifeTime,
        0,
        2.0f
    );

    if (bDrawChaseCameraMarker)
    {
        DrawDebugSphere(
            GetWorld(),
            ChaseCamPos_Tick,
            20.0f,
            12,
            FColor::Yellow,
            false,
            0.0f,
            0,
            2.0f
        );
    }
}

void ADebugProbeActor::DrawVelocityVector() const {
    
    if (!Vel_Tick.IsNearlyZero())
    {
        const FVector ArrowEnd = Pos_Tick + Vel_Tick * VelocityArrowScale;

        DrawDebugDirectionalArrow(
            GetWorld(),        // UWorld* → contexto del mundo
            Pos_Tick,          // FVector → inicio de la flecha (posición actual)
            ArrowEnd,          // FVector → final de la flecha (dirección + magnitud escalada)
            1000.0f,             // float ArrowSize → tamaño de la punta de la flecha
            FColor::Black,       // FColor → color de la flecha
            false,             // bool bPersistentLines → si la flecha es persistente
            0.0f,              // float LifeTime → 0 = solo un frame
            0,                 // uint8 DepthPriority → prioridad de render
            5.0f               // float Thickness → grosor de la flecha
        );
    }
}

void ADebugProbeActor::DrawComponentFrame(
    const USceneComponent* Component,
    FColor ColorForward,
    FColor ColorRight,
    FColor ColorUp,
    float LineThickness
    ) const
{
    if (!Component || !GetWorld())
    {
        return;
    }

    const FVector Origin = Component->GetComponentLocation();

    const FVector ForwardEnd = Origin + Component->GetForwardVector() * BodyFrameAxisLength;
    const FVector RightEnd = Origin + Component->GetRightVector() * BodyFrameAxisLength;
    const FVector UpEnd = Origin + Component->GetUpVector() * BodyFrameAxisLength;

    DrawDebugLine(GetWorld(), Origin, ForwardEnd, ColorForward, false, 0.0f, 0, LineThickness);
    DrawDebugLine(GetWorld(), Origin, RightEnd, ColorRight, false, 0.0f, 0, LineThickness);
    DrawDebugLine(GetWorld(), Origin, UpEnd, ColorUp, false, 0.0f, 0, LineThickness);
}

void ADebugProbeActor::DrawActorFrame() const
{
    DrawComponentFrame(
        RootComponent,
        FColor::Red,
        FColor::Green,
        FColor::Blue,
        3.0f
    );
}

void ADebugProbeActor::DrawMeshFrame() const
{
    DrawComponentFrame(
        Mesh,
        FColor(160, 0, 200),   // morado → forward
        FColor::Yellow,    // gris oscuro → right
        FColor(0, 0, 0),       // negro → up
        1.5f
    );
}

void ADebugProbeActor::DrawChaseCameraFrame() const
{
    DrawComponentFrame(
        ChaseCamera,
        FColor::Red,
        FColor::Green,
        FColor::Blue,
        3.0f
    );
}

void ADebugProbeActor::PrintDebugInfo() const
{
    if (!GEngine)
    {
        return;
    }

    const FVector PosMeters = Pos_Tick * Units::CmToM;
    const FVector VelMeters = Vel_Tick * Units::CmToM;

    const FRotator ActorRot = GetActorRotation();

    const FRotator MeshRelRot = Mesh ? Mesh->GetRelativeRotation() : FRotator::ZeroRotator;
    const FRotator MeshWorldRot = Mesh ? Mesh->GetComponentRotation() : FRotator::ZeroRotator;

    const FVector ActorFwd = GetActorForwardVector();
    const FVector MeshFwd = Mesh ? Mesh->GetForwardVector() : FVector::ZeroVector;
    
    // AddOnScreenDebugMessage displays these blocks in reverse visual order,
    // so they are intentionally called from bottom block to top block.

    GEngine->AddOnScreenDebugMessage(
        11,
        0.0f,
        FColor::Cyan,
        FString::Printf(
            TEXT("\nInput Mode: %s"
                "\n"
                "t (s): %.3f"
                "\nPos (m) [X Y Z]: %.2f | %.2f | %.2f"
                "\nVel (m/s) [X Y Z]: %.2f | %.2f | %.2f"
                "\nRadius (m): %.2f | Angularfreq (rad/s): %.2f"
                "\n"
                "\nCamera Mode: %s"
                "\n"
                "\nActor Rot [P Y R]: %.2f | %.2f | %.2f"
                "\nMesh RelRot [P Y R]: %.2f | %.2f | %.2f"
                "\nMesh WorldRot [P Y R]: %.2f | %.2f | %.2f"
                "\n"
                "\nActor Fwd: %.2f | %.2f | %.2f"
                "\nMesh  Fwd: %.2f | %.2f | %.2f"),
            bUIInputMode ? TEXT("UI") : TEXT("FIXED"),
            
            MotionTime,
            PosMeters.X, PosMeters.Y, PosMeters.Z,
            VelMeters.X, VelMeters.Y, VelMeters.Z,
            RadiusParam.Current* Units::CmToM, OmegaParam.Current,

            *GetCameraModeString(),

            ActorRot.Pitch, ActorRot.Yaw, ActorRot.Roll,
            MeshRelRot.Pitch, MeshRelRot.Yaw, MeshRelRot.Roll,
            MeshWorldRot.Pitch, MeshWorldRot.Yaw, MeshWorldRot.Roll,

            ActorFwd.X, ActorFwd.Y, ActorFwd.Z,
            MeshFwd.X, MeshFwd.Y, MeshFwd.Z
        )
    );

    FString PromptText;

    if (!bReleased)
    {
        PromptText = FString::Printf(
            TEXT("CLICK ANYWHERE AND PRESS SPACE BAR TO RELEASE\n"
                "Avg. DeltaTime: %.4f\n"
                "\n"
                "CONTROLS\n"
                "Space : Start simulation\n"
                "C     : Change camera\n"
                "Tab   : Toggle UI/Game\n"
                "Esc   : Exit"),
            AverageDeltaTime
        );
    }
    else
    {
        PromptText = FString::Printf(
            TEXT("HELICAL TRANSLATION ACTIVE\n"
                "Running Time: %.2f | Avg. dt: %.4f\n"
                "\n"
                "CONTROLS\n"
                "C     : Change camera\n"
                "Tab   : Toggle UI/Game\n"
                "Esc   : Exit"),
            RunningTime,
            AverageDeltaTime
        );
    }

    FColor PromptColor = !bReleased ? FColor::Red : FColor::Green;

    GEngine->AddOnScreenDebugMessage(
        10,
        0.0f,
        PromptColor,
        PromptText
    );
}

FString ADebugProbeActor::GetCameraModeString() const
{
    switch (CameraMode)
    {
    case ECameraMode::Onboard:
        return TEXT("ONBOARD");

    case ECameraMode::Chase:
    {
        return FString::Printf(
            TEXT("CHASE [P:%s Y:%s R:%s]"),
            BoolToTEXT(SpringArm && SpringArm->bInheritPitch),
            BoolToTEXT(SpringArm && SpringArm->bInheritYaw),
            BoolToTEXT(SpringArm && SpringArm->bInheritRoll)
        );
    }

    case ECameraMode::Top:
        return TEXT("BOTTOM");

    case ECameraMode::Free:
        return TEXT("FREE");

    default:
        return TEXT("UNKNOWN");
    }
}

const TCHAR* ADebugProbeActor::BoolToTEXT(bool bValue) const
{
    return bValue ? TEXT("True") : TEXT("False");
}