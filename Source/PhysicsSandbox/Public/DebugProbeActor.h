// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "DebugProbeActor.generated.h"

// Forward declarations
class USceneComponent;
class UStaticMeshComponent;
class UCameraComponent;
class USpringArmComponent;
class UDebugProbeControlWidget;
class APlayerController;

UENUM()
enum class ECameraMode : uint8
{
	Onboard,
	Chase,
	Top,
	Free
};

UCLASS()
class PHYSICSSANDBOX_API ADebugProbeActor : public AActor
{
	GENERATED_BODY()

public:
	ADebugProbeActor();

	// Public parameter API
	UFUNCTION(BlueprintCallable, Category = "Simulation|Trajectory")
	void SetOmegaTarget(float NewOmega);

	UFUNCTION(BlueprintPure, Category = "Simulation|Trajectory")
	float GetOmegaTarget() const { return Omega; }

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

private:
	// =========================
	// Components
	// =========================
	UPROPERTY(VisibleAnywhere, Category = "Components")
	USceneComponent* SceneRoot = nullptr;

	UPROPERTY(VisibleAnywhere, Category = "Components")
	UStaticMeshComponent* Mesh = nullptr;

	UPROPERTY(VisibleAnywhere, Category = "Components")
	UCameraComponent* OnboardCamera = nullptr;

	UPROPERTY(VisibleAnywhere, Category = "Components")
	USpringArmComponent* SpringArm = nullptr;

	UPROPERTY(VisibleAnywhere, Category = "Components")
	UCameraComponent* ChaseCamera = nullptr;

	UPROPERTY(VisibleAnywhere, Category = "Components")
	UCameraComponent* TopCamera = nullptr;

	// =========================
	// UI / Controller references
	// =========================
	APlayerController* PlayerController = nullptr;

	UPROPERTY(EditAnywhere, Category = "UI")
	TSubclassOf<UDebugProbeControlWidget> ControlWidgetClass;

	UPROPERTY()
	UDebugProbeControlWidget* ControlWidget = nullptr;

	// =========================
	// Interaction / camera state
	// =========================
	ECameraMode CameraMode = ECameraMode::Onboard;

	bool bUIInputMode = false;
	bool bReleased = false;

	// =========================
	// Simulation state
	// =========================
	int FrameCount = 0;
	float RunningTime = 0.0f;
	float AverageDeltaTime = 0.0f;
	float Time = 0.0f; // Tiempo físico desde que el actor comienza a moverse
	float Theta = 0.0f; // rad

	FVector Pos_0 = FVector::ZeroVector; // cm
	FVector Pos_Tick = FVector::ZeroVector; // cm
	FVector Pos_prevTick = FVector::ZeroVector; // cm
	FVector Vel_Tick = FVector::ZeroVector; // cm/s

	FVector ChaseCamPos_Tick = FVector::ZeroVector; // cm
	FVector ChaseCamPos_prevTick = FVector::ZeroVector; // cm

	// =========================
	// Mesh
	// =========================
	UPROPERTY(EditAnywhere, Category = "Mesh")
	FRotator MeshRotationOffset = FRotator(0.0f, 0.0f, 0.0f); // (Pitch, Yaw, Roll)

	// =========================
	// Simulation: general
	// =========================
	UPROPERTY(EditAnywhere, Category = "Simulation")
	float g = -980.0f; // cm/s^2 (gravedad terrestre en Unreal)

	// =========================
	// Simulation: trajectory parameters
	// =========================
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation|Trajectory")
	float Omega = PI / 4; // Angular frequency (rad/s)

private:
	float OmegaTransitionStart = Omega;
	float OmegaTarget = Omega;
	float OmegaTransitionElapsedTime = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Simulation|Trajectory")
	float OmegaTransitionDuration = 0.7f; // s

	bool bOmegaTransitionActive = false;

	UPROPERTY(EditAnywhere, Category = "Simulation|Trajectory")
	float Radius = 500.0f; // cm

	UPROPERTY(EditAnywhere, Category = "Simulation|Trajectory")
	float Vel_Z = 100.0f; // cm/s

	// =========================
	// Simulation: rotation parameters
	// =========================
	UPROPERTY(EditAnywhere, Category = "Simulation|Rotation")
	bool bSpinAroundForward = true;

	UPROPERTY(EditAnywhere, Category = "Simulation|Rotation")
	float RollRateDeg = 45.0f; // deg/s

	UPROPERTY(EditAnywhere, Category = "Simulation|Rotation")
	float RollAngleDeg = 0.0f; // deg

	// =========================
	// Debug settings
	// =========================
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bEnableDebugDraw = true;

	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawTrajectory = true;

	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawVelocityVector = false;

	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawChaseCameraTrajectory = true;

	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawChaseCameraMarker = false;

	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawActorFrame = true;

	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawMeshFrame = false;

	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawChaseCameraFrame = true;

	UPROPERTY(EditAnywhere, Category = "Debug")
	float BodyFrameAxisLength = 200.0f; // cm

	UPROPERTY(EditAnywhere, Category = "Debug")
	float VelocityArrowScale = 1.0f; // Velocity vector scale factor

	// =========================
	// Internal methods: input / camera / UI
	// =========================
	void ReleaseActor();
	void ToggleCamera();
	void ApplyCameraMode();
	void ToggleInputMode();
	void ApplyInputMode();

	// =========================
	// Internal methods: simulation
	// =========================
	FVector ComputeHelixPosition();
	FVector ComputeVelocityVector(float DeltaTime) const;
	void UpdateActorRotation(float DeltaTime);
	void UpdateOmegaTransition(float DeltaTime);
	float ComputeQuinticSmoothStep(float Alpha) const;

	// =========================
	// Internal methods: debug draw / text
	// =========================
	void DrawActorTrajectory() const;
	void DrawChaseCameraTrajectory() const;
	void DrawVelocityVector() const;

	void DrawComponentFrame(
		const USceneComponent* Component,
		FColor ColorForward,
		FColor ColorRight,
		FColor ColorUp,
		float LineThickness
	) const;

	void DrawActorFrame() const;
	void DrawMeshFrame() const;
	void DrawChaseCameraFrame() const;
	void PrintDebugInfo() const;
	FString GetCameraModeString() const;
	const TCHAR* BoolToTEXT(bool bValue) const;
};