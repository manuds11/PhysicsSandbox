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

struct FTransParam
{
	float Current = 0.0f;
	float Target = 0.0f;
	float Start = 0.0f;

	float ElapsedTime = 0.0f;
	float TransDuration = 0.7f;

	bool bActive = false;

	FTransParam() = default;

	explicit FTransParam(float InitialValue)
		: Current(InitialValue)
		, Target(InitialValue)
		, Start(InitialValue)
	{
	}

	float GetTarget() const { return Target; }

	void SetTarget(float NewTarget)
	{
		Start = Current;
		Target = NewTarget;
		ElapsedTime = 0.0f;
		bActive = true;
	}

	void Update(float DeltaTime)
	{
		if (!bActive) return;

		ElapsedTime += DeltaTime;

		const float Alpha = FMath::Clamp(ElapsedTime / TransDuration, 0.0f, 1.0f);
		// Quintic smoothstep: S(a) = 6a^5 - 15a^4 + 10a^3
		const float A2 = Alpha * Alpha;
		const float A3 = A2 * Alpha;
		const float SmoothStep =
			A3 * (10.0f - 15.0f * Alpha + 6.0f * A2);

		Current = FMath::Lerp(Start, Target, SmoothStep);

		if (Alpha >= 1.0f)
		{
			Current = Target;
			bActive = false;
		}
	}
};

UCLASS()
class PHYSICSSANDBOX_API ADebugProbeActor : public AActor
{
	GENERATED_BODY()

public:
	ADebugProbeActor();

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
	float MotionTime = 0.0f; // Tiempo físico desde que el actor comienza a moverse
	float Theta = 0.0f; // rad
	float ZOffset = 0.0f;

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
	// =========================
	// Widget controllable parameters
	// =========================
	FTransParam OmegaParam{ PI / 4 };   // rad/s
	FTransParam RadiusParam{ 500.0f };  // cm
	FTransParam VelZParam{ 100.0f };	// cm/s
private:
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
	bool DrawChaseCameraFrame_Default = true;
	bool bDrawChaseCameraFrame = DrawChaseCameraFrame_Default;

	UPROPERTY(EditAnywhere, Category = "Debug")
	float TrajectoryLifeTime = 10.0f; // s

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
	FVector ComputeHelixPosition(float DeltaTime);
	FVector ComputeVelocityVector(float DeltaTime) const;
	void UpdateActorRotation(float DeltaTime);

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