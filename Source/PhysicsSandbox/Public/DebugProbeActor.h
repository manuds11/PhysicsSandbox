// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/SpringArmComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraComponent.h"

#include "DebugProbeActor.generated.h"

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

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

private:
	// Components
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

	// Actor::Tick variables
	int FrameCount = 0;
	float RunningTime = 0.0f; //
	float AverageDeltaTime = 0.0f;
	float Time = 0.0f; // Tiempo físico desde que el actor comienza a moverse

	// Reference frame
	FVector Pos_0 = FVector::ZeroVector;
	FVector Pos_Tick = FVector::ZeroVector;
	FVector Pos_prevTick = FVector::ZeroVector;
	FVector Vel_Tick = FVector::ZeroVector;
	FVector ChaseCamPos_Tick = FVector::ZeroVector;
	FVector ChaseCamPos_prevTick = FVector::ZeroVector;

	// Camera variables
	UPROPERTY(EditAnywhere, Category = "Cameras")
	bool bUseOnboardCamera = true;
	
	ECameraMode CameraMode = ECameraMode::Onboard;

	// Action variables
	bool bReleased = false;
	
	// EDITABLE
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bEnableDebugDraw = true;
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawTrajectory = true;
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawVelocityVector = false;
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawChaseCameraTrajectory = true;
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawActorFrame = true;
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawMeshFrame = false;
	UPROPERTY(EditAnywhere, Category = "Debug")
	float BodyFrameAxisLength = 200.0f;
	UPROPERTY(EditAnywhere, Category = "Debug")
	float VelocityArrowScale = 1.0f;							// Scale factor of speed module. 1 means its real size.
	
	// Physical variables
	UPROPERTY(EditAnywhere, Category = "Physics")
	FRotator MeshRotationOffset = FRotator(0.0f, 0.0f, 0.0f);	// (Pitch, Yaw, Roll) -- En el UE editor se muestran en distinto orden.
	UPROPERTY(EditAnywhere, Category = "Physics")
	float g = -980.0f;											// g = 980 cm/s^2 Para gravedad terrestre en unreal
	
	// Trajectory parameters
	UPROPERTY(EditAnywhere, Category = "Physics")
	float Radius = 500.0f;
	UPROPERTY(EditAnywhere, Category = "Physics")
	float Omega = PI/4;											// Angular freq (rad/s)
	UPROPERTY(EditAnywhere, Category = "Physics")
	float Vel_Z = 100.0f;										// cm/s
	// Rotation
	UPROPERTY(EditAnywhere, Category = "Rotation")
	bool bSpinAroundForward = true;
	UPROPERTY(EditAnywhere, Category = "Rotation")
	float RollRateDeg = 45.0f;									// deg/s
	UPROPERTY(EditAnywhere, Category = "Rotation")
	float RollAngleDeg = 0.0f;

	// METHODS
	void ReleaseActor();
	void ToggleCamera();
	void ApplyCameraMode();
	FVector ComputeHelixPosition(float CurrentTime) const;
	FVector ComputeVelocityVector(float dt) const;
	void UpdateActorRotation(float DeltaTime);
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
	void PrintDebugInfo() const;
	FString GetCameraModeString() const;
	const TCHAR* BoolToTEXT(bool bValue) const;
};