#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SimuCameraController.generated.h"

class A1AxisCamera;
class APlayerController;
class APawn;

UCLASS()
class PHYSICSSANDBOX_API ASimuCameraController
	: public AActor
{
	GENERATED_BODY()

public:

	// -------------------------------------------------------------------------
	// Construction
	// -------------------------------------------------------------------------

	ASimuCameraController();

	// -------------------------------------------------------------------------
	// Unreal lifecycle
	// -------------------------------------------------------------------------

	virtual void BeginPlay() override;

	virtual void Tick(
		float DeltaTime
	) override;

private:

	// -------------------------------------------------------------------------
	// Camera switching
	// -------------------------------------------------------------------------

	void ToggleCamera();

	void Activate1AxisCamera();

	void ActivateFreeCamera();

	// -------------------------------------------------------------------------
	// Camera references
	// -------------------------------------------------------------------------

	UPROPERTY(
		EditInstanceOnly,
		Category = "Camera"
	)
	A1AxisCamera* OneAxisCamera =
		nullptr;

	APlayerController* PlayerController =
		nullptr;

	APawn* FreeCameraPawn =
		nullptr;

	// -------------------------------------------------------------------------
	// Runtime state
	// -------------------------------------------------------------------------

	bool bUsing1AxisCamera =
		true;
};