#include "Actors/SimuCameraController.h"

#include "Actors/1AxisCamera.h"

#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"

// -----------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------

ASimuCameraController::ASimuCameraController()
{
	PrimaryActorTick.bCanEverTick =
		true;
}

// -----------------------------------------------------------------------------
// Unreal lifecycle
// -----------------------------------------------------------------------------

void ASimuCameraController::BeginPlay()
{
	Super::BeginPlay();

	PlayerController =
		GetWorld()->GetFirstPlayerController();

	if (!PlayerController)
	{
		return;
	}

	FreeCameraPawn =
		PlayerController->GetPawn();

	Activate1AxisCamera();
}

void ASimuCameraController::Tick(
	float DeltaTime
)
{
	Super::Tick(
		DeltaTime
	);

	if (!PlayerController)
	{
		return;
	}

	if (
		PlayerController->WasInputKeyJustPressed(
			EKeys::C
		)
		)
	{
		ToggleCamera();
	}
}

// -----------------------------------------------------------------------------
// Camera switching
// -----------------------------------------------------------------------------

void ASimuCameraController::ToggleCamera()
{
	if (bUsing1AxisCamera)
	{
		ActivateFreeCamera();
	}
	else
	{
		Activate1AxisCamera();
	}
}

void ASimuCameraController::Activate1AxisCamera()
{
	if (
		!PlayerController
		|| !OneAxisCamera
		)
	{
		return;
	}

	PlayerController->SetViewTarget(
		OneAxisCamera
	);

	PlayerController->SetIgnoreMoveInput(
		true
	);

	bUsing1AxisCamera =
		true;
}

void ASimuCameraController::ActivateFreeCamera()
{
	if (
		!PlayerController
		|| !FreeCameraPawn
		)
	{
		return;
	}

	PlayerController->SetViewTarget(
		FreeCameraPawn
	);

	PlayerController->SetIgnoreMoveInput(
		false
	);

	bUsing1AxisCamera =
		false;
}