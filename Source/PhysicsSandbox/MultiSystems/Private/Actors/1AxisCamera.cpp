#include "Actors/1AxisCamera.h"

#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"

// -----------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------

A1AxisCamera::A1AxisCamera()
{
	PrimaryActorTick.bCanEverTick =
		true;
}

void A1AxisCamera::Tick(
	float DeltaTime
)
{
	Super::Tick(
		DeltaTime
	);

	UpdateMovement(
		static_cast<double>(
			DeltaTime
			)
	);
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

void A1AxisCamera::UpdateMovement(
	double Dt
)
{
	APlayerController* PlayerController =
		GetWorld()->GetFirstPlayerController();

	if (!PlayerController)
	{
		return;
	}

	double MovementInput =
		0.0;

	if (
		PlayerController->IsInputKeyDown(
			EKeys::W
		)
		)
	{
		MovementInput +=
			1.0;
	}

	if (
		PlayerController->IsInputKeyDown(
			EKeys::S
		)
		)
	{
		MovementInput -=
			1.0;
	}

	if (
		FMath::IsNearlyZero(
			MovementInput
		)
		)
	{
		return;
	}

	const FVector Movement =
		GetActorForwardVector()
		* MovementInput
		* MoveSpeed
		* Dt;

	AddActorWorldOffset(
		Movement
	);
}