#pragma once

#include "CoreMinimal.h"
#include "Camera/CameraActor.h"
#include "1AxisCamera.generated.h"

UCLASS()
class PHYSICSSANDBOX_API A1AxisCamera
	: public ACameraActor
{
	GENERATED_BODY()

public:

	// -------------------------------------------------------------------------
	// Construction
	// -------------------------------------------------------------------------

	A1AxisCamera();

	// -------------------------------------------------------------------------
	// Unreal lifecycle
	// -------------------------------------------------------------------------

	virtual void Tick(
		float DeltaTime
	) override;

private:

	// -------------------------------------------------------------------------
	// Movement
	// -------------------------------------------------------------------------

	void UpdateMovement(
		double Dt
	);

	UPROPERTY(
		EditAnywhere,
		Category = "Camera"
	)
	double MoveSpeed = 200.0;
};