#pragma once

#include "CoreMinimal.h"

struct FBody
{
	FVector2D Position = FVector2D::ZeroVector;
	FVector2D Velocity = FVector2D::ZeroVector;

	FVector2D Acceleration = FVector2D::ZeroVector;
	FVector2D NetForce = FVector2D::ZeroVector;

	double Mass = 1.0;

	bool bXFixed = false;
	bool bYFixed = false;
};

struct FPort
{
	FName Name;
	int32 BodyIndex = INDEX_NONE;
	FVector2D LocalOffset = FVector2D::ZeroVector;
};

class ISysElement
{
public:
	virtual ~ISysElement() = default;

	virtual void ApplyForces(TArray<FBody>& Bodies) const = 0;
};