#pragma once

#include "CoreMinimal.h"

struct FBody
{
	FVector2D Position = FVector2D::ZeroVector;		// in m	
	FVector2D Velocity = FVector2D::ZeroVector;		// in m/s

	FVector2D Acceleration = FVector2D::ZeroVector;	// in m/s^2
	FVector2D NetForce = FVector2D::ZeroVector;		// in N

	double Mass = 1.0;								// in kg

	bool bXFixed = false;
	bool bYFixed = false;
};

class ISysElement
{
public:
	virtual ~ISysElement() = default;

	virtual void GetConnectedBodies(
		int32& OutBodyA,
		int32& OutBodyB
	) const = 0;

	virtual void ApplyForces(TArray<FBody>& Bodies) const = 0;
};