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

	static FBody Fixed(
		const FVector2D& Position
	)
	{
		FBody Body;

		Body.Position = Position;

		Body.bXFixed = true;
		Body.bYFixed = true;

		return Body;
	}

	static FBody SlidingMassX(
		const FVector2D& Position,
		double Mass
	)
	{
		FBody Body;

		Body.Position = Position;
		Body.Mass = Mass;

		Body.bYFixed = true;

		return Body;
	}

	static FBody Free(
		const FVector2D& Position,
		double Mass
	)
	{
		FBody Body;

		Body.Position = Position;
		Body.Mass = Mass;

		return Body;
	}
};

enum class ESysElementType : uint8
{
	SpringDamper,
	PendulumRod
};

class ISysElement
{
public:
	virtual ~ISysElement() = default;

	virtual ESysElementType GetElementType() const = 0;

	virtual bool GetConnectedBodies(
		int32& OutBodyA,
		int32& OutBodyB
	) const
	{
		return false;
	}

	virtual void ApplyForces(TArray<FBody>& Bodies) = 0;

	virtual FColor GetDebugColor() const = 0;

	virtual bool GetEquilibriumPoint(
		const TArray<FBody>& Bodies,
		FVector2D& OutPoint
	) const
	{
		return false;
	}
};