#include "Elements/SpringDamper.h"

FSpringDamper::FSpringDamper(
	int32 InBodyA,
	int32 InBodyB,
	double InStiffness,
	double InDamping,
	double InRestLength
)
	: BodyA(InBodyA)
	, BodyB(InBodyB)
	, Stiffness(InStiffness)
	, Damping(InDamping)
	, RestLength(InRestLength)
{
}

void FSpringDamper::ApplyForces(TArray<FBody>& Bodies) const
{
	if (!Bodies.IsValidIndex(BodyA) || !Bodies.IsValidIndex(BodyB))
	{
		return;
	}

	FBody& A = Bodies[BodyA];
	FBody& B = Bodies[BodyB];

	const FVector2D Delta = B.Position - A.Position;
	const double Length = Delta.Size();

	if (Length <= UE_SMALL_NUMBER)
	{
		return;
	}

	const FVector2D Direction = Delta / Length;
	const double Extension = Length - RestLength;

	const FVector2D RelativeVelocity = B.Velocity - A.Velocity;
	const double RelativeSpeed = FVector2D::DotProduct(RelativeVelocity, Direction);

	const double TotalForce =
		Stiffness * Extension + Damping * RelativeSpeed;

	const FVector2D Force = TotalForce * Direction;

	A.NetForce += Force;
	B.NetForce -= Force;
}