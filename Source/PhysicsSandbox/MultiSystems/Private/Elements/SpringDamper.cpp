#include "Elements/SpringDamper.h"

// -----------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------

FSpringDamper::FSpringDamper(
	int32 InBodyAIndex,
	int32 InBodyBIndex,
	double InStiffness,
	double InDamping,
	double InRestLength
)
	: BodyAIndex(InBodyAIndex)
	, BodyBIndex(InBodyBIndex)
	, Stiffness(InStiffness)
	, Damping(InDamping)
	, RestLength(InRestLength)
{
}

// -----------------------------------------------------------------------------
// ISysElement interface
// -----------------------------------------------------------------------------

ESysElementType FSpringDamper::GetElementType() const
{
	return ESysElementType::SpringDamper;
}

bool FSpringDamper::GetConnectedBodies(
	int32& OutBodyA,
	int32& OutBodyB
) const
{
	OutBodyA = BodyAIndex;
	OutBodyB = BodyBIndex;

	return true;
}

// -----------------------------------------------------------------------------
// Simulation
// -----------------------------------------------------------------------------

void FSpringDamper::ApplyForces(TArray<FBody>& Bodies)
{
	check(Bodies.IsValidIndex(BodyAIndex));
	check(Bodies.IsValidIndex(BodyBIndex));

	FBody& A = Bodies[BodyAIndex];
	FBody& B = Bodies[BodyBIndex];

	const FVector2D Delta = B.Position - A.Position;
	const double Length = Delta.Size();				// Módulo de Delta

	if (Length <= UE_SMALL_NUMBER)
	{
		return;
	}

	const FVector2D Direction = Delta / Length;		// La dirección es el vector unitario que apunta de A a B
	const double Extension = Length - RestLength;

	const FVector2D RelativeVelocity = B.Velocity - A.Velocity;
	const double RelativeSpeed = FVector2D::DotProduct(RelativeVelocity, Direction ); // Proyección de la velocidad sobre la dirección del muelle

	const double TotalForce =
		Stiffness * Extension + Damping * RelativeSpeed;

	const FVector2D Force = TotalForce * Direction;

	A.NetForce += Force;
	B.NetForce -= Force;
}

// -----------------------------------------------------------------------------
// Visualization data
// -----------------------------------------------------------------------------


double FSpringDamper::ComputePotentialEnergy(
	const TArray<FBody>& Bodies
) const
{
	check(Bodies.IsValidIndex(BodyAIndex));
	check(Bodies.IsValidIndex(BodyBIndex));

	const FVector2D Delta =
		Bodies[BodyBIndex].Position
		- Bodies[BodyAIndex].Position;

	const double Extension =
		Delta.Size() - RestLength;

	return 0.5 * Stiffness * FMath::Square(Extension);
}


bool FSpringDamper::GetEquilibriumPoint(
	const TArray<FBody>& Bodies,
	FVector2D& OutPoint
) const
{
	if (!Bodies.IsValidIndex(BodyAIndex) || !Bodies.IsValidIndex(BodyBIndex))
	{
		OutPoint = FVector2D::ZeroVector;
		return false;
	}

	const FBody& A = Bodies[BodyAIndex];
	const FBody& B = Bodies[BodyBIndex];

	const FVector2D Delta = B.Position - A.Position;
	const double Length = Delta.Size();

	if (Length <= UE_SMALL_NUMBER)
	{
		OutPoint = A.Position;
		return true;
	}

	const FVector2D Direction = Delta / Length;

	OutPoint = A.Position + RestLength * Direction;

	return true;
}