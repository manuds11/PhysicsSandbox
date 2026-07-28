#include "Simulation/FullSys.h"
#include "Math/Units.h"
#include "Elements/PendulumRod.h"

// ------------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------

FFullSys::FFullSys()
	: Integrator(MakeUnique<FExplicitEulerSysIntegrator>())
{
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

void FFullSys::Initialize()
{
	AssembleRodSystem();
}

int32 FFullSys::AddBody(const FBody& Body)
{
	return Bodies.Add(Body);
}

int32 FFullSys::AddElement(TUniquePtr<ISysElement> Element)
{
	if (!Element)
	{
		return INDEX_NONE;
	}

	return Elements.Add(MoveTemp(Element));
}

FSubSys& FFullSys::CreateSubSys(FName Name)
{
	FSubSys NewSubSys;
	NewSubSys.Name = Name;

	const int32 SubSysIndex = SubSystems.Add(NewSubSys);

	return SubSystems[SubSysIndex];
}

void FFullSys::SetIntegrator(TUniquePtr<ISysIntegrator> InIntegrator)
{
	if (InIntegrator)
	{
		Integrator = MoveTemp(InIntegrator);
	}
}

// -----------------------------------------------------------------------------
// Getters
// -----------------------------------------------------------------------------

const TArray<FBody>& FFullSys::GetBodies() const
{
	return Bodies;
}

const TArray<TUniquePtr<ISysElement>>& FFullSys::GetElements() const
{
	return Elements;
}

const TArray<FSubSys>& FFullSys::GetSubSystems() const
{
	return SubSystems;
}

// -----------------------------------------------------------------------------
// MultiRod System
// -----------------------------------------------------------------------------

void FFullSys::AssembleRodSystem()
{
	CollectRods();

	const int32 NumRods =
		RodSystemArray.Num();

	RodSysMatrixA.SetNumZeroed(
		NumRods * NumRods
	);

	RodSysVectorB.SetNumZeroed(
		NumRods
	);

	RodSysVectorLambda.SetNumZeroed(
		NumRods
	);
}

void FFullSys::UpdateRodSystemValues()
{	
	// Check Matrix consistency
	const int32 NumRods =
		RodSystemArray.Num();

	check(
		RodSysMatrixA.Num()
		== NumRods * NumRods
	);

	check(
		RodSysVectorB.Num()
		== NumRods
	);

	check(
		RodSysVectorLambda.Num()
		== NumRods
	);
	
	for (double& Value : RodSysMatrixA)
	{
		Value = 0.0;
	}

	for (double& Value : RodSysVectorB)
	{
		Value = 0.0;
	}

	for (double& Value : RodSysVectorLambda)
	{
		Value = 0.0;
	}

	UpdateRodSystemJacobians();

	// Próximos pasos:
	// UpdateRodMatrixA();
	// UpdateRodVectorB();
}

void FFullSys::CollectRods()
{
	RodSystemArray.Reset();

	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		if (!Element)
		{
			continue;
		}

		if (
			Element->GetElementType()
			!= ESysElementType::PendulumRod
			)
		{
			continue;
		}

		FPendulumRod* PendulumRod =
			static_cast<FPendulumRod*>(
				Element.Get()
				);

		RodSystemArray.Add(PendulumRod);
	}
}

void FFullSys::UpdateRodSystemJacobians()
{
	for (FPendulumRod* PendulumRod : RodSystemArray)
	{
		if (!PendulumRod)
		{
			continue;
		}

		PendulumRod->UpdateRodJacobian(Bodies);
	}
}

double& FFullSys::MatrixIndex2ArrayIndex(
	int32 Row,
	int32 Column
)
{
	const int32 NumRods =
		RodSystemArray.Num();

	check(Row >= 0 && Row < NumRods);
	check(Column >= 0 && Column < NumRods);

	return RodSysMatrixA[ Row * NumRods + Column ];
}

const double& FFullSys::MatrixIndex2ArrayIndex(
	int32 Row,
	int32 Column
) const
{
	const int32 NumRods =
		RodSystemArray.Num();

	check(Row >= 0 && Row < NumRods);
	check(Column >= 0 && Column < NumRods);

	return RodSysMatrixA[
		Row * NumRods + Column
	];
}

// -----------------------------------------------------------------------------
// Simulation pipeline
// -----------------------------------------------------------------------------

void FFullSys::Step(double Dt)
{
	ClearForces();
	ApplyGravity();
	ApplyElementInteractions();

	UpdateRodSystemValues();

	ComputeAccelerations();
	Integrate(Dt);
	ProjectConstraintVelocities();
	ProjectConstraintPositions();
	ApplyFixedAxes();
}

// -----------------------------------------------------------------------------
// Simulation Subprocesses 
// -----------------------------------------------------------------------------

void FFullSys::ClearForces()
{
	for (FBody& Body : Bodies)
	{
		Body.NetForce = FVector2D::ZeroVector;
	}
}

void FFullSys::ApplyGravity()
{
	const FVector2D Gravity(0.0, PhysicsConsts::Gravity);

	for (FBody& Body : Bodies)
	{
		if (Body.Mass <= UE_SMALL_NUMBER)
		{
			continue;
		}

		Body.NetForce += Body.Mass * -Gravity;
	}
}

void FFullSys::ApplyElementInteractions()
{
	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		Element->ApplyForces(Bodies);
	}
}

void FFullSys::ComputeAccelerations()
{
	for (FBody& Body : Bodies)
	{
		Body.Acceleration = FVector2D::ZeroVector;

		if (Body.Mass > UE_SMALL_NUMBER)
		{
			Body.Acceleration = Body.NetForce / Body.Mass;
		}

		if (Body.bXFixed)
		{
			Body.Acceleration.X = 0.0;
		}

		if (Body.bYFixed)
		{
			Body.Acceleration.Y = 0.0;
		}
	}
}

void FFullSys::Integrate(double Dt)
{
	if (!Integrator)
	{
		return;
	}

	for (FBody& Body : Bodies)
	{
		if (!Body.bXFixed)
		{
			Integrator->IntegrateScalar(
				Body.Position.X,
				Body.Velocity.X,
				Body.Acceleration.X,
				Dt
			);
		}

		if (!Body.bYFixed)
		{
			Integrator->IntegrateScalar(
				Body.Position.Y,
				Body.Velocity.Y,
				Body.Acceleration.Y,
				Dt
			);
		}
	}
}

void FFullSys::ApplyFixedAxes()		// Faltan reacciones 
{
	for (FBody& Body : Bodies)
	{
		if (Body.bXFixed)
		{
			Body.Velocity.X = 0.0;
			Body.Acceleration.X = 0.0;
			Body.NetForce.X = 0.0;
		}

		if (Body.bYFixed)
		{
			Body.Velocity.Y = 0.0;
			Body.Acceleration.Y = 0.0;
			Body.NetForce.Y = 0.0;
		}
	}
}

void FFullSys::ProjectConstraintVelocities()
{
	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		if (!Element)
		{
			continue;
		}

		Element->ProjectVelocities(Bodies);
	}
}

void FFullSys::ProjectConstraintPositions()
{
	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		if (!Element)
		{
			continue;
		}

		Element->ProjectPositions(Bodies);
	}
}