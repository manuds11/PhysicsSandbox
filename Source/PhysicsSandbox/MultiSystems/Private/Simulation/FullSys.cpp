#include "Simulation/FullSys.h"
#include "Math/Units.h"
#include "Elements/PendulumRod.h"
#include "Math/DenseLinearSolver.h"

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
	check(!bInitialized);

	RodSys.BuildEmptyStructure(
		Elements
	);

	bInitialized = true;
}

int32 FFullSys::AddBody(const FBody& Body)
{
	check(!bInitialized);

	checkf(
		FMath::IsFinite(Body.Mass)
		&& Body.Mass > UE_SMALL_NUMBER,
		TEXT("FBody mass must be finite and positive.")
	);

	checkf(
		FMath::IsFinite(Body.Position.X)
		&& FMath::IsFinite(Body.Position.Y)
		&& FMath::IsFinite(Body.Velocity.X)
		&& FMath::IsFinite(Body.Velocity.Y),
		TEXT("FBody initial state must be finite.")
	);

	return Bodies.Add(Body);
}

int32 FFullSys::AddElement(TUniquePtr<ISysElement> Element)
{
	check(!bInitialized);

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
	check(InIntegrator);

	Integrator = MoveTemp(InIntegrator);
}

FSysState FFullSys::BuildFullState() const
{
	const int32 NumCoordinates = 2 * Bodies.Num();

	FSysState State;

	State.Q_Full.resize(NumCoordinates);
	State.QDot_Full.resize(NumCoordinates);

	for (int32 BodyIndex = 0; BodyIndex < Bodies.Num(); ++BodyIndex)
	{
		const FBody& Body = Bodies[BodyIndex];

		const int32 XIndex = 2 * BodyIndex;
		const int32 YIndex = XIndex + 1;

		State.Q_Full(XIndex) = Body.Position.X;
		State.Q_Full(YIndex) = Body.Position.Y;

		State.QDot_Full(XIndex) = Body.Velocity.X;
		State.QDot_Full(YIndex) = Body.Velocity.Y;
	}

	return State;
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
// Simulation pipeline
// -----------------------------------------------------------------------------

bool FFullSys::EvaluateDynamics()
{
	ClearForces();
	ApplyGravity();
	ApplyNonConstraintInteractions();

	if (!RodSys.RunConstraintForces(Bodies))
	{
		return false;
	}

	ComputeAccelerations();

	return true;
}

bool FFullSys::Step(double Dt)
{
	check(bInitialized);

	checkf(
		FMath::IsFinite(Dt) && Dt > 0.0,
		TEXT("Simulation Dt must be finite and positive.")
	);

	if (!EvaluateDynamics())
	{
		return false;
	}

	Integrate(Dt);

	ApplyFixedAxes();

	if (!ensureMsgf(
		IsStateFinite(),
		TEXT("Simulation state became non-finite after integration.")
	))
	{
		return false;
	}

	if (!RodSys.RunConstraintCorrections(Bodies))
	{
		return false;
	}

	if (!ensureMsgf(
		IsStateFinite(),
		TEXT("Simulation state became non-finite after constraint correction.")
	))
	{
		return false;
	}

	RodSys.UpdateStatisticalErrorData();

	return true;
}

// -----------------------------------------------------------------------------º
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

void FFullSys::ApplyNonConstraintInteractions()
{
	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		if (!Element)
		{
			continue;
		}

		if (
			Element->GetElementType()
			== ESysElementType::PendulumRod
			)
		{
			continue;
		}

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
	check(Integrator);

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

// -----------------------------------------------------------------------------º
// Debug helpers
// -----------------------------------------------------------------------------

bool FFullSys::IsStateFinite() const
{
	for (const FBody& Body : Bodies)
	{
		const bool bPositionFinite =
			FMath::IsFinite(Body.Position.X)
			&& FMath::IsFinite(Body.Position.Y);

		const bool bVelocityFinite =
			FMath::IsFinite(Body.Velocity.X)
			&& FMath::IsFinite(Body.Velocity.Y);

		const bool bAccelerationFinite =
			FMath::IsFinite(Body.Acceleration.X)
			&& FMath::IsFinite(Body.Acceleration.Y);

		const bool bForceFinite =
			FMath::IsFinite(Body.NetForce.X)
			&& FMath::IsFinite(Body.NetForce.Y);

		if (
			!bPositionFinite
			|| !bVelocityFinite
			|| !bAccelerationFinite
			|| !bForceFinite
			)
		{
			return false;
		}
	}

	return true;
}