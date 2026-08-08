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
	RodSys.Assemble(
		Elements
	);
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
// Simulation pipeline
// -----------------------------------------------------------------------------

void FFullSys::Step(double Dt)
{
	ClearForces();
	ApplyGravity();
	ApplyNonConstraintInteractions();

	// -------------------------------------------------------------------------
	// Constraint forces
	// -------------------------------------------------------------------------

	RodSys.UpdateConstraintForces(
		Bodies
	);

	const bool bRodSystemSolved =
		RodSys.SolveConstraintForces();

	if (!bRodSystemSolved)
	{
		return;
	}

	RodSys.ApplyRodConstraintForces(
		Bodies
	);

	// -------------------------------------------------------------------------
	// Integration
	// -------------------------------------------------------------------------

	ComputeAccelerations();
	Integrate(Dt);

	ApplyFixedAxes();

	// -------------------------------------------------------------------------
	// Velocity correction
	// -------------------------------------------------------------------------

	RodSys.UpdateVelocityCorrection(
		Bodies
	);

	const bool bVelocityCorrectionSolved =
		RodSys.SolveVelocityCorrection();

	if (!bVelocityCorrectionSolved)
	{
		return;
	}

	RodSys.ApplyVelocityCorrection(
		Bodies
	);

	// -------------------------------------------------------------------------
	// Diagnostics
	// -------------------------------------------------------------------------

	RodSys.UpdateStatisticalErrorData();
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