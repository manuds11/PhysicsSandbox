#include "Simulation/FullSys.h"

FFullSys::FFullSys()
{
	Integrator = MakeUnique<FExplicitEulerSysIntegrator>();
}

int32 FFullSys::AddBody(const FBody& Body)
{
	return Bodies.Add(Body);
}

void FFullSys::AddElement(TUniquePtr<ISysElement> Element)
{
	if (Element)
	{
		Elements.Add(MoveTemp(Element));
	}
}

void FFullSys::SetIntegrator(TUniquePtr<ISysIntegrator> InIntegrator)
{
	if (InIntegrator)
	{
		Integrator = MoveTemp(InIntegrator);
	}
}

void FFullSys::Step(double Dt)
{
	ClearForces();
	ApplyElementInteractions();
	ComputeAccelerations();
	Integrate(Dt);
	ApplyFixedAxes();
}

const TArray<FBody>& FFullSys::GetBodies() const
{
	return Bodies;
}

void FFullSys::ClearForces()
{
	for (FBody& Body : Bodies)
	{
		Body.NetForce = FVector2D::ZeroVector;
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