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

	IntegrationEnergy = {};
	RodSys.ResetEnergyDiagnostics();

	RodSys.BuildEmptyStructure(Elements);

	EnergyBalance.Initial = ComputeSystemEnergy();
	EnergyBalance.Current = EnergyBalance.Initial;

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

int32 FFullSys::AddForceInput(const FForceInput& ForceInput)
{
	check(!bInitialized);

	check(Bodies.IsValidIndex(ForceInput.BodyIndex));

	check(
		FMath::IsFinite(ForceInput.Force.X)
		&& FMath::IsFinite(ForceInput.Force.Y)
	);

	return ForceInputs.Add(ForceInput);
}

void FFullSys::SetForceInput(
	int32 ForceInputIndex,
	const FVector2D& Force
)
{
	checkf(
		ForceInputs.IsValidIndex(ForceInputIndex),
		TEXT("Invalid ForceInputIndex %d."),
		ForceInputIndex
	);
	check(
		FMath::IsFinite(Force.X)
		&& FMath::IsFinite(Force.Y)
	);

	ForceInputs[ForceInputIndex].Force = Force;
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

	Eigen::VectorXd Q_Full(NumCoordinates);
	Eigen::VectorXd QDot_Full(NumCoordinates);

	for (int32 BodyIndex = 0; BodyIndex < Bodies.Num(); ++BodyIndex)
	{
		const FBody& Body = Bodies[BodyIndex];

		const int32 XIndex = 2 * BodyIndex;
		const int32 YIndex = XIndex + 1;

		Q_Full(XIndex) = Body.Position.X;
		Q_Full(YIndex) = Body.Position.Y;

		QDot_Full(XIndex) = Body.Velocity.X;
		QDot_Full(YIndex) = Body.Velocity.Y;
	}

	return FSysState(
		MoveTemp(Q_Full),
		MoveTemp(QDot_Full)
	);
}

FSysStateDerivative FFullSys::BuildFullStateDerivative(
	const Eigen::VectorXd& InQDot_Full
) const
{
	const int32 NumCoordinates = 2 * Bodies.Num();

	check(InQDot_Full.size() == NumCoordinates);

	Eigen::VectorXd QDotDot_Full(NumCoordinates);

	for (int32 BodyIndex = 0; BodyIndex < Bodies.Num(); ++BodyIndex)
	{
		const FBody& Body = Bodies[BodyIndex];

		const int32 XIndex = 2 * BodyIndex;
		const int32 YIndex = XIndex + 1;

		QDotDot_Full(XIndex) = Body.Acceleration.X;
		QDotDot_Full(YIndex) = Body.Acceleration.Y;
	}

	return FSysStateDerivative(
		InQDot_Full,
		MoveTemp(QDotDot_Full)
	);
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
	ApplyForceInputs();
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

	const double DissipatedPowerBefore =
		ComputeDissipatedPower();

	if (!EvaluateDynamics())
	{
		return false;
	}

	IntegrationEnergy.Before =
		ComputeSystemEnergy();

	Integrate(Dt);

	ApplyFixedAxes();

	IntegrationEnergy.After =
		ComputeSystemEnergy();

	if (!ensureMsgf(
		IsStateFinite(),
		TEXT(
			"Simulation state became non-finite after constraint correction."
		)
	))
	{
		return false;
	}

	if (!RodSys.RunConstraintCorrections(
		Bodies,
		[this]()
		{
			return ComputeSystemEnergy();
		}
	))
	{
		return false;
	}

	if (!ensureMsgf(
		IsStateFinite(),
		TEXT(
			"Simulation state became non-finite after constraint correction."
		)
	))
	{
		return false;
	}

	RodSys.UpdateStatisticalErrorData();

	UpdateEnergyBalance(
		Dt,
		DissipatedPowerBefore
	);

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

void FFullSys::ApplyForceInputs()
{
	for (const FForceInput& Input : ForceInputs)
	{
		check(Bodies.IsValidIndex(Input.BodyIndex));

		check(
			FMath::IsFinite(Input.Force.X)
			&& FMath::IsFinite(Input.Force.Y)
		);

		Bodies[Input.BodyIndex].NetForce += Input.Force;
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

// -----------------------------------------------------------------------------º
// Energy diagnostics
// -----------------------------------------------------------------------------

FSystemEnergyState FFullSys::ComputeSystemEnergy() const
{
	FSystemEnergyState Energy;

	// -------------------------------------------------------------------------
	// Bodies: kinetic and gravitational energy
	// -------------------------------------------------------------------------

	for (const FBody& Body : Bodies)
	{
		const double Vx =
			Body.bXFixed ? 0.0 : Body.Velocity.X;

		const double Vy =
			Body.bYFixed ? 0.0 : Body.Velocity.Y;

		Energy.Kinetic +=
			0.5 * Body.Mass * (Vx * Vx + Vy * Vy);

		// Gravity force currently applied as:
		// F_gravity = -Mass * (0, PhysicsConsts::Gravity)
		//
		// Therefore V_gravity = Mass * PhysicsConsts::Gravity * Y

		Energy.Gravitational +=
			Body.Mass
			* PhysicsConsts::Gravity
			* Body.Position.Y;
	}

	// -------------------------------------------------------------------------
	// Elements: potential energy
	// -------------------------------------------------------------------------

	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		if (Element)
		{
			Energy.Elastic += 
				Element->ComputePotentialEnergy(Bodies);
		}
	}

	return Energy;
}


double FFullSys::ComputeDissipatedPower() const
{
	double TotalPower = 0.0;

	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		if (Element)
		{
			TotalPower += Element->ComputeDissipatedPower(Bodies);
		}
	}

	return TotalPower;
}

void FFullSys::UpdateEnergyBalance(double Dt, double DissipatedPowerBefore)
{
	const double DissipatedPowerAfter = ComputeDissipatedPower();

	const double DissipatedIncrement =
		0.5 * Dt * (DissipatedPowerBefore + DissipatedPowerAfter);

	const double PreviousDissipated = EnergyBalance.Current.TotalDissipated;

	EnergyBalance.Current = ComputeSystemEnergy();

	EnergyBalance.Current.TotalDissipated =
		PreviousDissipated + DissipatedIncrement;
}