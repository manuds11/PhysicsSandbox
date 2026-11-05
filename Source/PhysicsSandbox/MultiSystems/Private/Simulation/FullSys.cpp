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
	RodSys.InitializeRodStates(Bodies);

	BuildStateIndexMap();

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
// State-space adaptators
// -----------------------------------------------------------------------------

void FFullSys::BuildStateIndexMap()
{
	StateIndexMap = {};

	for (int32 BodyIndex = 0; BodyIndex < Bodies.Num(); ++BodyIndex)
	{
		const FBody& Body = Bodies[BodyIndex];

		const int32 XIndex = 2 * BodyIndex;
		const int32 YIndex = XIndex + 1;

		if (Body.bXFixed)
		{
			StateIndexMap.QFull_FixedIndices.Add(XIndex);
		}
		else
		{
			StateIndexMap.Q_FRToFullIndices.Add(XIndex);
		}

		if (Body.bYFixed)
		{
			StateIndexMap.QFull_FixedIndices.Add(YIndex);
		}
		else
		{
			StateIndexMap.Q_FRToFullIndices.Add(YIndex);
		}
	}

	StateIndexMap.SizeQ_FR = StateIndexMap.Q_FRToFullIndices.Num();
	StateIndexMap.SizeQ_Full = 2 * Bodies.Num();

	const int32 SizeQ_Full = StateIndexMap.SizeQ_Full;

	// X = [Q ; QDot]
	for (const int32 QIndex : StateIndexMap.Q_FRToFullIndices)
	{
		StateIndexMap.X_FRToFullIndices.Add(QIndex);
	}

	for (const int32 QIndex : StateIndexMap.Q_FRToFullIndices)
	{
		StateIndexMap.X_FRToFullIndices.Add(SizeQ_Full + QIndex);
	}

	for (const int32 QIndex : StateIndexMap.QFull_FixedIndices)
	{
		StateIndexMap.X_FullFixedIndices.Add(QIndex);
	}

	for (const int32 QIndex : StateIndexMap.QFull_FixedIndices)
	{
		StateIndexMap.X_FullFixedIndices.Add(SizeQ_Full + QIndex);
	}

	check(
		StateIndexMap.Q_FRToFullIndices.Num()
		+ StateIndexMap.QFull_FixedIndices.Num()
		== SizeQ_Full
	);
}

FSysState FFullSys::BodiesToFullStateStruct() const
{
	// Builds an independent snapshot of the current full system state from Bodies, copying its values.

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
	// Builds an independent snapshot of the full state derivative.
	// QDot_Full is provided explicitly to keep it consistent with its associated state.

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

void FFullSys::SetXState2Bodies(
	const FSysState& InputX_Full
)
{
	const int32 NumCoordinates = 2 * Bodies.Num();

	check(InputX_Full.Q.size() == NumCoordinates);
	check(InputX_Full.QDot.size() == NumCoordinates);

	for (int32 BodyIndex = 0; BodyIndex < Bodies.Num(); ++BodyIndex)
	{
		FBody& Body = Bodies[BodyIndex];

		const int32 XIndex = 2 * BodyIndex;
		const int32 YIndex = XIndex + 1;

		Body.Position.X = InputX_Full.Q(XIndex);
		Body.Position.Y = InputX_Full.Q(YIndex);

		Body.Velocity.X = InputX_Full.QDot(XIndex);
		Body.Velocity.Y = InputX_Full.QDot(YIndex);
	}
}

void FFullSys::SyncRodStates()
{
	RodSys.InitializeRodStates(Bodies);
}

// -----------------------------------------------------------------------------
// Simulation pipeline
// -----------------------------------------------------------------------------

bool FFullSys::EvaluateXDot(
	const FSysState& InputX_Full,
	const Eigen::VectorXd& Tau_FR,
	const TArray<FU_ExtForce>& U_ExtForces,
	FSysStateDerivative& OutputXDot_Full
)
{
	check(bInitialized);
	check(Tau_FR.size() == StateIndexMap.SizeQ_FR);
	check(Tau_FR.allFinite());

	SetXState2Bodies(InputX_Full);

	ClearForces();
	ApplyGravity();
	ApplyU_ExtForces(U_ExtForces);
	ApplyTau_FR(Tau_FR);
	ApplyNonConstraintInteractions();

	if (!RodSys.RunConstraintForces(Bodies))
	{
		return false;
	}

	ComputeAccelerations();

	OutputXDot_Full = BuildFullStateDerivative(InputX_Full.QDot);

	return true;
}

bool FFullSys::EvaluateNextXState(
	double Dt,
	const FSysState& InputX_Full,
	const Eigen::VectorXd& Tau_FR,
	const TArray<FU_ExtForce>& U_ExtForces,
	FSysState& OutputXNext_Full,
	bool bRecordEnergyDiagnostics
)
{
	check(bInitialized);
	checkf(FMath::IsFinite(Dt) && Dt > 0.0, TEXT("Simulation Dt must be finite and positive."));

	FSysStateDerivative XDot_Full;

	if (!EvaluateXDot(
		InputX_Full,
		Tau_FR,
		U_ExtForces,
		XDot_Full
	))
	{
		return false;
	}

	if (bRecordEnergyDiagnostics)
	{
		IntegrationEnergy.Before = ComputeSystemEnergy();
	}

	Integrate(Dt);

	ApplyFixedAxes();

	if (bRecordEnergyDiagnostics)
	{
		IntegrationEnergy.After = ComputeSystemEnergy();
	}

	if (!ensureMsgf(
		IsStateFinite(),
		TEXT("Simulation state became non-finite after integration.")
	))
	{
		return false;
	}

	if (
		!RodSys.RunConstraintCorrections(Bodies, 
		[this](){	return ComputeSystemEnergy();	},
		bRecordEnergyDiagnostics
		)
	)
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

	OutputXNext_Full = BodiesToFullStateStruct();

	return true;
}

bool FFullSys::Step(
	double Dt,
	const FSysState& InputX_Full,
	const Eigen::VectorXd& Tau_FR,
	const TArray<FU_ExtForce>& U_ExtForces,
	FSysState& OutputXNext_Full
)
{
	check(bInitialized);
	checkf(FMath::IsFinite(Dt) && Dt > 0.0, TEXT("Simulation Dt must be finite and positive."));

	SetXState2Bodies(InputX_Full);

	const double DissipatedPowerBefore = ComputeDissipatedPower();

	const bool bRecordEnergyDiagnostics = true;

	if (!EvaluateNextXState(
		Dt,
		InputX_Full,
		Tau_FR,
		U_ExtForces,
		OutputXNext_Full,
		bRecordEnergyDiagnostics
	))
	{
		return false;
	}

	RodSys.UpdateStatisticalErrorData();

	UpdateEnergyBalance(Dt, DissipatedPowerBefore);

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

void FFullSys::ApplyU_ExtForces(const TArray<FU_ExtForce>& U_ExtForces)
{
	for (const FU_ExtForce& U_ExtForce : U_ExtForces)
	{
		check(Bodies.IsValidIndex(U_ExtForce.BodyApplicationIndex));
		check(FMath::IsFinite(U_ExtForce.Vector.X) && FMath::IsFinite(U_ExtForce.Vector.Y));

		Bodies[U_ExtForce.BodyApplicationIndex].NetForce += U_ExtForce.Vector;
	}
}

void FFullSys::ApplyTau_FR(const Eigen::VectorXd& Tau_FR)
{
	check(Tau_FR.size() == StateIndexMap.SizeQ_FR);

	for (int32 Q_FRIndex = 0; Q_FRIndex < StateIndexMap.SizeQ_FR; ++Q_FRIndex)
	{
		const int32 Q_FullIndex = StateIndexMap.Q_FRToFullIndices[Q_FRIndex];
		const int32 BodyIndex = Q_FullIndex / 2;

		if (Q_FullIndex % 2 == 0)
		{
			Bodies[BodyIndex].NetForce.X += Tau_FR(Q_FRIndex);
		}
		else
		{
			Bodies[BodyIndex].NetForce.Y += Tau_FR(Q_FRIndex);
		}
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