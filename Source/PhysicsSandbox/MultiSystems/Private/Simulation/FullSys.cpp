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
	if (RodSystemArray.IsEmpty())
	{
		return;
	}

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
	UpdateRodSystemMatrixA();

	// Próximo paso:
	// UpdateRodSystemVectorB();
}

void FFullSys::UpdateRodSystemMatrixA()
{
	const int32 NumRods =
		RodSystemArray.Num();

	for (int32 Row = 0; Row < NumRods; ++Row)
	{
		for (int32 Column = 0;
			Column < NumRods;
			++Column)
		{
			MatrixIndex2ArrayIndex(Row, Column) =
				ComputeA_ij(
					*RodSystemArray[Row],
					*RodSystemArray[Column]
				);
		}
	}
}

double FFullSys::ComputeA_ij(
	const FPendulumRod& Rod_i,
	const FPendulumRod& Rod_j
) const
{
	int32 BodyPivot_i = INDEX_NONE;
	int32 BodyBob_i = INDEX_NONE;

	int32 BodyPivot_j = INDEX_NONE;
	int32 BodyBob_j = INDEX_NONE;

	Rod_i.GetConnectedBodies(
		BodyPivot_i,
		BodyBob_i
	);

	Rod_j.GetConnectedBodies(
		BodyPivot_j,
		BodyBob_j
	);

	const FRodJacobian& Jacobian_i =
		Rod_i.GetRodJacobian();

	const FRodJacobian& Jacobian_j =
		Rod_j.GetRodJacobian();

	const FRodExtreme Pivot_i
	{
		BodyPivot_i,
		Jacobian_i.JPivot
	};

	const FRodExtreme Bob_i
	{
		BodyBob_i,
		Jacobian_i.JBob
	};

	const FRodExtreme Pivot_j
	{
		BodyPivot_j,
		Jacobian_j.JPivot
	};

	const FRodExtreme Bob_j
	{
		BodyBob_j,
		Jacobian_j.JBob
	};

	const auto ComputeA_ijComponent =
		[this](
			const FRodExtreme& Rod_iExtreme,
			const FRodExtreme& Rod_jExtreme
			) -> double
		{
			if (Rod_iExtreme.BodyIndex != Rod_jExtreme.BodyIndex)
			{
				return 0.0;
			}

			const int32 SharedBodyIndex =
				Rod_iExtreme.BodyIndex;

			if (!Bodies.IsValidIndex(SharedBodyIndex))
			{
				return 0.0;
			}

			const FBody& SharedBody =
				Bodies[SharedBodyIndex];

			if (SharedBody.Mass <= UE_SMALL_NUMBER)
			{
				return 0.0;
			}

			const double InverseMass =
				1.0 / SharedBody.Mass;

			double A_ijComponent = 0.0;

			if (!SharedBody.bXFixed)
			{
				A_ijComponent +=
					Rod_iExtreme.Jacobian.X *
					Rod_jExtreme.Jacobian.X *
					InverseMass;
			}

			if (!SharedBody.bYFixed)
			{
				A_ijComponent +=
					Rod_iExtreme.Jacobian.Y *
					Rod_jExtreme.Jacobian.Y *
					InverseMass;
			}

			return A_ijComponent;
		};

	/*
	 * A_ij = J_i M^-1 J_j^T
	 *
	 * Each rod contributes one Jacobian block for its pivot
	 * and another for its bob. Only blocks associated with
	 * the same body produce a non-zero component.
	 */
	return
		ComputeA_ijComponent(Pivot_i, Pivot_j) +
		ComputeA_ijComponent(Pivot_i, Bob_j) +
		ComputeA_ijComponent(Bob_i, Pivot_j) +
		ComputeA_ijComponent(Bob_i, Bob_j);
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

	return RodSysMatrixA[
		Row * NumRods + Column
	];
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