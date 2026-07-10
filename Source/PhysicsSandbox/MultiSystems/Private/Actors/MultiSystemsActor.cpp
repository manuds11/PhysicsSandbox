// Fill out your copyright notice in the Description page of Project Settings.

#include "Actors/MultiSystemsActor.h"
#include "Math/Units.h"
#include "Elements/PendulumRod.h"

#include "Elements/SpringDamper.h"
#include "DrawDebugHelpers.h"

// -----------------------------------------------------------------------------
// Construction and lifecycle
// -----------------------------------------------------------------------------

AMultiSystemsActor::AMultiSystemsActor()
{
 	// Set this actor to call Tick() every frame. You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;	
}

void AMultiSystemsActor::BeginPlay()
{
	Super::BeginPlay();
	
	BuildDemoSystem();

	ConfigureIntegrator();
}

void AMultiSystemsActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (bIsSimulationRunning)
	{
		RunSimFixedSteps(DeltaTime);
	}

	DrawSystem();
	PrintInfo();
}

// -----------------------------------------------------------------------------
// Input
// -----------------------------------------------------------------------------

void AMultiSystemsActor::ToggleSimulation()
{
	bIsSimulationRunning = !bIsSimulationRunning;

	/*if (bEnableCsvLogging)
	{
		if (bIsSimulationRunning)
		{
			CsvLogger.Reset();

			LogCurrentSample();
		}
		else
		{
			FlushCsvLog();
		}
	}*/
}

void AMultiSystemsActor::ConfigureIntegrator()
{
	switch (IntegratorType)
	{
	case EIntegratorType::ExplicitEuler:
		FullSys.SetIntegrator(
			MakeUnique<FExplicitEulerSysIntegrator>()
		);
		break;

	case EIntegratorType::SemiImplicitEuler:
		FullSys.SetIntegrator(
			MakeUnique<FSemiImplicitEulerSysIntegrator>()
		);
		break;

	default:
		checkNoEntry();
		break;
	}
}

// -----------------------------------------------------------------------------
// System setup
// -----------------------------------------------------------------------------

void AMultiSystemsActor::BuildDemoSystem()
{
	// Subsystem 1: Two masses connected by two springs to a fixed wall. Subystem is declared.
	FSubSys& DoubleSpring =
		FullSys.CreateSubSys(TEXT("DoubleSpring"));

	// Declare the bodies of the subsystem with required parameters to define them and
	// store them in an array in FullSys Class. The body index in the array identifies
	// each body and is saved. The bodies index allow us define interactions between
	// bodies.
	const int32 WallIndex = 
		FullSys.AddBody(
			FBody::Fixed( 
				FVector2D(0.0, 0.0)
			)
		);

	const int32 Mass1Index =
		FullSys.AddBody(
			FBody::SlidingMassX(
				FVector2D(2.0, 0.0),
				1.0						// mass kg
			)
		);

	const int32 Mass2Index =
		FullSys.AddBody(
			FBody::SlidingMassX(
				FVector2D(3.5, 0.0),
				1.0
			)
		);

	const int32 BobIndex =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(4.5, 0.0),
				1.0
			)
		);

	// Declare elements with its connections and parameters and store them in an
	// array in FullSys Class. The element index in the array identifies each element 
	// and is saved. The element index allow us define interactions between bodies.

	const int32 Spring1Index =
		FullSys.AddElement(MakeUnique<FSpringDamper>(
			WallIndex,
			Mass1Index,
			20.0,
			0.5,
			1.5
		));

	const int32 Spring2Index =
		FullSys.AddElement(MakeUnique<FSpringDamper>(
			Mass1Index,
			Mass2Index,
			20.0,
			0.5,
			1.5
		));

	const int32 PendulumRodIndex =
		FullSys.AddElement(MakeUnique<FPendulumRod>(
			Mass2Index,
			BobIndex
		));

	PendulumIndexForDebug = PendulumRodIndex;
	
	// We associate the bodies and elements to the subsystem by their indexes. 
	// We attach an identification tag. 
	DoubleSpring.AddBody(TEXT("Wall"), WallIndex);
	DoubleSpring.AddBody(TEXT("Mass1"), Mass1Index);
	DoubleSpring.AddBody(TEXT("Mass2"), Mass2Index);
	DoubleSpring.AddBody(TEXT("Bob"), BobIndex);

	DoubleSpring.AddElement(TEXT("Spring1"), Spring1Index);
	DoubleSpring.AddElement(TEXT("Spring2"), Spring2Index);
	DoubleSpring.AddElement(TEXT("PendulumRod"), PendulumRodIndex);

	DoubleSpring.AddPort(TEXT("End"), Mass2Index);
}

// -----------------------------------------------------------------------------
// Simulation
// -----------------------------------------------------------------------------

void AMultiSystemsActor::RunSimFixedSteps(double FrameDeltaTime)
{
	if (FixedTimeStep <= 0.0 || FrameDeltaTime <= 0.0)
	{
		return;
	}

	RealRunningTime += FrameDeltaTime;
	FrameCount++;

	AverageDeltaTime +=
		(FrameDeltaTime - AverageDeltaTime)
		/ static_cast<double>(FrameCount);

	const double ClampedFrameDeltaTime =
		FMath::Min(FrameDeltaTime, MaxFrameDeltaTime);

	SimulationTimeDebt += ClampedFrameDeltaTime;

	int32 SubStepCount = 0;

	while (
		SimulationTimeDebt >= FixedTimeStep
		&& SubStepCount < MaxSubSteps)
	{
		FullSys.Step(FixedTimeStep);

		SimulatedRunningTime += FixedTimeStep;
		SimulationTimeDebt -= FixedTimeStep;

		SubStepCount++;

		/*if (benablecsvlogging)
		{
			logcurrentsample();
		}*/
	}

	LastSubStepCount = SubStepCount;

	if (SubStepCount >= MaxSubSteps)
	{
		SimulationTimeDebt = FMath::Min(SimulationTimeDebt, FixedTimeStep);
	}

	SimulationDelay = RealRunningTime - SimulatedRunningTime;
}

// -----------------------------------------------------------------------------
// Visualization and debug
// -----------------------------------------------------------------------------

FVector AMultiSystemsActor::ToWorld(const FVector2D& P) const
{
	return GetActorLocation()
		+ VisualizationOffset
		+ FVector(
			Units::MToCm * P.X,
			0.0,
			Units::MToCm * P.Y
		);
}

void AMultiSystemsActor::DrawSystem() const
{
	const TArray<FBody>& Bodies = FullSys.GetBodies();

	for (const FBody& Body : Bodies)
	{
		DrawDebugSphere(
			GetWorld(),
			ToWorld(Body.Position),
			10.0f,
			16,
			Body.bXFixed && Body.bYFixed ? FColor::Red : FColor::Green,
			false,
			0.0f
		);
	}

	const TArray<TUniquePtr<ISysElement>>& Elements =
		FullSys.GetElements();

	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		if (!Element)
		{
			continue;
		}

		int32 BodyAIndex = INDEX_NONE;
		int32 BodyBIndex = INDEX_NONE;

		if (!Element->GetConnectedBodies(BodyAIndex, BodyBIndex))
		{
			continue;
		}

		if (!Bodies.IsValidIndex(BodyAIndex) || !Bodies.IsValidIndex(BodyBIndex))
		{
			continue;
		}

		DrawDebugLine(
			GetWorld(),
			ToWorld(Bodies[BodyAIndex].Position),
			ToWorld(Bodies[BodyBIndex].Position),
			FColor::Yellow,
			false,
			0.0f,
			0,
			2.0f
		);

		FVector2D EquilibriumPoint;

		if (Element->GetEquilibriumPoint(Bodies, EquilibriumPoint))
		{
			DrawDebugLine(
				GetWorld(),
				ToWorld(EquilibriumPoint) + FVector(0.0, 0.0, -50.0),
				ToWorld(EquilibriumPoint) + FVector(0.0, 0.0, 50.0),
				FColor::Blue,
				false,
				0.0f,
				0,
				2.0f
			);
		}
	}
}

void AMultiSystemsActor::PrintInfo() const
{
	if (!GEngine)
	{
		return;
	}

	const FColor TitleColor =
		bIsSimulationRunning
		? FColor::Green
		: FColor::Red;

	const FColor InfoColor =
		bIsSimulationRunning
		? FColor::Cyan
		: FColor::Silver;

	GEngine->AddOnScreenDebugMessage(
		0,
		0.0f,
		TitleColor,
		TEXT("MULTI SYSTEMS")
	);

	GEngine->AddOnScreenDebugMessage(
		1,
		0.0f,
		InfoColor,
		FString::Printf(
			TEXT(
				"SPACE Start/Stop | "
				"Real: %.2f s | "
				"Delay: %.5f s | "
				"AvgDt: %.5f s | "
				"FixedDt: %.5f s"
			),
			RealRunningTime,
			SimulationDelay,
			AverageDeltaTime,
			FixedTimeStep
		)
	);

	const TArray<TUniquePtr<ISysElement>>& Elements =
		FullSys.GetElements();

	if (!Elements.IsValidIndex(PendulumIndexForDebug))
	{
		return;
	}

	const FPendulumRod* PendulumRod =
		static_cast<const FPendulumRod*>(
			Elements[PendulumIndexForDebug].Get()
			);

	if (!PendulumRod)
	{
		return;
	}

	const FPendulumPositions& PendulumPos =
		PendulumRod->GetPendulumPositions();

	const FPendulumVelocities& PendulumVel =
		PendulumRod->GetPendulumVelocities();

	GEngine->AddOnScreenDebugMessage(
		2,
		0.0f,
		FColor::Yellow,
		FString::Printf(
			TEXT(
				"Pendulum | "
				"L: %.4f m | "
				"Err: %.4f %% | "
				"Vr: %.6f m/s | "
				"T: %.3f N"
			),
			PendulumPos.ComputedLength,
			100.0 * PendulumPos.LengthRelError,
			PendulumVel.Bob2PivotRadial,
			PendulumRod->GetLastComputedTension()
		)
	);
}