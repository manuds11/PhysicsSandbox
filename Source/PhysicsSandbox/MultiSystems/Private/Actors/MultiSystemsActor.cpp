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

	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		EnableInput(PC);
	}

	if (InputComponent)
	{
		InputComponent->BindKey(
			EKeys::SpaceBar,
			IE_Pressed,
			this,
			&AMultiSystemsActor::ToggleSimulation
		);
	}
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
	// -------------------------------------------------------------------------
	// Subsystem
	// -------------------------------------------------------------------------

	FSubSys& DoubleSpringDoublePendulum =
		FullSys.CreateSubSys(
			TEXT("DoubleSpringDoublePendulum")
		);

	// -------------------------------------------------------------------------
	// Bodies
	// -------------------------------------------------------------------------

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
				2.0
			)
		);

	const int32 Mass2Index =
		FullSys.AddBody(
			FBody::SlidingMassX(
				FVector2D(3.5, 0.0),
				1.0
			)
		);

	const int32 Bob1Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(4.5, 0.0),
				0.5
			)
		);

	const int32 Bob2Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(4.5, -1.0),
				1.0
			)
		);

	// -------------------------------------------------------------------------
	// Elements
	// -------------------------------------------------------------------------

	const int32 Spring1Index =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				WallIndex,
				Mass1Index,
				20.0,
				0.2,
				1.5
			)
		);

	const int32 Spring2Index =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				Mass1Index,
				Mass2Index,
				20.0,
				0.2,
				1.5
			)
		);

	const int32 PendulumRod1Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Mass2Index,
				Bob1Index
			)
		);

	const int32 PendulumRod2Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Bob1Index,
				Bob2Index
			)
		);

	// -------------------------------------------------------------------------
	// Subsystem associations
	// -------------------------------------------------------------------------

	DoubleSpringDoublePendulum.AddBody(
		TEXT("Wall"),
		WallIndex
	);

	DoubleSpringDoublePendulum.AddBody(
		TEXT("Mass1"),
		Mass1Index
	);

	DoubleSpringDoublePendulum.AddBody(
		TEXT("Mass2"),
		Mass2Index
	);

	DoubleSpringDoublePendulum.AddBody(
		TEXT("Bob1"),
		Bob1Index
	);

	DoubleSpringDoublePendulum.AddBody(
		TEXT("Bob2"),
		Bob2Index
	);

	DoubleSpringDoublePendulum.AddElement(
		TEXT("Spring1"),
		Spring1Index
	);

	DoubleSpringDoublePendulum.AddElement(
		TEXT("Spring2"),
		Spring2Index
	);

	DoubleSpringDoublePendulum.AddElement(
		TEXT("PendulumRod1"),
		PendulumRod1Index
	);

	DoubleSpringDoublePendulum.AddElement(
		TEXT("PendulumRod2"),
		PendulumRod2Index
	);

	DoubleSpringDoublePendulum.AddPort(
		TEXT("End"),
		Mass2Index
	);

	// Must be called after all bodies and elements have been added.
	FullSys.Initialize();
}

// -----------------------------------------------------------------------------
// Fixed step simulation
// -----------------------------------------------------------------------------

void AMultiSystemsActor::RunSimFixedSteps(double FrameDeltaTime)
{
	if (FixedTimeStep <= 0.0 || FrameDeltaTime <= 0.0)
	{
		return;
	}

	SimuWallClockTime += FrameDeltaTime;
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
		++FixedStepCount;

		SimuPhysicalTime += FixedTimeStep;
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

	SimulationDelay = SimuWallClockTime - SimuPhysicalTime;
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
	const TArray<FBody>& Bodies =
		FullSys.GetBodies();

	for (const FBody& Body : Bodies)
	{
		const double DebugRadius =
			ComputeBodyDebugRadius(
				Body.Mass
			);

		DrawDebugSphere(
			GetWorld(),
			ToWorld(Body.Position),
			DebugRadius,
			16,
			Body.bXFixed && Body.bYFixed
			? FColor::Red
			: FColor::Green,
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

		int32 BodyAIndex =
			INDEX_NONE;

		int32 BodyBIndex =
			INDEX_NONE;

		if (
			!Element->GetConnectedBodies(
				BodyAIndex,
				BodyBIndex
			)
			)
		{
			continue;
		}

		if (
			!Bodies.IsValidIndex(BodyAIndex)
			|| !Bodies.IsValidIndex(BodyBIndex)
			)
		{
			continue;
		}

		DrawDebugLine(
			GetWorld(),
			ToWorld(
				Bodies[BodyAIndex].Position
			),
			ToWorld(
				Bodies[BodyBIndex].Position
			),
			FColor::Yellow,
			false,
			0.0f,
			0,
			2.0f
		);

		FVector2D EquilibriumPoint;

		if (
			Element->GetEquilibriumPoint(
				Bodies,
				EquilibriumPoint
			)
			)
		{
			DrawDebugLine(
				GetWorld(),
				ToWorld(EquilibriumPoint)
				+ FVector(
					0.0,
					0.0,
					-50.0
				),
				ToWorld(EquilibriumPoint)
				+ FVector(
					0.0,
					0.0,
					50.0
				),
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

	FString DebugText;

	DebugText += TEXT("MULTI SYSTEMS\n");

	DebugText += FString::Printf(
		TEXT(
			"SPACE Start/Stop\n"
			"\n"
			"SimulTime: %.3f s | FixedSteps: %llu\n"
			"Delay: %.5f s | AvgDt: %.5f s | FixedDt: %.6f s\n\n"
		),
		SimuPhysicalTime, FixedStepCount,
		SimulationDelay, AverageDeltaTime, FixedTimeStep
	);

	const TArray<FPendulumRod*>& RodSystemArray =
		FullSys.GetRodSystemArray();

	const TArray<double>& RodSystemLambda =
		FullSys.GetRodSystemLambdaVector();

	check(
		RodSystemArray.Num()
		== RodSystemLambda.Num()
	);

	for (
		int32 Rod_iIndex = 0;
		Rod_iIndex < RodSystemArray.Num();
		++Rod_iIndex
		)
	{
		const FPendulumRod* Rod_i =
			RodSystemArray[Rod_iIndex];

		if (!Rod_i)
		{
			continue;
		}

		const FPendulumPositions& PendulumPos_i =
			Rod_i->GetPendulumPositions();

		const FPendulumVelocities& PendulumVel_i =
			Rod_i->GetPendulumVelocities();

		const FInstantErrorData& InstantErrorData_i =
			Rod_i->GetInstantErrorData();

		const FStatisticalErrorData& StatisticalErrorData_i =
			Rod_i->GetStatisticalErrorData();

		const double Lambda_i =
			RodSystemLambda[Rod_iIndex];

		DebugText += FString::Printf(
			TEXT(
				"--------------------------------------------------\n"
				"ROD %d\n\n"

				"L computed: %.10f m  ||  L Abs error: %.4f m  ||  L Rel error: %.10f %%\n\n"

				"LastStep L Increment: %.10f microm/step  ||  Mean Step L Increment: %.10f microm/step\n\n"

				"Radial Vel: %.12f microm/s  ||  Mean Radial Vel: %.12f microm/s\n\n"

				"Lambda: %.4f N\n\n"
			),

			Rod_iIndex + 1,

			PendulumPos_i.ComputedLength,
			InstantErrorData_i.LengthAbsError,
			100.0 * InstantErrorData_i.LengthRelError,

			1.0e6 * StatisticalErrorData_i.StepLengthIncrement,
			1.0e6 * StatisticalErrorData_i.MeanStepLengthIncrement,

			1.0e6 * PendulumVel_i.Bob2PivotRadial,
			1.0e6 * StatisticalErrorData_i.MeanRadialVelocity,

			Lambda_i
		);
	}

	const FColor TextColor =
		bIsSimulationRunning
		? FColor::Green
		: FColor::Red;

	GEngine->AddOnScreenDebugMessage(
		0,
		0.0f,
		TextColor,
		DebugText
	);
}

// -----------------------------------------------------------------------------
// Debug Helpers
// -----------------------------------------------------------------------------

double AMultiSystemsActor::ComputeBodyDebugRadius(
	double Mass
) const
{
	constexpr double ReferenceMass = 1.0;
	constexpr double ReferenceRadius = 8.0;

	if (Mass <= UE_SMALL_NUMBER)
	{
		return ReferenceRadius;
	}

	const double ComputedRadius =
		ReferenceRadius
		* FMath::Pow(
			Mass / ReferenceMass,
			1.0 / 3.0
		);

	return FMath::Clamp(
		ComputedRadius,
		4.0,
		20.0
	);
}