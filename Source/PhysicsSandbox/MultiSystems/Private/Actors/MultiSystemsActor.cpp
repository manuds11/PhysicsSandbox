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

	FSubSys& TripleInvertedPendulum =
		FullSys.CreateSubSys(
			TEXT("TripleInvertedPendulum")
		);

	// -------------------------------------------------------------------------
	// Bodies - Pendulum
	// -------------------------------------------------------------------------

	const int32 BaseIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(2.50, -1.20)
			)
		);

	const int32 Mass1Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(2.58, -0.40),
				1.4
			)
		);

	const int32 Mass2Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(2.38, 0.32),
				1.0
			)
		);

	const int32 Mass3Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(2.62, 1.20),
				0.7
			)
		);

	const int32 LeftAnchorIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(1.14, 1.20)
			)
		);

	const int32 RightAnchorIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(3.86, 1.20)
			)
		);
	
	// -------------------------------------------------------------------------
	// Rods
	// -------------------------------------------------------------------------

	const int32 Rod1Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BaseIndex,
				Mass1Index
			)
		);

	const int32 Rod2Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Mass1Index,
				Mass2Index
			)
		);

	const int32 Rod3Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Mass2Index,
				Mass3Index
			)
		);

	// -------------------------------------------------------------------------
	// Upper stabilization springs
	// -------------------------------------------------------------------------

	const int32 LeftSpringIndex =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				LeftAnchorIndex,
				Mass3Index,
				30.0,
				0.05,
				1.48
			)
		);

	const int32 RightSpringIndex =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				Mass3Index,
				RightAnchorIndex,
				30.0,
				0.05,
				1.24
			)
		);

	// -------------------------------------------------------------------------
	// Subsystem associations - Bodies
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddBody(
		TEXT("Base"),
		BaseIndex
	);

	TripleInvertedPendulum.AddBody(
		TEXT("Mass1"),
		Mass1Index
	);

	TripleInvertedPendulum.AddBody(
		TEXT("Mass2"),
		Mass2Index
	);

	TripleInvertedPendulum.AddBody(
		TEXT("Mass3"),
		Mass3Index
	);

	TripleInvertedPendulum.AddBody(
		TEXT("LeftAnchor"),
		LeftAnchorIndex
	);

	TripleInvertedPendulum.AddBody(
		TEXT("RightAnchor"),
		RightAnchorIndex
	);

	// -------------------------------------------------------------------------
	// Subsystem associations - Rods
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddElement(
		TEXT("Rod1"),
		Rod1Index
	);

	TripleInvertedPendulum.AddElement(
		TEXT("Rod2"),
		Rod2Index
	);

	TripleInvertedPendulum.AddElement(
		TEXT("Rod3"),
		Rod3Index
	);

	// -------------------------------------------------------------------------
	// Subsystem associations - Springs
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddElement(
		TEXT("LeftSpring"),
		LeftSpringIndex
	);

	TripleInvertedPendulum.AddElement(
		TEXT("RightSpring"),
		RightSpringIndex
	);

	// -------------------------------------------------------------------------
	// Initialization
	// -------------------------------------------------------------------------

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
			ToWorld(Bodies[BodyAIndex].Position),
			ToWorld(Bodies[BodyBIndex].Position),
			Element->GetDebugColor(),
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

	DebugText +=
		TEXT("MULTI SYSTEMS\n");

	const TCHAR* CameraMode =
		TEXT("Unknown");

	if (SimuCameraController)
	{
		CameraMode =
			SimuCameraController->IsUsing1AxisCamera()
			? TEXT("1Axis")
			: TEXT("Free");
	}

	DebugText += FString::Printf(
		TEXT(
			"SPACE Start/Stop\n"
			"Key 'C' Camera Toggle: %s\n"
			"\n"
			"SimulTime: %.2f s | FixedSteps: %llu\n"
			"Delay: %.3f s | AvgDt: %.3f s | FixedDt: %.4f s\n\n"
		),
		CameraMode,
		SimuPhysicalTime,
		FixedStepCount,
		SimulationDelay,
		AverageDeltaTime,
		FixedTimeStep
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

				"L Rel Error: %.3e   ||   Radial Vel: %.3e m/s\n"
			),

			Rod_iIndex + 1,

			InstantErrorData_i.LengthRelError,
			InstantErrorData_i.RadialVelocityError
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