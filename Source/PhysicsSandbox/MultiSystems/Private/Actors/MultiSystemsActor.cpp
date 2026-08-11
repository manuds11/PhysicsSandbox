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

	FSubSys& SuspendedRodQuadrilateral =
		FullSys.CreateSubSys(
			TEXT("SuspendedRodQuadrilateral")
		);

	// -------------------------------------------------------------------------
	// Bodies
	// -------------------------------------------------------------------------

	const int32 BodyAIndex =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(-1.4, 1.8),
				1.0
			)
		);

	const int32 BodyBIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(1.0, 1.5)
			)
		);

	const int32 BodyCIndex =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(1.3, -0.3),
				1.8
			)
		);

	const int32 BodyDIndex =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(-0.8, -0.9),
				0.7
			)
		);

	// -------------------------------------------------------------------------
	// Elements
	// -------------------------------------------------------------------------

	const int32 RodABIndex =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BodyAIndex,
				BodyBIndex
			)
		);

	const int32 RodBCIndex =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BodyBIndex,
				BodyCIndex
			)
		);

	const int32 RodCDIndex =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BodyCIndex,
				BodyDIndex
			)
		);

	const int32 RodDAIndex =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BodyDIndex,
				BodyAIndex
			)
		);

	const int32 DiagonalSpringACIndex =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				BodyAIndex,
				BodyCIndex,
				18.0,
				0.35,
				2.6
			)
		);

	const int32 DiagonalSpringBDIndex =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				BodyBIndex,
				BodyDIndex,
				11.0,
				0.65,
				2.9
			)
		);

	// -------------------------------------------------------------------------
	// Subsystem associations
	// -------------------------------------------------------------------------

	SuspendedRodQuadrilateral.AddBody(
		TEXT("BodyA"),
		BodyAIndex
	);

	SuspendedRodQuadrilateral.AddBody(
		TEXT("BodyB"),
		BodyBIndex
	);

	SuspendedRodQuadrilateral.AddBody(
		TEXT("BodyC"),
		BodyCIndex
	);

	SuspendedRodQuadrilateral.AddBody(
		TEXT("BodyD"),
		BodyDIndex
	);

	SuspendedRodQuadrilateral.AddElement(
		TEXT("RodAB"),
		RodABIndex
	);

	SuspendedRodQuadrilateral.AddElement(
		TEXT("RodBC"),
		RodBCIndex
	);

	SuspendedRodQuadrilateral.AddElement(
		TEXT("RodCD"),
		RodCDIndex
	);

	SuspendedRodQuadrilateral.AddElement(
		TEXT("RodDA"),
		RodDAIndex
	);

	SuspendedRodQuadrilateral.AddElement(
		TEXT("DiagonalSpring"),
		DiagonalSpringACIndex
	);

	SuspendedRodQuadrilateral.AddElement(
		TEXT("DiagonalSpringAC"),
		DiagonalSpringACIndex
	);


	SuspendedRodQuadrilateral.AddElement(
		TEXT("DiagonalSpringBD"),
		DiagonalSpringBDIndex
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