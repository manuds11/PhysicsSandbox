// Fill out your copyright notice in the Description page of Project Settings.

#include "Actors/MultiSystemsActor.h"

#include "Scenario/TriplePendulum.h"
#include "Scenario/TriplePendulumWithTirants.h"
#include "Scenario/DoubleSpringDoublePendulum.h"

#include "Elements/PendulumRod.h"
#include "Elements/SpringDamper.h"

#include "Math/Units.h"

#include "Logging/PhysicsSandboxLog.h"

#include "DrawDebugHelpers.h"

#include "ThirdParty/EigenIncludes.h"

// =============================================================================
// Construction
// =============================================================================

AMultiSystemsActor::AMultiSystemsActor()
{
 	// Set this actor to call Tick() every frame. You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;	
}

void AMultiSystemsActor::BeginPlay()
{
	Super::BeginPlay();


	// Scenario
	// ------------------------

	using FScenarioSystem = FTriplePendulumWithTirants;

	if (!InitializeSystem<FScenarioSystem>(
		FScenarioSystem::EGeometryPreset::SlidingBase,
		FScenarioSystem::EActuationPreset::BaseAndMass1,
		FScenarioSystem::EStatePreset::StrongRight,
		FScenarioSystem::EStatePreset::VerticalEquilibrium
	))
	{
		UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

		bIsSimulationRunning = false;
		SetActorTickEnabled(false);

		return;
	}


	// Input
	// ------------------------

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


	// Display
	// ------------------------

	Display.Initialize(
		GetWorld(),
		&FullSys,
		&ControlSys,
		VisualizationOffset,
		bDrawControlForces,
		ControlForceArrowScale,
		ControlForceArrowMaxLength
	);
}

// Scenario initialization and its helpers
// -----------------------------------------------------------------------------

template<typename TSystem>
bool AMultiSystemsActor::InitializeSystem(
	typename TSystem::EGeometryPreset GeometryPreset,
	typename TSystem::EActuationPreset ActuationPreset,
	typename TSystem::EStatePreset InitialStatePreset,
	typename TSystem::EStatePreset ReferenceStatePreset
)
{
	// System construction
	// ------------------------

	TSystem System;

	System.BuildSystem(
		GeometryPreset,
		ActuationPreset,
		FullSys,
		ControlSys,
		U_ExtForces
	);


	// Physical initialization
	// ------------------------

	FullSys.Initialize();
	ConfigureSimulation();


	// Control allocation
	// ------------------------

	if (!ControlSys.InitializeControlAllocation(FullSys))
	{
		UE_LOG(LogPhysicsSandbox, Error, TEXT("Failed to initialize control allocation."));
		return false;
	}


	// Initial state
	// ------------------------

	const FSysState X_Initial_Full = TSystem::BuildStatePreset(
		InitialStatePreset
	);

	FullSys.SetXState2Bodies(X_Initial_Full);
	FullSys.SyncRodStates();


	// Controller initialization
	// ------------------------

	if (ControlSys.GetNumControlChannels() == 0)
	{
		UE_LOG(LogPhysicsSandbox, Display, TEXT("No actuation configured. Reference state ignored."));
		return true;
	}

	const FSysState X_Reference_Full = TSystem::BuildStatePreset(
		ReferenceStatePreset
	);

	const Eigen::VectorXd U_Reference = Eigen::VectorXd::Zero(
		ControlSys.GetNumControlChannels()
	);

	ControlSys.SetReference(X_Reference_Full, U_Reference);

	if (!ControlSys.InitializeLQRController(
		FullSys,
		FixedTimeStep,
		U_ExtForces
	))
	{
		UE_LOG(LogPhysicsSandbox, Error, TEXT("Failed to initialize LQR controller."));
		return false;
	}

	return true;
}

void AMultiSystemsActor::ConfigureSimulation()
{
	ConfigureIntegrator();

	FullSys.SetVelocityCorrection1Enabled(true);
	FullSys.SetPositionCorrectionsEnabled(true);
	FullSys.SetVelocityCorrection2Enabled(true);
}

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
// Lifecycle
// -----------------------------------------------------------------------------

void AMultiSystemsActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (bIsSimulationRunning)
	{
		RunSimFixedSteps(DeltaTime);
	}

	// Display
	// ------------------------

	Display.UpdateActorLocation(GetActorLocation());
	Display.DrawSystem();
	PrintInfo();
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

		if (!RunPlant(FixedTimeStep))
		{
			bIsSimulationRunning = false;
			break;
		}

		++FixedStepCount;
		SimuPhysicalTime += FixedTimeStep;
		SimulationTimeDebt -= FixedTimeStep;
		++SubStepCount;

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

bool AMultiSystemsActor::RunPlant(double Dt)
{
	if (Dt <= 0.0)
	{
		return false;
	}

	const FSysState X_k = FullSys.BodiesToFullStateStruct();
	const Eigen::VectorXd U_Control = ControlSys.ComputeUControl(X_k, FullSys.GetStateIndexMap());
	const Eigen::VectorXd Tau_FR = ControlSys.ComputeTau_FR(U_Control);
	FSysState X_kPlus1;

	return FullSys.Step(Dt, X_k, Tau_FR, U_ExtForces, X_kPlus1);
}

// -----------------------------------------------------------------------------
// HUD 
// ----------------------------------------------------------------------------

void AMultiSystemsActor::PrintInfo() const
{
	if (!GEngine)
	{
		return;
	}

	FString DebugText;

	DebugText +=
		TEXT("MULTI SYSTEMS\n");

	// =========================================================================
	// Input / camera info
	// =========================================================================

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
			"SPACE Start/Stop | Camera C: %s\n"
		),
		CameraMode
	);

	// =========================================================================
	// Simulation time
	// =========================================================================

	const double RealTimeFactor =
		SimuWallClockTime > UE_SMALL_NUMBER
		? SimuPhysicalTime / SimuWallClockTime
		: 0.0;

	DebugText += FString::Printf(
		TEXT(
			"Sim time: %.2f s | Wall clock: %.2f s | Delay: %+.3f s | RTF: %.3f | FixedSteps: %llu | FixedDt: %.5f s\n"
			"AvgFrameDt: %.4f s | LastSubSteps: %d\n"
		),
		SimuPhysicalTime,
		SimuWallClockTime,
		SimulationDelay,
		RealTimeFactor,
		FixedStepCount,
		FixedTimeStep,
		AverageDeltaTime,
		LastSubStepCount
	);

	// =========================================================================
	// Rod diagnostics
	// =========================================================================

	/*
	const TArray<FPendulumRod*>& RodSystemArray =
		FullSys.GetRodSystemArray();

	const TArray<double>& RodSystemLambda =
		FullSys.GetRodSystemLambdaVector();

	check(
		RodSystemArray.Num()
		== RodSystemLambda.Num()
	);

	for (int32 Rod_iIndex = 0; Rod_iIndex < RodSystemArray.Num(); ++Rod_iIndex)
	{
		const FPendulumRod* Rod_i =
			RodSystemArray[Rod_iIndex];

		if (!Rod_i)
		{
			continue;
		}

		const FInstantErrorData& InstantErrorData_i =
			Rod_i->GetInstantErrorData();

		const double Lambda_i =
			RodSystemLambda[Rod_iIndex];

		DebugText += FString::Printf(
			TEXT(
				"Rod %d | L Rel Error: %.3e | Radial Vel: %.3e m/s | Lambda: %.3e\n"
			),
			Rod_iIndex + 1,
			InstantErrorData_i.LengthRelError,
			InstantErrorData_i.RadialVelocityError,
			Lambda_i
		);
	}
	*/

	// =========================================================================
	// Energy balance
	// =========================================================================

	const FSystemEnergyBalance& Balance =
		FullSys.GetEnergyBalance();

	const FSystemEnergyState& CurrentEnergyState =
		Balance.Current;

	const FConstraintCorrectionEnergyDiagnostics& CorrectionBalance =
		FullSys.GetCorrectionEnergyDiagnostics();

	const double EnergyResidual =
		Balance.EnergyResidualSinceStart();

	const double EnergyResidualExcludingCorrections =
		EnergyResidual - CorrectionBalance.AccumulatedProjectionEnergy();

	DebugText += FString::Printf(
		TEXT(
			"\nENERGY BALANCE\n"
			"Initial Mech: %.4f J | Mechanical: %.4f J | Delta Mech: %+.4f J | Dissipated: %.4f J\n"
			"Control Work: %+.4f J | Accum Correction: %+.4f J\n" //  | External Work: Disabled
			"Energy Residual: %+.6f J | Residual Excl. Corrections: %+.6f J\n"
		),
		Balance.Initial.Mechanical(),
		CurrentEnergyState.Mechanical(),
		Balance.MechanicalVariationSinceStart(),
		CurrentEnergyState.TotalDissipated,

		CurrentEnergyState.TotalControlWork,
		CorrectionBalance.AccumulatedProjectionEnergy(),

		EnergyResidual,
		EnergyResidualExcludingCorrections
	);

	// =========================================================================
	// Energy components
	// =========================================================================

	/*
	DebugText += FString::Printf(
		TEXT(
			"\nENERGY COMPONENTS\n"
			"Kinetic: %.6f J | Gravitational: %.6f J | Elastic: %.6f J\n"
		),
		CurrentEnergyState.Kinetic,
		CurrentEnergyState.Gravitational,
		CurrentEnergyState.Elastic
	);
	*/

	// =========================================================================
	// Current step energy increments
	// =========================================================================

	/*
	const FDeltaEnergy& IntegrationBalance =
		FullSys.GetIntegrationEnergy();

	DebugText += FString::Printf(
		TEXT(
			"\nSTEP ENERGY INCREMENTS\n"
			"Integration: %+.6f J | Vel1: %+.6f J | Pos: %+.6f J | Vel2: %+.6f J | "
			"Total Correction: %+.6f J | Total Step: %+.6f J\n"
		),
		IntegrationBalance.DeltaMechanical(),
		CorrectionBalance.InitialVelocityProjection.DeltaMechanical(),
		CorrectionBalance.PositionProjection.DeltaMechanical(),
		CorrectionBalance.FinalVelocityProjection.DeltaMechanical(),
		CorrectionBalance.StepProjectionEnergyIncrement(),
		IntegrationBalance.DeltaMechanical()
			+ CorrectionBalance.StepProjectionEnergyIncrement()
	);
	*/

	// =========================================================================
	// Detailed accumulated correction energy
	// =========================================================================

	/*
	DebugText += FString::Printf(
		TEXT(
			"\nACCUMULATED CORRECTION ENERGY\n"
			"Vel1: %+.6f J | Pos: %+.6f J | Vel2: %+.6f J | Total: %+.6f J\n"
		),
		CorrectionBalance.AccumulatedInitialVelocity.Mechanical(),
		CorrectionBalance.AccumulatedPosition.Mechanical(),
		CorrectionBalance.AccumulatedFinalVelocity.Mechanical(),
		CorrectionBalance.AccumulatedProjectionEnergy()
	);
	*/

	// =========================================================================
	// LQR design timing - To be implemented
	// =========================================================================

	/*
	DebugText += FString::Printf(
		TEXT(
			"\nLQR DESIGN\n"
			"Kernel: %.3f ms | A: %.3f ms | B: %.3f ms | Controllability: %.3f ms | "
			"DARE: %.3f ms | K: %.3f ms | Total: %.3f ms\n"
		),
		...
	);
	*/

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