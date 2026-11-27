#include "Actors/ActorScenarioMgmt.h"
#include "Actors/MultiSystemsActor.h"

#include "Scenarios/TriplePendulum.h"
#include "Scenarios/TriplePendulumWithTirants.h"
#include "Scenarios/DoubleSpringDoublePendulum.h"

#include "Logging/PhysicsSandboxLog.h"


// =============================================================================
// System initialization
// =============================================================================

template<typename TSystem>
bool AMultiSystemsActor::InitializeSystem(
	typename TSystem::EGeometryPreset GeometryPreset,
	typename TSystem::EActuationPreset ActuationPreset,
	typename TSystem::EStatePreset InitialStatePreset,
	typename TSystem::EStatePreset ReferenceStatePreset		// Ignored if no actuation is configured
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

	const FSysState X_Initial_Full = TSystem::BuildStatePreset(InitialStatePreset);

	FullSys.SetXState2Bodies(X_Initial_Full);
	FullSys.SyncRodStates();


	// Controller initialization
	// ------------------------

	if (ControlSys.GetNumControlChannels() == 0)
	{
		UE_LOG(LogPhysicsSandbox, Display, TEXT("No actuation configured. Reference state ignored."));
		return true;
	}

	const FSysState X_Reference_Full = TSystem::BuildStatePreset(ReferenceStatePreset);

	const Eigen::VectorXd U_Reference = Eigen::VectorXd::Zero(ControlSys.GetNumControlChannels());

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


// =============================================================================
// Scenario initialization
// =============================================================================

bool AMultiSystemsActor::InitializeScenario(EScenarioPreset Preset)
{
	switch (Preset)
	{

		// Triple pendulum
		// ------------------------

	case EScenarioPreset::TriplePendulum_Passive:
	{
		using FScenarioSystem = FTriplePendulum;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::StandardGeometry,
			FScenarioSystem::EActuationPreset::NoActuation,
			FScenarioSystem::EStatePreset::StrongRight,
			FScenarioSystem::EStatePreset::VerticalEquilibrium
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}

	case EScenarioPreset::TriplePendulum_BaseOnly:
	{
		using FScenarioSystem = FTriplePendulum;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::StandardGeometry,
			FScenarioSystem::EActuationPreset::BaseOnly,
			FScenarioSystem::EStatePreset::ModerateLeft,
			FScenarioSystem::EStatePreset::VerticalEquilibrium
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}

	case EScenarioPreset::TriplePendulum_BaseAndMass2:
	{
		using FScenarioSystem = FTriplePendulum;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::StandardGeometry,
			FScenarioSystem::EActuationPreset::BaseAndMass2,
			FScenarioSystem::EStatePreset::StrongRight,
			FScenarioSystem::EStatePreset::VerticalEquilibrium
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}


	// Triple pendulum with tirants - Fixed base
	// ------------------------

	case EScenarioPreset::Tirants_FixedBase_Passive:
	{
		using FScenarioSystem = FTriplePendulumWithTirants;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::FixedBase,
			FScenarioSystem::EActuationPreset::NoActuation,
			FScenarioSystem::EStatePreset::ModerateLeft,
			FScenarioSystem::EStatePreset::VerticalEquilibrium
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}

	case EScenarioPreset::Tirants_FixedBase_Mass1Only:
	{
		using FScenarioSystem = FTriplePendulumWithTirants;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::FixedBase,
			FScenarioSystem::EActuationPreset::Mass1Only,
			FScenarioSystem::EStatePreset::ModerateLeft,
			FScenarioSystem::EStatePreset::VerticalEquilibrium
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}

	case EScenarioPreset::Tirants_FixedBase_Mass3Only:
	{
		using FScenarioSystem = FTriplePendulumWithTirants;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::FixedBase,
			FScenarioSystem::EActuationPreset::Mass3Only,
			FScenarioSystem::EStatePreset::ModerateLeft,
			FScenarioSystem::EStatePreset::VerticalEquilibrium
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}


	// Triple pendulum with tirants - Sliding base
	// ------------------------

	case EScenarioPreset::Tirants_SlidingBase_Passive:
	{
		using FScenarioSystem = FTriplePendulumWithTirants;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::SlidingBase,
			FScenarioSystem::EActuationPreset::NoActuation,
			FScenarioSystem::EStatePreset::StrongRight,
			FScenarioSystem::EStatePreset::VerticalEquilibrium
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}

	case EScenarioPreset::Tirants_SlidingBase_BaseOnly:
	{
		using FScenarioSystem = FTriplePendulumWithTirants;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::SlidingBase,
			FScenarioSystem::EActuationPreset::BaseOnly,
			FScenarioSystem::EStatePreset::SlightLeft,
			FScenarioSystem::EStatePreset::VerticalEquilibrium
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}

	case EScenarioPreset::Tirants_SlidingBase_BaseAndMass1:
	{
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

			return false;
		}

		return true;
	}


	// Double spring double pendulum
	// ------------------------

	case EScenarioPreset::DoubleSpring_Normal:
	{
		using FScenarioSystem = FDoubleSpringDoublePendulum;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::StandardGeometry,
			FScenarioSystem::EActuationPreset::NoActuation,
			FScenarioSystem::EStatePreset::Normal,
			FScenarioSystem::EStatePreset::Normal
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}

	case EScenarioPreset::DoubleSpring_HighEnergy:
	{
		using FScenarioSystem = FDoubleSpringDoublePendulum;

		if (!InitializeSystem<FScenarioSystem>(
			FScenarioSystem::EGeometryPreset::StandardGeometry,
			FScenarioSystem::EActuationPreset::NoActuation,
			FScenarioSystem::EStatePreset::HighEnergyRight,
			FScenarioSystem::EStatePreset::Normal
		))
		{
			UE_LOG(LogPhysicsSandbox, Error, TEXT("Scenario initialization failed. Simulation disabled."));

			bIsSimulationRunning = false;
			SetActorTickEnabled(false);

			return false;
		}

		return true;
	}


	// Invalid scenario
	// ------------------------

	default:
	{
		UE_LOG(LogPhysicsSandbox, Error, TEXT("Invalid scenario preset. Simulation disabled."));

		bIsSimulationRunning = false;
		SetActorTickEnabled(false);

		return false;
	}

	}
}