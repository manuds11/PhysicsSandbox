#pragma once

#include "CoreMinimal.h"

#include "Simulation/SysState.h"
#include "Simulation/FullSysTypes.h"

class FFullSys;
class FControlSys;


// =============================================================================
// System variant
// =============================================================================

enum class EDemoSystemVariant : uint8
{
	TriplePendulum,
	TriplePendulumWithTirants,
	DoubleSpringDoublePendulum
};


// =============================================================================
// Control mode
// =============================================================================

enum class EDemoControlMode : uint8
{
	None,
	LQR
};


// =============================================================================
// Named scenarios
// =============================================================================

enum class EDemoScenario : uint8
{
	// Triple pendulum
	TriplePendulum_ModerateLeftRecovery,
	TriplePendulum_ExtremeRightRecovery,

	// Triple pendulum with tirants
	TriplePendulumWithTirants_Default,

	// Passive spring-pendulum system
	DoubleSpringDoublePendulum_Normal,
	DoubleSpringDoublePendulum_HighEnergyRight
};


// =============================================================================
// Scenario configuration
// =============================================================================

struct FDemoScenarioConfig
{
	FName Name;

	EDemoSystemVariant SystemVariant = EDemoSystemVariant::TriplePendulum;
	EDemoControlMode ControlMode = EDemoControlMode::None;

	FSysState InitialState;
	FSysState ReferenceState;
};


// =============================================================================
// Scenario
// =============================================================================

class PHYSICSSANDBOX_API FScenario
{
public:

	static FDemoScenarioConfig GetScenario(EDemoScenario Scenario);

	static void BuildSystem(
		EDemoSystemVariant SystemVariant,
		FFullSys& FullSys,
		FControlSys& ControlSys,
		TArray<FU_ExtForce>& U_ExtForces
	);
};