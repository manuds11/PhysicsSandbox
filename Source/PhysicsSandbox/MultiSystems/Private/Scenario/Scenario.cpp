#include "Scenario/Scenario.h"

#include "Scenario/TriplePendulum.h"
#include "Scenario/TriplePendulumWithTirants.h"
#include "Scenario/DoubleSpringDoublePendulum.h"


FDemoScenarioConfig FScenario::GetScenario(EDemoScenario Scenario)
{
	FDemoScenarioConfig Config;

	switch (Scenario)
	{
		// =========================================================================
		// Triple pendulum
		// =========================================================================

	case EDemoScenario::TriplePendulum_ModerateLeftRecovery:

		Config.Name = TEXT("Triple Pendulum - Moderate Left Recovery");
		Config.SystemVariant = EDemoSystemVariant::TriplePendulum;
		Config.ControlMode = EDemoControlMode::LQR;

		Config.InitialState =
			FTriplePendulum::BuildState(
				FTriplePendulum::ModerateLeft
			);

		Config.ReferenceState =
			FTriplePendulum::BuildState(
				FTriplePendulum::VerticalEquilibrium
			);

		break;


	case EDemoScenario::TriplePendulum_ExtremeRightRecovery:

		Config.Name = TEXT("Triple Pendulum - Extreme Right Recovery");
		Config.SystemVariant = EDemoSystemVariant::TriplePendulum;
		Config.ControlMode = EDemoControlMode::LQR;

		Config.InitialState =
			FTriplePendulum::BuildState(
				FTriplePendulum::ExtremeRight
			);

		Config.ReferenceState =
			FTriplePendulum::BuildState(
				FTriplePendulum::VerticalEquilibrium
			);

		break;


		// =========================================================================
		// Triple pendulum with tirants
		// =========================================================================

	case EDemoScenario::TriplePendulumWithTirants_Default:

		Config.Name = TEXT("Triple Pendulum With Tirants");
		Config.SystemVariant = EDemoSystemVariant::TriplePendulumWithTirants;
		Config.ControlMode = EDemoControlMode::None;

		Config.InitialState =
			FTriplePendulumWithTirants::BuildState(
				FTriplePendulumWithTirants::Default
			);

		break;


		// =========================================================================
		// Double spring - double pendulum
		// =========================================================================

	case EDemoScenario::DoubleSpringDoublePendulum_Normal:

		Config.Name = TEXT("Double Spring Double Pendulum - Normal");
		Config.SystemVariant = EDemoSystemVariant::DoubleSpringDoublePendulum;
		Config.ControlMode = EDemoControlMode::None;

		Config.InitialState =
			FDoubleSpringDoublePendulum::BuildState(
				FDoubleSpringDoublePendulum::Normal
			);

		break;


	case EDemoScenario::DoubleSpringDoublePendulum_HighEnergyRight:

		Config.Name = TEXT("Double Spring Double Pendulum - High Energy Right");
		Config.SystemVariant = EDemoSystemVariant::DoubleSpringDoublePendulum;
		Config.ControlMode = EDemoControlMode::None;

		Config.InitialState =
			FDoubleSpringDoublePendulum::BuildState(
				FDoubleSpringDoublePendulum::HighEnergyRight
			);

		break;


	default:
		checkNoEntry();
		break;
	}

	return Config;
}