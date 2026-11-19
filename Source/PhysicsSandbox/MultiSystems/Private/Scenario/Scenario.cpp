#include "Scenario/Scenario.h"


FDemoScenarioConfig FScenario::GetScenario(EDemoScenario Scenario)
{
	FDemoScenarioConfig Config;

	switch (Scenario)
	{
	case EDemoScenario::PassivePendulum:
		Config.Name = TEXT("Passive Pendulum");
		Config.SystemVariant = EDemoSystemVariant::TriplePendulum;

		Config.InitialState = EDemoStatePreset::SlightLeft;

		Config.bUseControl = false;
		break;

	case EDemoScenario::EquilibriumValidation:
		Config.Name = TEXT("Equilibrium Validation");
		Config.SystemVariant = EDemoSystemVariant::TriplePendulum;

		Config.InitialState = EDemoStatePreset::VerticalEquilibrium;
		Config.ReferenceState = EDemoStatePreset::VerticalEquilibrium;

		Config.bUseControl = true;
		break;

	case EDemoScenario::SlightLeftRecovery:
		Config.Name = TEXT("Slight Left Recovery");
		Config.SystemVariant = EDemoSystemVariant::TriplePendulum;

		Config.InitialState = EDemoStatePreset::SlightLeft;
		Config.ReferenceState = EDemoStatePreset::VerticalEquilibrium;

		Config.bUseControl = true;
		break;

	case EDemoScenario::SlightRightRecovery:
		Config.Name = TEXT("Slight Right Recovery");
		Config.SystemVariant = EDemoSystemVariant::TriplePendulum;

		Config.InitialState = EDemoStatePreset::SlightRight;
		Config.ReferenceState = EDemoStatePreset::VerticalEquilibrium;

		Config.bUseControl = true;
		break;

	default:
		checkNoEntry();
		break;
	}

	return Config;
}