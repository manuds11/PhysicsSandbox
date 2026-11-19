#pragma once

#include "CoreMinimal.h"


enum class EDemoSystemVariant : uint8
{
	TriplePendulum,
	TriplePendulumWithTirants
};


enum class EDemoStatePreset : uint8
{
	VerticalEquilibrium,
	SlightLeft,
	SlightRight
};


enum class EDemoScenario : uint8
{
	PassivePendulum,
	EquilibriumValidation,
	SlightLeftRecovery,
	SlightRightRecovery
};


struct FDemoScenarioConfig
{
	FName Name;

	EDemoSystemVariant SystemVariant = EDemoSystemVariant::TriplePendulum;

	EDemoStatePreset InitialState = EDemoStatePreset::VerticalEquilibrium;
	EDemoStatePreset ReferenceState = EDemoStatePreset::VerticalEquilibrium;

	bool bUseControl = false;
};


class PHYSICSSANDBOX_API FScenario
{
public:

	static FDemoScenarioConfig GetScenario(EDemoScenario Scenario);
};