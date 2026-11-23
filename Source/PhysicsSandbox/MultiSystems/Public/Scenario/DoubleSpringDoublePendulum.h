#pragma once

#include "CoreMinimal.h"

#include "Simulation/SysState.h"

class FFullSys;


struct FDoubleSpringDoublePendulum
{
	enum EStatePreset : uint8
	{
		Normal,
		HighEnergyRight
	};

	static void BuildSystem(FFullSys& FullSys);

	static FSysState BuildState(EStatePreset Preset);
};