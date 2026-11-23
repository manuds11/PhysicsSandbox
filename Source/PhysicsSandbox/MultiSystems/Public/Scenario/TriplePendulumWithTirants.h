#pragma once

#include "CoreMinimal.h"

#include "Simulation/SysState.h"

class FFullSys;


struct FTriplePendulumWithTirants
{
	enum EStatePreset : uint8
	{
		Default
	};

	static void BuildSystem(FFullSys& FullSys);

	static FSysState BuildState(EStatePreset Preset);
};