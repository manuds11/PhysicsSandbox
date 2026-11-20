#pragma once

#include "CoreMinimal.h"

#include "Simulation/SysState.h"
#include "Simulation/FullSysTypes.h"

class FFullSys;
class FControlSys;


struct FTriplePendulum
{
	enum EStatePreset : uint8
	{
		VerticalEquilibrium,
		SlightLeft,
		ModerateLeft,
		StrongLeft,
		StrongRight,
		ExtremeRight
	};

	static void BuildSystem(
		FFullSys& FullSys,
		FControlSys& ControlSys,
		TArray<FU_ExtForce>& U_ExtForces
	);

	static FSysState BuildState(EStatePreset Preset);
};