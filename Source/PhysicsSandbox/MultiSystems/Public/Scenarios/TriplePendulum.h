#pragma once

#include "CoreMinimal.h"

#include "Simulation/SysState.h"
#include "Simulation/FullSysTypes.h"

class FFullSys;
class FControlSys;


struct FTriplePendulum
{
public:

	enum class EGeometryPreset : uint8
	{
		StandardGeometry
	};

	enum class EStatePreset : uint8
	{
		VerticalEquilibrium,
		SlightLeft,
		ModerateLeft,
		StrongLeft,
		StrongRight,
		ExtremeRight
	};

	enum class EActuationPreset : uint8
	{
		NoActuation,
		BaseOnly,
		BaseAndMass2
	};


	struct FBodiesIndex
	{
		int32 Base = INDEX_NONE;
		int32 Mass1 = INDEX_NONE;
		int32 Mass2 = INDEX_NONE;
		int32 Mass3 = INDEX_NONE;
	};

	const FBodiesIndex& GetBodiesIndex() const
	{
		return BodiesIndex;
	}

	void BuildSystem(
		EGeometryPreset GeometryPreset,
		EActuationPreset ActuationPreset,
		FFullSys& FullSys,
		FControlSys& ControlSys,
		TArray<FU_ExtForce>& U_ExtForces
	);

	static FSysState BuildStatePreset(
		EStatePreset Preset
	);

private:

	FBodiesIndex BodiesIndex;

	void BuildGeometry(
		EGeometryPreset Preset,
		FFullSys& FullSys
	);

	void ConfigureActuation(
		EActuationPreset Preset,
		FControlSys& ControlSys
	);

	void AddExternalForces(
		TArray<FU_ExtForce>& U_ExtForces
	);
};