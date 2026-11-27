#pragma once

#include "CoreMinimal.h"

#include "Simulation/SysState.h"
#include "Simulation/FullSysTypes.h"

class FFullSys;
class FControlSys;


struct FDoubleSpringDoublePendulum
{
public:

	// -----------------------------------------------------------------------------
	// Presets
	// -----------------------------------------------------------------------------

	enum class EGeometryPreset : uint8
	{
		StandardGeometry
	};

	enum class EStatePreset : uint8
	{
		Normal,
		HighEnergyRight
	};

	enum class EActuationPreset : uint8
	{
		NoActuation
	};


	struct FBodiesIndex
	{
		int32 Wall = INDEX_NONE;
		int32 Mass1 = INDEX_NONE;
		int32 Mass2 = INDEX_NONE;
		int32 Bob1 = INDEX_NONE;
		int32 Bob2 = INDEX_NONE;
	};


	// -----------------------------------------------------------------------------
	// Public API
	// -----------------------------------------------------------------------------

	void BuildSystem(
		EGeometryPreset GeometryPreset,
		EActuationPreset ActuationPreset,
		FFullSys& FullSys,
		FControlSys& ControlSys,
		TArray<FU_ExtForce>& U_ExtForces
	);

	static FSysState BuildStatePreset(EStatePreset Preset);

	const FBodiesIndex& GetBodiesIndex() const
	{
		return BodiesIndex;
	}


private:

	FBodiesIndex BodiesIndex;


	// -----------------------------------------------------------------------------
	// System construction
	// -----------------------------------------------------------------------------

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