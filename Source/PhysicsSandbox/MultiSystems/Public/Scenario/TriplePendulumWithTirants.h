#pragma once

#include "CoreMinimal.h"

#include "Simulation/SysState.h"
#include "Simulation/FullSysTypes.h"

class FFullSys;
class FControlSys;


struct FTriplePendulumWithTirants
{
public:

	// -----------------------------------------------------------------------------
	// Presets
	// -----------------------------------------------------------------------------

	enum EGeometryPreset : uint8
	{
		SlidingBase,
		FixedBase
	};

	enum EStatePreset : uint8
	{
		VerticalEquilibrium,
		Default,
		SlightLeft,
		ModerateLeft,
		StrongRight
	};

	enum EActuationPreset : uint8
	{
		NoActuation,
		BaseOnly,
		Mass1Only,
		Mass3Only
	};


	struct FBodiesIndex
	{
		int32 Base = INDEX_NONE;
		int32 Mass1 = INDEX_NONE;
		int32 Mass2 = INDEX_NONE;
		int32 Mass3 = INDEX_NONE;
		int32 LeftAnchor = INDEX_NONE;
		int32 RightAnchor = INDEX_NONE;
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