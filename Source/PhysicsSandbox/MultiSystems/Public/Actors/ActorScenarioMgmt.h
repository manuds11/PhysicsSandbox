#pragma once

#include "CoreMinimal.h"

enum class EScenarioPreset : uint8
{
	// Triple pendulum
	TriplePendulum_Passive,
	TriplePendulum_BaseOnly,
	TriplePendulum_BaseAndMass2,

	// Triple pendulum with tirants - Fixed base
	Tirants_FixedBase_Passive,
	Tirants_FixedBase_Mass1Only,
	Tirants_FixedBase_Mass3Only,

	// Triple pendulum with tirants - Sliding base
	Tirants_SlidingBase_Passive,
	Tirants_SlidingBase_BaseOnly,
	Tirants_SlidingBase_BaseAndMass1,

	// Double spring double pendulum
	DoubleSpring_Normal,
	DoubleSpring_HighEnergy
};