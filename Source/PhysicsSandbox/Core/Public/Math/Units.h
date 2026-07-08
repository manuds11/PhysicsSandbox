#pragma once

namespace Units
{
    inline constexpr double CmToM = 0.01;
    inline constexpr double MToCm = 100.0;
}

namespace PhysicsConsts
{
    inline constexpr double Gravity = 9.81;
}

namespace CartesianBase
{
    inline const FVector2D e_X(1.0, 0.0);
    inline const FVector2D e_Y(0.0, 1.0);
}