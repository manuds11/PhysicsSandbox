#pragma once

#include "CoreMinimal.h"

THIRD_PARTY_INCLUDES_START

#pragma push_macro("check")
#undef check

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <contrib/Eigen/Riccati>

#pragma pop_macro("check")

THIRD_PARTY_INCLUDES_END