#include "Simulation/SubSys.h"

FName FSubSys::FindBodyNameByIndex(int32 BodyIndex) const
{
    for (const FBodyRef& BodyRef : Bodies)
    {
        if (BodyRef.BodyIndex == BodyIndex)
        {
            return BodyRef.Name;
        }
    }

    return TEXT("UnknownBody");
}