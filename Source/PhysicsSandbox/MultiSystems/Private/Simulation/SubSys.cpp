#include "Simulation/SubSys.h"

void FSubSys::AddBody(
    FName BodyName,
    int32 BodyIndex
)
{
    Bodies.Add({    // Construye implícitamente FBodyRef
        BodyName,
        BodyIndex
        });
}

void FSubSys::AddElement( 
    FName BodyName,
    int32 ElementIndex
)
{
    Elements.Add({  // Construye implícitamente FElementRef
        BodyName,
        ElementIndex
        });
}

void FSubSys::AddPort( 
    FName BodyName,
    int32 BodyIndex
)
{
    Ports.Add({     // Construye implícitamente FPort
        BodyName,
        BodyIndex
        });
}

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