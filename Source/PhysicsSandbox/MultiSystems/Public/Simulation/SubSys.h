#pragma once

#include "CoreMinimal.h"

struct FBodyRef
{
    FName Name;
    int32 BodyIndex = INDEX_NONE;
};

struct FElementRef
{
    FName Name;
    int32 ElementIndex = INDEX_NONE;
};

struct FPort
{
    FName Name;
    int32 BodyIndex = INDEX_NONE;
};

struct FSubSys
{
    FName Name;

    TArray<FBodyRef> Bodies;
    TArray<FElementRef> Elements;
    TArray<FPort> Ports;

    void AddBody(FName BodyName, int32 BodyIndex);
    void AddElement(FName BodyName, int32 ElementIndex);
    void AddPort(FName BodyName, int32 BodyIndex);

    FName FindBodyNameByIndex(int32 BodyIndex) const;
};