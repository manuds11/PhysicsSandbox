// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Simulation/FullSys.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MultiSystemsActor.generated.h"

UCLASS()
class PHYSICSSANDBOX_API AMultiSystemsActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMultiSystemsActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	FFullSys FullSys;

	void BuildDemoSystem();
	void DrawSystem() const;
};
