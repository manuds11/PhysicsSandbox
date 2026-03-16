// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DebugProbeActor.generated.h"

class UStaticMeshComponent;

UCLASS()
class PHYSICSSANDBOX_API ADebugProbeActor : public AActor
{
	GENERATED_BODY()

public:
	ADebugProbeActor();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

private:
	// Altura base desde la que oscila
	FVector StartLocation;

	// Componente visible del actor (siempre existe)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "DebugProbe", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UStaticMeshComponent> MeshComp;

public:
	// Velocidad de oscilación (rad/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DebugProbe")
	float Speed = 2.0f;

	// Amplitud en cm
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DebugProbe")
	float Amplitude = 50.0f;
};