// Fill out your copyright notice in the Description page of Project Settings.

#include "Actors/MultiSystemsActor.h"
#include "Math/Units.h"

#include "Elements/SpringDamper.h"
#include "DrawDebugHelpers.h"

// Sets default values
AMultiSystemsActor::AMultiSystemsActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AMultiSystemsActor::BeginPlay()
{
	Super::BeginPlay();
	
	BuildDemoSystem();
}

// Called every frame
void AMultiSystemsActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	RunSimFixedSteps(DeltaTime);
	DrawSystem();
}

void AMultiSystemsActor::BuildDemoSystem()
{
	FBody Wall;
	Wall.Position = FVector2D(0.0, 0.0);
	Wall.bXFixed = true;
	Wall.bYFixed = true;

	FBody Mass1;
	Mass1.Position = FVector2D(2.0, 0.0);
	Mass1.Velocity = FVector2D(0.0, 0.0);
	Mass1.Mass = 1.0;
	Mass1.bYFixed = true;

	FBody Mass2;
	Mass2.Position = FVector2D(2.5, 0.0);
	Mass2.Velocity = FVector2D(0.0, 0.0);
	Mass2.Mass = 1.0;
	Mass2.bYFixed = true;

	const int32 WallIndex = FullSys.AddBody(Wall);
	const int32 Mass1Index = FullSys.AddBody(Mass1);
	const int32 Mass2Index = FullSys.AddBody(Mass2);

	FullSys.AddElement(MakeUnique<FSpringDamper>(
		WallIndex,
		Mass1Index,
		20.0,
		1.0,
		1.5
	));

	FullSys.AddElement(MakeUnique<FSpringDamper>(
		Mass1Index,
		Mass2Index,
		20.0,   
		1.0,
		1.5
	));
}

void AMultiSystemsActor::RunSimFixedSteps(double FrameDeltaTime)
{
	if (FixedTimeStep <= 0.0 || FrameDeltaTime <= 0.0)
	{
		return;
	}

	RealRunningTime += FrameDeltaTime;
	FrameCount++;

	AverageDeltaTime +=
		(FrameDeltaTime - AverageDeltaTime)
		/ static_cast<double>(FrameCount);

	const double ClampedFrameDeltaTime =
		FMath::Min(FrameDeltaTime, MaxFrameDeltaTime);

	SimulationTimeDebt += ClampedFrameDeltaTime;

	int32 SubStepCount = 0;

	while (
		SimulationTimeDebt >= FixedTimeStep
		&& SubStepCount < MaxSubSteps)
	{
		FullSys.Step(FixedTimeStep);

		SimulatedRunningTime += FixedTimeStep;
		SimulationTimeDebt -= FixedTimeStep;

		SubStepCount++;

		/*if (benablecsvlogging)
		{
			logcurrentsample();
		}*/
	}

	LastSubStepCount = SubStepCount;

	if (SubStepCount >= MaxSubSteps)
	{
		SimulationTimeDebt = FMath::Min(SimulationTimeDebt, FixedTimeStep);
	}

	SimulationDelay = RealRunningTime - SimulatedRunningTime;
}

FVector AMultiSystemsActor::ToWorld(const FVector2D& P) const
{
	return GetActorLocation()
		+ VisualizationOffset
		+ FVector(
			Units::MToCm * P.X,
			0.0,
			Units::MToCm * P.Y
		);
}

void AMultiSystemsActor::DrawSystem() const
{
	const TArray<FBody>& Bodies = FullSys.GetBodies();

	for (const FBody& Body : Bodies)
	{
		DrawDebugSphere(
			GetWorld(),
			ToWorld(Body.Position),
			10.0f,
			16,
			Body.bXFixed && Body.bYFixed ? FColor::Red : FColor::Green,
			false,
			0.0f
		);
	}

	if (Bodies.Num() >= 3)
	{
		DrawDebugLine(GetWorld(), ToWorld(Bodies[0].Position), ToWorld(Bodies[1].Position), FColor::Yellow, false, 0.0f, 0, 2.0f);
		DrawDebugLine(GetWorld(), ToWorld(Bodies[1].Position), ToWorld(Bodies[2].Position), FColor::Yellow, false, 0.0f, 0, 2.0f);
	}
}