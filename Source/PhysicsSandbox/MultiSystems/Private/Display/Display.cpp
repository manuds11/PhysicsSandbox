#include "Display/Display.h"

#include "Simulation/FullSys.h"
#include "Control/ControlSys.h"
#include "DrawDebugHelpers.h"
#include "Math/Units.h"
                     
void FMultiSysDisplay::Initialize(
    UWorld* InWorld,
    const FFullSys* InFullSys,
    const FControlSys* InControlSys,
    const FVector& InVisualizationOffset,
    bool bInDrawControlForces,
    double InControlForceArrowScale,
    double InControlForceArrowMaxLength
)
{
    World = InWorld;
    FullSys = InFullSys;
    ControlSys = InControlSys;

    VisualizationOffset = InVisualizationOffset;

    bDrawControlForces = bInDrawControlForces;
    ControlForceArrowScale = InControlForceArrowScale;
    ControlForceArrowMaxLength = InControlForceArrowMaxLength;
}

void FMultiSysDisplay::UpdateActorLocation(const FVector& InActorLocation)
{
    ActorLocation = InActorLocation;
}

FVector FMultiSysDisplay::ToWorld(
    const FVector2D& P
) const
{
    return ActorLocation
        + VisualizationOffset
        + FVector(
            Units::MToCm * P.X,
            0.0,
            Units::MToCm * P.Y
        );
}

void FMultiSysDisplay::DrawSystem() const
{
    if (!World || !FullSys)
    {
        return;
    }

    const TArray<FBody>& Bodies =
        FullSys->GetBodies();

    for (const FBody& Body : Bodies)
    {
        const double DebugRadius =
            ComputeBodyDebugRadius(
                Body.Mass
            );

        DrawDebugSphere(
            World,
            ToWorld(Body.Position),
            DebugRadius,
            16,
            Body.bXFixed && Body.bYFixed
            ? FColor::Red
            : FColor::Green,
            false,
            0.0f
        );
    }

    const TArray<TUniquePtr<ISysElement>>& Elements =
        FullSys->GetElements();

    for (const TUniquePtr<ISysElement>& Element : Elements)
    {
        if (!Element)
        {
            continue;
        }

        int32 BodyAIndex = INDEX_NONE;
        int32 BodyBIndex = INDEX_NONE;

        if (!Element->GetConnectedBodies(
            BodyAIndex,
            BodyBIndex
        ))
        {
            continue;
        }

        if (
            !Bodies.IsValidIndex(BodyAIndex)
            || !Bodies.IsValidIndex(BodyBIndex)
            )
        {
            continue;
        }

        DrawDebugLine(
            World,
            ToWorld(Bodies[BodyAIndex].Position),
            ToWorld(Bodies[BodyBIndex].Position),
            Element->GetDebugColor(),
            false,
            0.0f,
            0,
            2.0f
        );

        FVector2D EquilibriumPoint;

        if (
            Element->GetEquilibriumPoint(
                Bodies,
                EquilibriumPoint
            )
            )
        {
            DrawDebugLine(
                World,
                ToWorld(EquilibriumPoint)
                + FVector(0.0, 0.0, -50.0),
                ToWorld(EquilibriumPoint)
                + FVector(0.0, 0.0, 50.0),
                FColor::Blue,
                false,
                0.0f,
                0,
                2.0f
            );
        }
    }

    DrawControlForces();
}

// -----------------------------------------------------------------------------
// Display Helpers
// -----------------------------------------------------------------------------

double FMultiSysDisplay::ComputeBodyDebugRadius(
    double Mass
) const
{
    constexpr double ReferenceMass = 1.0;
    constexpr double ReferenceRadius = 8.0;

    if (Mass <= UE_SMALL_NUMBER)
    {
        return ReferenceRadius;
    }

    const double ComputedRadius =
        ReferenceRadius
        * FMath::Pow(
            Mass / ReferenceMass,
            1.0 / 3.0
        );

    return FMath::Clamp(
        ComputedRadius,
        4.0,
        20.0
    );
}

void FMultiSysDisplay::DrawStatePerturbation(
	const Eigen::VectorXd& DeltaX_Full,
	double PositionScale,
	double VelocityScale
) const
{
	const TArray<FBody>& Bodies =
		FullSys->GetBodies();

	for (
		int32 BodyIndex = 0;
		BodyIndex < Bodies.Num();
		++BodyIndex
		)
	{
		DrawStatePerturbationAtBody(
			BodyIndex,
			DeltaX_Full,
			PositionScale,
			VelocityScale
		);
	}
}

void FMultiSysDisplay::DrawStatePerturbationAtBody(
	int32 BodyIndex,
	const Eigen::VectorXd& DeltaX_Full,
	double PositionScale,
	double VelocityScale
) const
{
	const TArray<FBody>& Bodies =
		FullSys->GetBodies();

	check(
		Bodies.IsValidIndex(BodyIndex)
	);

	const int32 SizeQ_Full =
		FullSys->GetStateIndexMap().SizeQ_Full;

	check(
		DeltaX_Full.size()
		== 2 * SizeQ_Full
	);

	const FBody& Body =
		Bodies[BodyIndex];

	const int32 QXIndex =
		2 * BodyIndex;

	const int32 QYIndex =
		QXIndex + 1;

	const int32 VXIndex =
		SizeQ_Full + QXIndex;

	const int32 VYIndex =
		SizeQ_Full + QYIndex;

	const FVector2D BodyDeltaPos(
		DeltaX_Full(QXIndex),
		DeltaX_Full(QYIndex)
	);

	const FVector2D BodyDeltaVel(
		DeltaX_Full(VXIndex),
		DeltaX_Full(VYIndex)
	);

	const FVector2D DeltaPosScaled =
		Body.Position
		+ PositionScale * BodyDeltaPos;

	const FVector2D DeltaVelScaled =
		Body.Position
		+ VelocityScale * BodyDeltaVel;

	DrawDebugDirectionalArrow(
		World,
		ToWorld(Body.Position),
		ToWorld(DeltaPosScaled),
		8.0f,
		FColor::Yellow,
		false,
		0.0f,
		0,
		2.0f
	);

	DrawDebugSphere(
		World,
		ToWorld(DeltaPosScaled),
		4.0f,
		12,
		FColor::Yellow,
		false,
		0.0f
	);

	DrawDebugDirectionalArrow(
		World,
		ToWorld(Body.Position),
		ToWorld(DeltaVelScaled),
		8.0f,
		FColor::Cyan,
		false,
		0.0f,
		0,
		2.0f
	);
}

void FMultiSysDisplay::DrawKernelZVector(
	const Eigen::VectorXd& Z_ker
) const
{
	Eigen::VectorXd DeltaX_Full;

	if (
		!ControlSys->BuildDeltaX_FullFromZ_ker(
			*FullSys,
			Z_ker,
			DeltaX_Full
		)
		)
	{
		return;
	}

	constexpr double PositionScale = 0.5;
	constexpr double VelocityScale = 0.5;

	DrawStatePerturbation(
		DeltaX_Full,
		PositionScale,
		VelocityScale
	);
}

void FMultiSysDisplay::DrawKernelZVectorAtBody(
	int32 BodyIndex,
	const Eigen::VectorXd& Z_ker
) const
{
	Eigen::VectorXd DeltaX_Full;

	if (
		!ControlSys->BuildDeltaX_FullFromZ_ker(
			*FullSys,
			Z_ker,
			DeltaX_Full
		)
		)
	{
		return;
	}

	constexpr double PositionScale = 0.5;
	constexpr double VelocityScale = 0.5;

	DrawStatePerturbationAtBody(
		BodyIndex,
		DeltaX_Full,
		PositionScale,
		VelocityScale
	);
}

void FMultiSysDisplay::DrawControlForces() const
{
	if (!bDrawControlForces)
	{
		return;
	}

	const TArray<FLinearControlChannel>& ControlChannels =
		ControlSys->GetControlChannels();

	const Eigen::VectorXd& U_Control =
		ControlSys->GetUControl();

	const TArray<FBody>& Bodies =
		FullSys->GetBodies();

	check(
		U_Control.size()
		== ControlChannels.Num()
	);

	for (
		int32 ChannelIndex = 0;
		ChannelIndex < ControlChannels.Num();
		++ChannelIndex
		)
	{
		const FLinearControlChannel& Channel =
			ControlChannels[ChannelIndex];

		const double U =
			U_Control(ChannelIndex);

		for (
			const FControlForceContribution& Contribution
			: Channel.ForceContributions
			)
		{
			const FVector2D ControlForce =
				U * Contribution.ForcePerUnitU;

			const double ForceMagnitude =
				ControlForce.Size();

			if (
				ForceMagnitude
				<= UE_SMALL_NUMBER
				)
			{
				continue;
			}

			const FVector2D ForceDirection =
				ControlForce
				/ ForceMagnitude;

			const double ArrowLength =
				FMath::Min(
					ForceMagnitude
					* ControlForceArrowScale,
					ControlForceArrowMaxLength
				);

			const FBody& Body =
				Bodies[
					Contribution.BodyApplicationIndex
				];

			const FVector ArrowStart =
				ToWorld(Body.Position);

			const FVector ArrowDirection(
				ForceDirection.X,
				0.0,
				ForceDirection.Y
			);

			const FVector ArrowEnd =
				ArrowStart
				+ ArrowDirection
				* ArrowLength;

			DrawDebugDirectionalArrow(
				World,
				ArrowStart,
				ArrowEnd,
				12.0f,
				FColor::Cyan,
				false,
				0.0f,
				0,
				3.0f
			);
		}
	}
}