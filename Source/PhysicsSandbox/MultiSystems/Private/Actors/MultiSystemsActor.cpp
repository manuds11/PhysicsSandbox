// Fill out your copyright notice in the Description page of Project Settings.

#include "Actors/MultiSystemsActor.h"
#include "Math/Units.h"
#include "Elements/PendulumRod.h"
#include "Elements/SpringDamper.h"
#include "DrawDebugHelpers.h"

#include "Logging/PhysicsSandboxLog.h"

#include <Eigen/Eigenvalues>

// -----------------------------------------------------------------------------
// Construction and lifecycle
// -----------------------------------------------------------------------------

AMultiSystemsActor::AMultiSystemsActor()
{
 	// Set this actor to call Tick() every frame. You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;	
}

void AMultiSystemsActor::BeginPlay()
{
	Super::BeginPlay();
	
	BuildDemoSystem();

	ConfigureIntegrator();

	FullSys.SetVelocityCorrection1Enabled(true);
	FullSys.SetPositionCorrectionsEnabled(true);
	FullSys.SetVelocityCorrection2Enabled(true);

	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		EnableInput(PC);
	}

	if (InputComponent)
	{
		InputComponent->BindKey(
			EKeys::SpaceBar,
			IE_Pressed,
			this,
			&AMultiSystemsActor::ToggleSimulation
		);
	}

	if (!ControlSys.DesignLQRController(
		FullSys,
		FixedTimeStep,
		X_Reference_Full,
		U_Reference,
		U_ExtForces,
		ActiveLQRModel
	))
	{
		UE_LOG(
			LogPhysicsSandbox,
			Error,
			TEXT("Failed to design initial LQR controller.")
		);

		return;
	}
}

void AMultiSystemsActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (bIsSimulationRunning)
	{
		RunSimFixedSteps(DeltaTime);
	}

	DrawSystem();
	PrintInfo();
}

// -----------------------------------------------------------------------------
// Input
// -----------------------------------------------------------------------------

void AMultiSystemsActor::ToggleSimulation()
{
	bIsSimulationRunning = !bIsSimulationRunning;

	/*if (bEnableCsvLogging)
	{
		if (bIsSimulationRunning)
		{
			CsvLogger.Reset();

			LogCurrentSample();
		}
		else
		{
			FlushCsvLog();
		}
	}*/
}

void AMultiSystemsActor::ConfigureIntegrator()
{
	switch (IntegratorType)
	{
	case EIntegratorType::ExplicitEuler:
		FullSys.SetIntegrator(
			MakeUnique<FExplicitEulerSysIntegrator>()
		);
		break;

	case EIntegratorType::SemiImplicitEuler:
		FullSys.SetIntegrator(
			MakeUnique<FSemiImplicitEulerSysIntegrator>()
		);
		break;

	default:
		checkNoEntry();
		break;
	}
}

// -----------------------------------------------------------------------------
// System setup
// -----------------------------------------------------------------------------

void AMultiSystemsActor::BuildDemoSystem()
{
	// -------------------------------------------------------------------------
	// Subsystem
	// -------------------------------------------------------------------------

	FSubSys& TripleInvertedPendulum =
		FullSys.CreateSubSys(
			TEXT("TripleInvertedPendulum")
		);

	// -------------------------------------------------------------------------
	// Bodies - Perfect inverted equilibrium
	// -------------------------------------------------------------------------

	constexpr double EquilibriumX = 2.50;

	const int32 BaseIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(EquilibriumX, -1.20)
			)
		);

	const int32 Mass1Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(EquilibriumX, -0.3960099503),
				1.4
			)
		);

	const int32 Mass2Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(EquilibriumX, 0.3512517174),
				1.0
			)
		);

	const int32 Mass3Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(EquilibriumX + 0.02, 1.2633920575),
				0.7
			)
		);

	// -------------------------------------------------------------------------
	// Rods
	// -------------------------------------------------------------------------

	const int32 Rod1Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				BaseIndex,
				Mass1Index
			)
		);

	const int32 Rod2Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Mass1Index,
				Mass2Index
			)
		);

	const int32 Rod3Index =
		FullSys.AddElement(
			MakeUnique<FPendulumRod>(
				Mass2Index,
				Mass3Index
			)
		);

	// -------------------------------------------------------------------------
	// Control Channels
	// -------------------------------------------------------------------------

	FLinearControlChannel Channel0(TEXT("Mass1Horizontal"));
	Channel0.AddForceContribution(Mass1Index, FVector2D(1.0, 0.0));
	ControlSys.AddLinearControlChannel(MoveTemp(Channel0));

	/*
	FLinearControlChannel Channel1(TEXT("Mass2Horizontal"));
	Channel1.AddForceContribution(Mass2Index, FVector2D(1.0, 0.0));
	ControlSys.AddLinearControlChannel(MoveTemp(Channel1));
	*/
	

	// -------------------------------------------------------------------------
	// External force inputs
	// -------------------------------------------------------------------------

	U_ExtForces.Reset();

	U_ExtForces.Add({
		Mass1Index,
		FVector2D::ZeroVector
		});

	// -------------------------------------------------------------------------
	// Subsystem associations - Bodies
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddBody(
		TEXT("Base"),
		BaseIndex
	);

	TripleInvertedPendulum.AddBody(
		TEXT("Mass1"),
		Mass1Index
	);

	TripleInvertedPendulum.AddBody(
		TEXT("Mass2"),
		Mass2Index
	);

	TripleInvertedPendulum.AddBody(
		TEXT("Mass3"),
		Mass3Index
	);

	// -------------------------------------------------------------------------
	// Subsystem associations - Rods
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddElement(
		TEXT("Rod1"),
		Rod1Index
	);

	TripleInvertedPendulum.AddElement(
		TEXT("Rod2"),
		Rod2Index
	);

	TripleInvertedPendulum.AddElement(
		TEXT("Rod3"),
		Rod3Index
	);

	// -------------------------------------------------------------------------
	// Initialization
	// -------------------------------------------------------------------------

	FullSys.Initialize();

	check(
		ControlSys.BuildControlAllocationG_FRMatrix(
			FullSys
		)
	);

	X_Reference_Full =
		FullSys.BodiesToFullStateStruct();

	U_Reference =
		Eigen::VectorXd::Zero(
			ControlSys.GetNumControlChannels()
		);
}

// -----------------------------------------------------------------------------
// Fixed step simulation
// -----------------------------------------------------------------------------

void AMultiSystemsActor::RunSimFixedSteps(double FrameDeltaTime)
{
	if (FixedTimeStep <= 0.0 || FrameDeltaTime <= 0.0)
	{
		return;
	}

	SimuWallClockTime += FrameDeltaTime;
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

		if (!RunControlLoopStep(FixedTimeStep))
		{
			bIsSimulationRunning = false;
			break;
		}

		++FixedStepCount;
		SimuPhysicalTime += FixedTimeStep;
		SimulationTimeDebt -= FixedTimeStep;
		++SubStepCount;

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

	SimulationDelay = SimuWallClockTime - SimuPhysicalTime;
}

bool AMultiSystemsActor::RunControlLoopStep(double Dt)
{
	if (Dt <= 0.0)
	{
		return false;
	}

	// Current state
	// -------------------------------------------------------------------------

	const FSysState X_k = FullSys.BodiesToFullStateStruct();

	const FStateDOFIndexMap& IndexMap = FullSys.GetStateIndexMap();

	const int32 SizeX_FR = IndexMap.X_FRToFullIndices.Num();

	// State error
	// -------------------------------------------------------------------------

	const Eigen::VectorXd DeltaX_Full =
		X_k.ToVector() - X_Reference_Full.ToVector();

	Eigen::VectorXd DeltaX_FR(SizeX_FR);

	for (int32 StateIndex_FR = 0; StateIndex_FR < SizeX_FR; ++StateIndex_FR)
	{
		const int32 StateIndex_Full = IndexMap.X_FRToFullIndices[StateIndex_FR];

		DeltaX_FR(StateIndex_FR) = DeltaX_Full(StateIndex_Full);
	}

	// FR -> ker
	// -------------------------------------------------------------------------

	const Eigen::VectorXd DeltaZ_ker =
		ActiveLQRModel.T_kerToFR.transpose() * DeltaX_FR;

	// LQR control law
	// -------------------------------------------------------------------------

	const Eigen::VectorXd DeltaU =
		-ActiveLQRModel.K * DeltaZ_ker;

	const Eigen::VectorXd U_Control = U_Reference + DeltaU;

	const Eigen::VectorXd Tau_FR =
		ControlSys.ComputeTau_FR(U_Control);

	// Control diagnostics
	// -------------------------------------------------------------------------

	const double* volatile DeltaZKerData = DeltaZ_ker.data();
	const double* volatile DeltaUData = DeltaU.data();
	const double* volatile UControlData = U_Control.data();

	const double DeltaXFRNorm = DeltaX_FR.norm();
	const double DeltaZKerNorm = DeltaZ_ker.norm();
	const double DeltaUNorm = DeltaU.norm();
	const double UControlNorm = U_Control.norm();
	const double TauFRNorm = Tau_FR.norm();

	// Physical simulation step
	// -------------------------------------------------------------------------

	FSysState X_kPlus1;

	if (!FullSys.Step(Dt, X_k, Tau_FR, U_ExtForces, X_kPlus1))
	{
		return false;
	}

	return true;
}

// -----------------------------------------------------------------------------
// Visualization and debug
// -----------------------------------------------------------------------------

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
	const TArray<FBody>& Bodies =
		FullSys.GetBodies();

	for (const FBody& Body : Bodies)
	{
		const double DebugRadius =
			ComputeBodyDebugRadius(
				Body.Mass
			);

		DrawDebugSphere(
			GetWorld(),
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
		FullSys.GetElements();

	for (const TUniquePtr<ISysElement>& Element : Elements)
	{
		if (!Element)
		{
			continue;
		}

		int32 BodyAIndex =
			INDEX_NONE;

		int32 BodyBIndex =
			INDEX_NONE;

		if (
			!Element->GetConnectedBodies(
				BodyAIndex,
				BodyBIndex
			)
			)
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
			GetWorld(),
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
				GetWorld(),
				ToWorld(EquilibriumPoint)
				+ FVector(
					0.0,
					0.0,
					-50.0
				),
				ToWorld(EquilibriumPoint)
				+ FVector(
					0.0,
					0.0,
					50.0
				),
				FColor::Blue,
				false,
				0.0f,
				0,
				2.0f
			);
		}
	}
}

void AMultiSystemsActor::PrintInfo() const
{
	if (!GEngine)
	{
		return;
	}

	FString DebugText;

	DebugText +=
		TEXT("MULTI SYSTEMS\n");

	// =========================================================================
	// Input / camera info
	// =========================================================================

	const TCHAR* CameraMode =
		TEXT("Unknown");

	if (SimuCameraController)
	{
		CameraMode =
			SimuCameraController->IsUsing1AxisCamera()
			? TEXT("1Axis")
			: TEXT("Free");
	}

	DebugText += FString::Printf(
		TEXT(
			"SPACE Start/Stop | Camera C: %s\n"
		),
		CameraMode
	);

	// =========================================================================
	// Simulation time
	// =========================================================================

	const double RealTimeFactor =
		SimuWallClockTime > UE_SMALL_NUMBER
		? SimuPhysicalTime / SimuWallClockTime
		: 0.0;

	DebugText += FString::Printf(
		TEXT(
			"Sim: %.2f s | Wall: %.2f s | Delay: %+.3f s | RTF: %.3f | FixedSteps: %llu | FixedDt: %.5f s\n"
			"AvgFrameDt: %.4f s | LastSubSteps: %d\n"
		),
		SimuPhysicalTime,
		SimuWallClockTime,
		SimulationDelay,
		RealTimeFactor,
		FixedStepCount,
		FixedTimeStep,
		AverageDeltaTime,
		LastSubStepCount
	);

	// =========================================================================
	// Rod diagnostics
	// =========================================================================

	/*
	const TArray<FPendulumRod*>& RodSystemArray =
		FullSys.GetRodSystemArray();

	const TArray<double>& RodSystemLambda =
		FullSys.GetRodSystemLambdaVector();

	check(
		RodSystemArray.Num()
		== RodSystemLambda.Num()
	);

	for (int32 Rod_iIndex = 0; Rod_iIndex < RodSystemArray.Num(); ++Rod_iIndex)
	{
		const FPendulumRod* Rod_i =
			RodSystemArray[Rod_iIndex];

		if (!Rod_i)
		{
			continue;
		}

		const FInstantErrorData& InstantErrorData_i =
			Rod_i->GetInstantErrorData();

		const double Lambda_i =
			RodSystemLambda[Rod_iIndex];

		DebugText += FString::Printf(
			TEXT(
				"Rod %d | L Rel Error: %.3e | Radial Vel: %.3e m/s | Lambda: %.3e\n"
			),
			Rod_iIndex + 1,
			InstantErrorData_i.LengthRelError,
			InstantErrorData_i.RadialVelocityError,
			Lambda_i
		);
	}
	*/

	// =========================================================================
	// Energy balance
	// =========================================================================

	const FSystemEnergyBalance& Balance =
		FullSys.GetEnergyBalance();

	const FSystemEnergyState& CurrentEnergyState =
		Balance.Current;

	const FConstraintCorrectionEnergyDiagnostics& CorrectionBalance =
		FullSys.GetCorrectionEnergyDiagnostics();

	const double EnergyResidual =
		Balance.EnergyResidualSinceStart();

	const double EnergyResidualExcludingCorrections =
		EnergyResidual - CorrectionBalance.AccumulatedProjectionEnergy();

	DebugText += FString::Printf(
		TEXT(
			"\nENERGY BALANCE\n"
			"Initial Mech: %.6f J | Mechanical: %.6f J | Delta Mech: %+.6f J | Dissipated: %.6f J\n"
			"Control Work: %+.6f J | Accum Correction: %+.6f J | External Work: NOT IMPLEMENTED\n"
			"Energy Residual: %+.6f J | Residual Excl. Corrections: %+.6f J\n"
		),
		Balance.Initial.Mechanical(),
		CurrentEnergyState.Mechanical(),
		Balance.MechanicalVariationSinceStart(),
		CurrentEnergyState.TotalDissipated,

		CurrentEnergyState.TotalControlWork,
		CorrectionBalance.AccumulatedProjectionEnergy(),

		EnergyResidual,
		EnergyResidualExcludingCorrections
	);

	// =========================================================================
	// Energy components
	// =========================================================================

	/*
	DebugText += FString::Printf(
		TEXT(
			"\nENERGY COMPONENTS\n"
			"Kinetic: %.6f J | Gravitational: %.6f J | Elastic: %.6f J\n"
		),
		CurrentEnergyState.Kinetic,
		CurrentEnergyState.Gravitational,
		CurrentEnergyState.Elastic
	);
	*/

	// =========================================================================
	// Current step energy increments
	// =========================================================================

	/*
	const FDeltaEnergy& IntegrationBalance =
		FullSys.GetIntegrationEnergy();

	DebugText += FString::Printf(
		TEXT(
			"\nSTEP ENERGY INCREMENTS\n"
			"Integration: %+.6f J | Vel1: %+.6f J | Pos: %+.6f J | Vel2: %+.6f J | "
			"Total Correction: %+.6f J | Total Step: %+.6f J\n"
		),
		IntegrationBalance.DeltaMechanical(),
		CorrectionBalance.InitialVelocityProjection.DeltaMechanical(),
		CorrectionBalance.PositionProjection.DeltaMechanical(),
		CorrectionBalance.FinalVelocityProjection.DeltaMechanical(),
		CorrectionBalance.StepProjectionEnergyIncrement(),
		IntegrationBalance.DeltaMechanical()
			+ CorrectionBalance.StepProjectionEnergyIncrement()
	);
	*/

	// =========================================================================
	// Detailed accumulated correction energy
	// =========================================================================

	/*
	DebugText += FString::Printf(
		TEXT(
			"\nACCUMULATED CORRECTION ENERGY\n"
			"Vel1: %+.6f J | Pos: %+.6f J | Vel2: %+.6f J | Total: %+.6f J\n"
		),
		CorrectionBalance.AccumulatedInitialVelocity.Mechanical(),
		CorrectionBalance.AccumulatedPosition.Mechanical(),
		CorrectionBalance.AccumulatedFinalVelocity.Mechanical(),
		CorrectionBalance.AccumulatedProjectionEnergy()
	);
	*/

	// =========================================================================
	// LQR design timing - To be implemented
	// =========================================================================

	/*
	DebugText += FString::Printf(
		TEXT(
			"\nLQR DESIGN\n"
			"Kernel: %.3f ms | A: %.3f ms | B: %.3f ms | Controllability: %.3f ms | "
			"DARE: %.3f ms | K: %.3f ms | Total: %.3f ms\n"
		),
		...
	);
	*/

	const FColor TextColor =
		bIsSimulationRunning
		? FColor::Green
		: FColor::Red;

	GEngine->AddOnScreenDebugMessage(
		0,
		0.0f,
		TextColor,
		DebugText
	);
}

// -----------------------------------------------------------------------------
// Debug Helpers
// -----------------------------------------------------------------------------

double AMultiSystemsActor::ComputeBodyDebugRadius(
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

void AMultiSystemsActor::DrawStatePerturbation(
	const Eigen::VectorXd& DeltaX_Full,
	double PositionScale,
	double VelocityScale
) const
{
	const TArray<FBody>& Bodies =
		FullSys.GetBodies();

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

void AMultiSystemsActor::DrawStatePerturbationAtBody(
	int32 BodyIndex,
	const Eigen::VectorXd& DeltaX_Full,
	double PositionScale,
	double VelocityScale
) const
{
	const TArray<FBody>& Bodies =
		FullSys.GetBodies();

	check(
		Bodies.IsValidIndex(BodyIndex)
	);

	const int32 SizeQ_Full =
		FullSys.GetStateIndexMap().SizeQ_Full;

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
		GetWorld(),
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
		GetWorld(),
		ToWorld(DeltaPosScaled),
		4.0f,
		12,
		FColor::Yellow,
		false,
		0.0f
	);

	DrawDebugDirectionalArrow(
		GetWorld(),
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

void AMultiSystemsActor::DrawKernelZVector(
	const Eigen::VectorXd& Z_ker
) const
{
	Eigen::MatrixXd J_FR;
	Eigen::MatrixXd JDot_FR;
	Eigen::MatrixXd H_X_FR;
	Eigen::MatrixXd T_kerToFR;

	if (!ControlSys.BuildJandJDot_FR(
		FullSys,
		J_FR,
		JDot_FR
	))
	{
		return;
	}

	if (!ControlSys.BuildH_FR(
		J_FR,
		JDot_FR,
		H_X_FR
	))
	{
		return;
	}

	if (!ControlSys.BuildKernelBasis_FR(
		H_X_FR,
		T_kerToFR
	))
	{
		return;
	}

	check(
		Z_ker.size()
		== T_kerToFR.cols()
	);

	const Eigen::VectorXd DeltaX_FR =
		ControlSys.TransformDeltaX_ker2FR(
			T_kerToFR,
			Z_ker
		);

	const Eigen::VectorXd DeltaX_Full =
		ControlSys.ExpandDeltaX_FRToFull(
			FullSys.GetStateIndexMap(),
			DeltaX_FR
		);

	constexpr double PositionScale = 0.5;
	constexpr double VelocityScale = 0.5;

	DrawStatePerturbation(
		DeltaX_Full,
		PositionScale,
		VelocityScale
	);
}

void AMultiSystemsActor::DrawKernelZVectorAtBody(
	int32 BodyIndex,
	const Eigen::VectorXd& Z_ker
) const
{
	Eigen::MatrixXd J_FR;
	Eigen::MatrixXd JDot_FR;
	Eigen::MatrixXd H_X_FR;
	Eigen::MatrixXd T_kerToFR;

	if (!ControlSys.BuildJandJDot_FR(
		FullSys,
		J_FR,
		JDot_FR
	))
	{
		return;
	}

	if (!ControlSys.BuildH_FR(
		J_FR,
		JDot_FR,
		H_X_FR
	))
	{
		return;
	}

	if (!ControlSys.BuildKernelBasis_FR(
		H_X_FR,
		T_kerToFR
	))
	{
		return;
	}

	check(
		Z_ker.size()
		== T_kerToFR.cols()
	);

	const Eigen::VectorXd DeltaX_FR =
		ControlSys.TransformDeltaX_ker2FR(
			T_kerToFR,
			Z_ker
		);

	const Eigen::VectorXd DeltaX_Full =
		ControlSys.ExpandDeltaX_FRToFull(
			FullSys.GetStateIndexMap(),
			DeltaX_FR
		);

	constexpr double PositionScale = 0.5;
	constexpr double VelocityScale = 0.5;

	DrawStatePerturbationAtBody(
		BodyIndex,
		DeltaX_Full,
		PositionScale,
		VelocityScale
	);
}
