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
	// Bodies - Pendulum
	// -------------------------------------------------------------------------

	const int32 BaseIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(2.50, -1.20)
			)
		);

	const int32 Mass1Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(2.58, -0.40),
				1.4
			)
		);

	const int32 Mass2Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(2.38, 0.32),
				1.0
			)
		);

	const int32 Mass3Index =
		FullSys.AddBody(
			FBody::Free(
				FVector2D(2.62, 1.20),
				0.7
			)
		);

	const int32 LeftAnchorIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(1.14, 1.20)
			)
		);

	const int32 RightAnchorIndex =
		FullSys.AddBody(
			FBody::Fixed(
				FVector2D(3.86, 1.20)
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
	// Upper stabilization springs
	// -------------------------------------------------------------------------

	const int32 LeftSpringIndex =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				LeftAnchorIndex,
				Mass3Index,
				35.0,
				0.15,
				1.48
			)
		);

	const int32 RightSpringIndex =
		FullSys.AddElement(
			MakeUnique<FSpringDamper>(
				RightAnchorIndex, 
				Mass3Index,
				35.0,
				0.15,
				1.48
			)
		);
	
	// -------------------------------------------------------------------------
	// Control Channels
	// -------------------------------------------------------------------------

	// Channel 1
	FLinearControlChannel Channel0(TEXT("Channel0"));

	Channel0.AddForceContribution(
		Mass1Index,
		FVector2D(1.0, 0.5)
	);

	Channel0.AddForceContribution(
		Mass3Index,
		FVector2D(-1.0, 0.0)
	);

	ControlSys.AddLinearControlChannel(
		MoveTemp(Channel0)
	);

	// Channel 2
	FLinearControlChannel Channel1(TEXT("Channel1"));

	Channel1.AddForceContribution(
		Mass2Index,
		FVector2D(0.0, 1.0)
	);

	Channel1.AddForceContribution(
		Mass3Index,
		FVector2D(0.5, -1.0)
	);

	ControlSys.AddLinearControlChannel(
		MoveTemp(Channel1)
	);

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

	TripleInvertedPendulum.AddBody(
		TEXT("LeftAnchor"),
		LeftAnchorIndex
	);

	TripleInvertedPendulum.AddBody(
		TEXT("RightAnchor"),
		RightAnchorIndex
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
	// Subsystem associations - Springs
	// -------------------------------------------------------------------------

	TripleInvertedPendulum.AddElement(
		TEXT("LeftSpring"),
		LeftSpringIndex
	);

	TripleInvertedPendulum.AddElement(
		TEXT("RightSpring"),
		RightSpringIndex
	);

	// -------------------------------------------------------------------------
	// Initialization
	// -------------------------------------------------------------------------

	// Must be called after all bodies and elements have been added.
	FullSys.Initialize();

	check(
		ControlSys.BuildControlAllocationG_FRMatrix(
			FullSys
		)
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

	// Nominal state and control
	// -------------------------------------------------------------------------

	const FSysState X_k =
		FullSys.BodiesToFullStateStruct();

	Eigen::VectorXd U_Control =
		Eigen::VectorXd::Zero(ControlSys.GetNumControlChannels());

	const Eigen::VectorXd Tau_FR =
		ControlSys.ComputeTau_FR(U_Control);

	// Constraint tangent space
	// -------------------------------------------------------------------------

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
		return false;
	}

	if (!ControlSys.BuildH_FR(
		J_FR,
		JDot_FR,
		H_X_FR
	))
	{
		return false;
	}

	if (!ControlSys.BuildKernelBasis_FR(
		H_X_FR,
		T_kerToFR
	))
	{
		return false;
	}

	// Discrete FR linearization
	// -------------------------------------------------------------------------

	constexpr double LinearizationEpsilonX = 1.0e-5;
	constexpr double LinearizationEpsilonU = 1.0e-5;

	Eigen::MatrixXd A_FR_Discrete;
	Eigen::MatrixXd B_FR_Discrete;

	if (!ControlSys.ComputeA_FR_Discrete(
		FullSys,
		Dt,
		X_k,
		Tau_FR,
		U_ExtForces,
		LinearizationEpsilonX,
		A_FR_Discrete
	))
	{
		return false;
	}

	if (!ControlSys.ComputeB_FR_Discrete(
		FullSys,
		Dt,
		X_k,
		U_Control,
		U_ExtForces,
		LinearizationEpsilonU,
		B_FR_Discrete
	))
	{
		return false;
	}

	// FR -> ker reduced model
	// -------------------------------------------------------------------------

	const Eigen::MatrixXd A_ker_Discrete =
		T_kerToFR.transpose() * A_FR_Discrete * T_kerToFR;

	const Eigen::MatrixXd B_ker_Discrete =
		T_kerToFR.transpose() * B_FR_Discrete;

	// Controllability
	// -------------------------------------------------------------------------

	FControllabilityResult ControllabilityResult;

	if (!ControlSys.AnalyzeControllability(
		A_ker_Discrete,
		B_ker_Discrete,
		ControllabilityResult
	))
	{
		return false;
	}

	/// LQR cost definition
	// -----------------------------------------------------------------------------

	const FStateDOFIndexMap& IndexMap =
		FullSys.GetStateIndexMap();

	const int32 SizeQ_FR = IndexMap.SizeQ_FR;

	const int32 SizeX_FR = IndexMap.X_FRToFullIndices.Num();

	const int32 NumControlChannels = ControlSys.GetNumControlChannels();

	constexpr double PositionStateWeight = 10.0;
	constexpr double VelocityStateWeight = 1.0;
	constexpr double ControlWeight = 1.0;

	Eigen::MatrixXd Q_FR = Eigen::MatrixXd::Zero(SizeX_FR, SizeX_FR);

	for (int32 QIndex_FR = 0; QIndex_FR < SizeQ_FR; ++QIndex_FR)
	{
		Q_FR(QIndex_FR, QIndex_FR) = PositionStateWeight;

		const int32 QDotIndex_FR =
			SizeQ_FR + QIndex_FR;

		Q_FR(QDotIndex_FR, QDotIndex_FR) = VelocityStateWeight;
	}

	const Eigen::MatrixXd Q_ker =
		T_kerToFR.transpose() * Q_FR * T_kerToFR;

	const Eigen::MatrixXd R =
		ControlWeight
		* Eigen::MatrixXd::Identity(NumControlChannels, NumControlChannels);

	// Solve Discrete Algebraic Riccati Equation (DARE)
	// -----------------------------------------------------------------------------

	FDAREResult DAREResult;

	if (!ControlSys.SolveDARE(A_ker_Discrete, B_ker_Discrete, Q_ker, R, DAREResult))
	{
		return false;
	}

	const Eigen::MatrixXd& P = DAREResult.P;

	const int32 PRows = static_cast<int32>(P.rows());
	const int32 PCols = static_cast<int32>(P.cols());

	const double PNorm = P.norm();

	const double DARERelativeResidual =
		DAREResult.Residual / FMath::Max(1.0, P.norm());

	const double PSymmetryError =
		(P - P.transpose()).norm();

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> PEigenSolver(P);

	if (PEigenSolver.info() != Eigen::Success)
	{
		return false;
	}

	const Eigen::VectorXd PEigenvalues =
		PEigenSolver.eigenvalues();

	const double PMinEigenvalue =
		PEigenvalues.minCoeff();

	const double PMaxEigenvalue =
		PEigenvalues.maxCoeff();

	// LQR gain
	// -------------------------------------------------------------------------

	// K

	// Control law
	// -------------------------------------------------------------------------

	// DeltaX_FR
	// DeltaZ_ker
	// DeltaU = -K * DeltaZ_ker
	// U_Control = NominalU_Control + DeltaU
	// Tau_FR = ControlSys.ComputeTau_FR(U_Control)

	// Physical simulation step
	// -------------------------------------------------------------------------

	FSysState X_kPlus1;

	if (!FullSys.Step(
		Dt,
		X_k,
		Tau_FR,
		U_ExtForces,
		X_kPlus1
	))
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

	const TCHAR* CameraMode =
		TEXT("Unknown");

	if (SimuCameraController)
	{
		CameraMode =
			SimuCameraController->IsUsing1AxisCamera()
			? TEXT("1Axis")
			: TEXT("Free");
	}

	/*
	DebugText += FString::Printf(
		TEXT(
			"SPACE Start/Stop\n"
			"Key 'C' Camera Toggle: %s\n"
			"SimulTime: %.2f s | FixedSteps: %llu\n"
			"Delay: %.3f s | AvgDt: %.3f s | FixedDt: %.4f s\n"
		),
		CameraMode,
		SimuPhysicalTime,
		FixedStepCount,
		SimulationDelay,
		AverageDeltaTime,
		FixedTimeStep
	);
	*/

	
	DebugText += FString::Printf(
		TEXT("SimulTime: %.2f s | FixedSteps: %llu\n"),

		SimuPhysicalTime, FixedStepCount
	);
	

	const TArray<FPendulumRod*>& RodSystemArray =
		FullSys.GetRodSystemArray();

	const TArray<double>& RodSystemLambda =
		FullSys.GetRodSystemLambdaVector();

	check(
		RodSystemArray.Num()
		== RodSystemLambda.Num()
	);

	for (
		int32 Rod_iIndex = 0;
		Rod_iIndex < RodSystemArray.Num();
		++Rod_iIndex
		)
	{
		const FPendulumRod* Rod_i =
			RodSystemArray[Rod_iIndex];

		if (!Rod_i)
		{
			continue;
		}

		const FInstantErrorData& InstantErrorData_i =
			Rod_i->GetInstantErrorData();

		const FStatisticalErrorData& StatisticalErrorData_i =
			Rod_i->GetStatisticalErrorData();

		const double Lambda_i =
			RodSystemLambda[Rod_iIndex];

		/*
		DebugText += FString::Printf(
			TEXT(
				"--------------------------------------------------\n"
				"ROD %d\n"

				"L Rel Error: %.3e   ||   Radial Vel: %.3e m/s\n"
			),

			Rod_iIndex + 1,

			InstantErrorData_i.LengthRelError,
			InstantErrorData_i.RadialVelocityError
		); */
	}

	const FSystemEnergyBalance& Balance = FullSys.GetEnergyBalance();
	const FSystemEnergyState& CurrentEnergyState = Balance.Current;
	const FDeltaEnergy& IntegrationBalance = FullSys.GetIntegrationEnergy();
	const FConstraintCorrectionEnergyDiagnostics& CorrectionBalance = FullSys.GetCorrectionEnergyDiagnostics();

	DebugText += FString::Printf(
		TEXT(
			"\n==================================================\n"
			"ENERGY BALANCE\n\n"

			"Initial Mechanical: %.6f J\n"
			"Mechanical: %.6f J\n"
			"Kinetic: %.6f J\n"
			"Gravitational: %.6f J\n"
			"Elastic: %.6f J\n"
			"\n"
			"Delta Mechanical: %+.6f J\n"
			"Total Dissipated: %.6f J\n"
			"Energy Residual: %+.6f J\n"
		),
		Balance.Initial.Mechanical(),
		CurrentEnergyState.Mechanical(),
		CurrentEnergyState.Kinetic,
		CurrentEnergyState.Gravitational,
		CurrentEnergyState.Elastic,

		Balance.MechanicalVariationSinceStart(),
		CurrentEnergyState.TotalDissipated,
		Balance.EnergyResidualSinceStart()
	);

	DebugText += FString::Printf(
		TEXT(
			"\n--------------------------------------------------\n"
			"STEP ENERGY INCREMENTS\n\n"

			"Integration: %+.6f J\n"
			"1º Vel Correction: %+.6f J\n"
			"Pos Correction: %+.6f J\n"
			"2º Vel Correction: %+.6f J\n"
			"Total Correction: %+.6f J\n"
			"Total Step: %+.6f J\n"
		),
		IntegrationBalance.DeltaMechanical(),
		CorrectionBalance.InitialVelocityProjection.DeltaMechanical(),
		CorrectionBalance.PositionProjection.DeltaMechanical(),
		CorrectionBalance.FinalVelocityProjection.DeltaMechanical(),
		CorrectionBalance.StepProjectionEnergyIncrement(),

		IntegrationBalance.DeltaMechanical()
		+ CorrectionBalance.StepProjectionEnergyIncrement()
	);

	DebugText += FString::Printf(
		TEXT(
			"\n--------------------------------------------------\n"
			"ACCUMULATED CORRECTION ENERGY\n\n"

			"1º Vel Correction: %+.6f J\n"
			"Pos Correction: %+.6f J\n"
			"2º Vel Correction: %+.6f J\n"
			"Total Correction: %+.6f J\n"
			"\n"
			"Residual Excluding Projections: %+.6f J\n"
		),
		CorrectionBalance.AccumulatedInitialVelocity.Mechanical(),
		CorrectionBalance.AccumulatedPosition.Mechanical(),
		CorrectionBalance.AccumulatedFinalVelocity.Mechanical(),
		CorrectionBalance.AccumulatedProjectionEnergy(),

		Balance.EnergyResidualSinceStart()
		- CorrectionBalance.AccumulatedProjectionEnergy()
	);

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
